/*
 * PMU support
 *
 * Copyright (C) 2015 Red Hat Inc.
 * Copyright (C) 2016 ARM Ltd.
 * Author: Mark Salter <msalter@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#define pr_fmt(fmt) "ACPI-PMU: " fmt
#include <linux/perf/arm_pmu.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>

#include <asm/cpu.h>

#define PMU_PDEV_NAME "armv8-pmu"

struct pmu_irq {
	int  gsi;
	int  trigger;
	bool registered;
};

struct pmu_types {
	int cpu_type;
	int cpu_count;
};

static struct pmu_irq pmu_irqs[NR_CPUS] __initdata;

/*
 * called from acpi_map_gic_cpu_interface()'s MADT parsing callback during boot
 * this routine saves off the GSI's and their trigger state for use when we are
 * ready to build the PMU platform device.
 */
void __init arm_pmu_parse_acpi(int cpu, struct acpi_madt_generic_interrupt *gic)
{
	pmu_irqs[cpu].gsi = gic->performance_interrupt;
	if (gic->flags & ACPI_MADT_PERFORMANCE_IRQ_MODE)
		pmu_irqs[cpu].trigger = ACPI_EDGE_SENSITIVE;
	else
		pmu_irqs[cpu].trigger = ACPI_LEVEL_SENSITIVE;
	pr_info("Assign CPU %d girq %d level %d\n", cpu, pmu_irqs[cpu].gsi,
						   pmu_irqs[cpu].trigger);
}

/* count number and type of CPU's in system */
static void __init arm_pmu_acpi_determine_cpu_types(struct pmu_types *pmus)
{
	int i, j;

	for_each_possible_cpu(i) {
		struct cpuinfo_arm64 *cinfo = per_cpu_ptr(&cpu_data, i);

		pr_devel("Present CPU %d is a %X\n", i,
					       MIDR_PARTNUM(cinfo->reg_midr));
		for (j = 0; j < NR_CPUS; j++) {
			if (pmus[j].cpu_type == MIDR_PARTNUM(cinfo->reg_midr)) {
				pmus[j].cpu_count++;
				break;
			}
			if (pmus[j].cpu_count == 0) {
				pmus[j].cpu_type = MIDR_PARTNUM(cinfo->reg_midr);
				pmus[j].cpu_count++;
				break;
			}
		}
	}
}

static int __init arm_pmu_acpi_register_pmu(int count, struct resource *res,
					    int last_cpu_id)
{
	int i;
	int err = -ENOMEM;
	bool free_gsi = false;
	struct platform_device *pdev;

	if (count) {
		pdev = platform_device_alloc(PMU_PDEV_NAME, last_cpu_id);

		if (pdev) {
			err = platform_device_add_resources(pdev,
							    res, count);
			if (!err) {
				err = platform_device_add(pdev);
				if (err) {
					pr_warn("Unable to register PMU device\n");
					free_gsi = true;
				}
			} else {
				pr_warn("Unable to add resources to device\n");
				free_gsi = true;
				platform_device_put(pdev);
			}
		} else {
			pr_warn("Unable to allocate platform device\n");
			free_gsi = true;
		}
	}

	/* unmark (and possibly unregister) registered GSIs */
	for_each_possible_cpu(i) {
		if (pmu_irqs[i].registered) {
			if (free_gsi)
				acpi_unregister_gsi(pmu_irqs[i].gsi);
			pmu_irqs[i].registered = false;
		}
	}

	return err;
}

/*
 * For the given cpu/pmu type, walk all known GSIs, register them, and add
 * them to the resource structure. Return the number of GSI's contained
 * in the res structure, and the id of the last CPU/PMU we added.
 */
static int __init arm_pmu_acpi_gsi_res(struct pmu_types *pmus,
				       struct resource *res, int *last_cpu_id)
{
	int i, count;
	int irq;

	pr_info("Setting up %d PMUs for CPU type %X\n", pmus->cpu_count,
							pmus->cpu_type);
	/* lets group all the PMU's from similar CPU's together */
	count = 0;
	for_each_possible_cpu(i) {
		struct cpuinfo_arm64 *cinfo = per_cpu_ptr(&cpu_data, i);

		if (pmus->cpu_type == MIDR_PARTNUM(cinfo->reg_midr)) {
			pr_devel("Setting up CPU %d\n", i);
			if (pmu_irqs[i].gsi == 0)
				continue;

			irq = acpi_register_gsi(NULL, pmu_irqs[i].gsi,
						pmu_irqs[i].trigger,
						ACPI_ACTIVE_HIGH);

			res[count].start = res[count].end = irq;
			res[count].flags = IORESOURCE_IRQ;

			if (pmu_irqs[i].trigger == ACPI_EDGE_SENSITIVE)
				res[count].flags |= IORESOURCE_IRQ_HIGHEDGE;
			else
				res[count].flags |= IORESOURCE_IRQ_HIGHLEVEL;

			pmu_irqs[i].registered = true;
			count++;
			(*last_cpu_id) = cinfo->reg_midr;

			if (irq_is_percpu(irq))
				pr_debug("PPI detected\n");
		}
	}
	return count;
}

static int __init pmu_acpi_init(void)
{
	struct resource	*res;
	int err = -ENOMEM;
	int count;
	int j, last_cpu_id;
	struct pmu_types *pmus;

	pr_debug("Prepare registration\n");
	if (acpi_disabled)
		return 0;

	pmus = kcalloc(NR_CPUS, sizeof(struct pmu_types), GFP_KERNEL);

	if (pmus) {
		arm_pmu_acpi_determine_cpu_types(pmus);

		for (j = 0; pmus[j].cpu_count; j++) {
			pr_devel("CPU type %d, count %d\n", pmus[j].cpu_type,
				 pmus[j].cpu_count);
			res = kcalloc(pmus[j].cpu_count,
				      sizeof(struct resource), GFP_KERNEL);

			/* for a given PMU type collect all the GSIs. */
			if (res) {
				count = arm_pmu_acpi_gsi_res(&pmus[j], res,
							     &last_cpu_id);
				/*
				 * register this set of interrupts
				 * with a new PMU device
				 */
				err = arm_pmu_acpi_register_pmu(count,
								res,
								last_cpu_id);
				kfree(res);
			} else
				pr_warn("PMU unable to allocate interrupt resource space\n");
		}

		kfree(pmus);
	} else
		pr_warn("PMU: Unable to allocate pmu count structures\n");

	return err;
}

arch_initcall(pmu_acpi_init);
