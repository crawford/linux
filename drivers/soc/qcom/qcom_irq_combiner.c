/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Driver for interrupt combiners in the Top-level Control and Status
 * Registers (TCSR) hardware block in Qualcomm Technologies chips.
 * An interrupt combiner in this block combines a set of interrupts by
 * OR'ing the individual interrupt signals into a summary interrupt
 * signal routed to a parent interrupt controller, and provides read-
 * only, 32-bit registers to query the status of individual interrupts.
 * The status bit for IRQ n is bit (n % 32) within register (n / 32)
 * of the given combiner. Thus, each combiner can be described as a set
 * of register offsets and the number of IRQs managed.
 *
 * The combiner driver is implemented as a gpio_chip to allow consumers
 * to find their IRQs using devm_gpiod_get_index/gpiod_to_irq under ACPI.
 * This is done because the ACPI core does not use the Resource Source
 * field on the Extended Interrupt Descriptor, which in theory could be
 * used to specify an alternative IRQ controller similar to the way
 * interrupt-parent is used in DT/OF. Under ACPI boot, consumers will
 * specify their IRQs as GpioInt resources, which can be found with
 * devm_gpiod_get_index, and then mapped to irqs by calling gpiod_to_irq.
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#define IRQS_PER_REGISTER 32

struct combiner_reg {
	void __iomem *addr;
	unsigned long mask;
};

struct combiner {
	struct gpio_chip    gpio_chip;
	struct irq_chip     irq_chip;
	int                 parent_irq;
	u32                 nr_regs;
	struct combiner_reg regs[0];
};

static inline u32 nr_irqs_to_nr_regs(u32 nr_irqs)
{
	return DIV_ROUND_UP(nr_irqs, IRQS_PER_REGISTER);
}

static inline u32 irq_register(int irq)
{
	return irq / IRQS_PER_REGISTER;
}

static inline u32 irq_bit(int irq)
{
	return irq % IRQS_PER_REGISTER;

}

static inline int irq_nr(u32 reg, u32 bit)
{
	return reg * IRQS_PER_REGISTER + bit;
}

static inline struct combiner *to_combiner(struct gpio_chip *gpio_chip)
{
	return container_of(gpio_chip, struct combiner, gpio_chip);
}

/*
 * Handler for the cascaded IRQ.
 */
static void combiner_handle_irq(struct irq_desc *desc)
{
	struct gpio_chip *gpio_chip = irq_desc_get_handler_data(desc);
	struct combiner *combiner = to_combiner(gpio_chip);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 reg;

	chained_irq_enter(chip, desc);

	for (reg = 0; reg < combiner->nr_regs; reg++) {
		int virq;
		int hwirq;
		u32 bit;
		u32 status;

		if (combiner->regs[reg].mask == 0)
			continue;

		status = readl_relaxed(combiner->regs[reg].addr);
		status &= combiner->regs[reg].mask;

		while (status) {
			bit = __ffs(status);
			status &= ~(1 << bit);
			hwirq = irq_nr(reg, bit);
			virq = irq_find_mapping(gpio_chip->irqdomain, hwirq);
			if (virq >= 0)
				generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

/*
 * irqchip callbacks
 */

static void combiner_irq_chip_mask_irq(struct irq_data *data)
{
	struct gpio_chip *gpio_chip = irq_data_get_irq_chip_data(data);
	struct combiner *combiner = to_combiner(gpio_chip);
	struct combiner_reg *reg = combiner->regs + irq_register(data->hwirq);

	clear_bit(irq_bit(data->hwirq), &reg->mask);
}

static void combiner_irq_chip_unmask_irq(struct irq_data *data)
{
	struct gpio_chip *gpio_chip = irq_data_get_irq_chip_data(data);
	struct combiner *combiner = to_combiner(gpio_chip);
	struct combiner_reg *reg = combiner->regs + irq_register(data->hwirq);

	set_bit(irq_bit(data->hwirq), &reg->mask);
}

#ifdef CONFIG_SMP
static int combiner_irq_chip_set_affinity(struct irq_data *data,
					 const struct cpumask *mask, bool force)
{
	struct gpio_chip *gpio_chip = irq_data_get_irq_chip_data(data);
	struct combiner *combiner = to_combiner(gpio_chip);
	struct irq_chip *parent_chip = irq_get_chip(combiner->parent_irq);
	struct irq_data *parent_data = irq_get_irq_data(combiner->parent_irq);

	if (parent_chip && parent_chip->irq_set_affinity)
		return parent_chip->irq_set_affinity(parent_data, mask, force);
	else
		return -EINVAL;
}
#endif

/*
 * gpiochip callbacks
 */

static
int combiner_pin_get_direction(struct gpio_chip *chip, unsigned offset)
{
	if (offset >= chip->ngpio)
		return -EINVAL;
	return GPIOF_DIR_IN;
}

static
int combiner_pin_direction_input(struct gpio_chip *chip, unsigned offset)
{
	/* Return success unconditionally all 'pins' are always inputs. */
	return 0;
}

static
int combiner_pin_direction_output(struct gpio_chip *chip, unsigned offset,
				  int value)
{
	/* Return error unconditionally all 'pins' are always inputs. */
	return -EINVAL;
}

static
int combiner_pin_get(struct gpio_chip *chip, unsigned offset)
{
	struct combiner *combiner = to_combiner(chip);
	struct combiner_reg *reg;
	u32 status;

	if (offset >= chip->ngpio)
		return -EINVAL;

	reg = combiner->regs + irq_register(offset);
	status = readl_relaxed(reg->addr);
	return (status & (1 << irq_bit(offset))) != 0;
}

static
void combiner_pin_set(struct gpio_chip *chip, unsigned offset, int value)
{
	/* No-op all 'pins' are always inputs. */
}

static
void combiner_pin_set_multiple(struct gpio_chip *chip, unsigned long *mask,
			       unsigned long *bits)
{
	/* No-op all 'pins' are always inputs. */
}

static void __iomem *tcsr_base;

#if CONFIG_OF

/*
 * DT/OF initialization uses the irqchip initialization framework.
 */

static int tcsr_ref_cnt;

struct of_combiner {
	struct device dev;
	struct combiner combiner;
};

static void tcsr_map(struct device_node *node)
{
	if (!tcsr_base)
		tcsr_base = of_iomap(node, 0);
	if (!IS_ERR(tcsr_base))
		tcsr_ref_cnt++;
}

static void tcsr_unmap(void)
{
	tcsr_ref_cnt--;
	if ((tcsr_ref_cnt == 0) && !IS_ERR(tcsr_base))
		iounmap(tcsr_base);
}

static int __init combiner_of_init(struct device_node *node,
				   struct device_node *parent)
{
	int err, i;
	u32 nr_irqs;
	u32 nr_regs;
	struct of_combiner *of_combiner;
	size_t alloc_sz;

	tcsr_map(node->parent);
	if (IS_ERR(tcsr_base)) {
		pr_err("%s: Error mapping memory resource.\n", node->name);
		return PTR_ERR(tcsr_base);
	}

	if (of_property_read_u32(node, "qcom,combiner-nr-irqs", &nr_irqs)) {
		pr_err("%s: Error reading number of IRQs.\n", node->name);
		err = -EINVAL;
		goto out_unmap;
	}

	if (nr_irqs == 0) {
		pr_err("%s: Invalid number of IRQs.\n", node->name);
		err = -EINVAL;
		goto out_unmap;
	}

	nr_regs = nr_irqs_to_nr_regs(nr_irqs);
	alloc_sz = sizeof(*of_combiner) + sizeof(struct combiner_reg) * nr_regs;
	of_combiner = kzalloc(alloc_sz, GFP_KERNEL);
	if (!of_combiner) {
		err = -ENOMEM;
		goto out_unmap;
	} else {
		u32 regs[nr_regs];

		if (of_property_read_u32_array(
			    node, "qcom,combiner-regs", regs, nr_regs)) {
			pr_err("%s: Error reading register array.\n",
			       node->name);
			err = -EINVAL;
			goto out_free_combiner;
		}

		of_combiner->combiner.parent_irq = of_irq_get(node, 0);
		if (of_combiner->combiner.parent_irq <= 0) {
			pr_err("%s: Error getting IRQ resource.\n", node->name);
			err = -EINVAL;
			goto out_free_combiner;
		}

		of_combiner->combiner.nr_regs = nr_regs;
		for (i = 0; i < nr_regs; i++) {
			of_combiner->combiner.regs[i].addr = tcsr_base + regs[i];
			of_combiner->combiner.regs[i].mask = 0;
		}
	}

	dev_set_name(&of_combiner->dev, node->name);
	of_combiner->dev.of_node = node;

	of_combiner->combiner.gpio_chip = (struct gpio_chip) {
		.base		  = -1,
		.ngpio		  = nr_irqs,
		.label		  = node->name,
		.parent		  = &of_combiner->dev,
		.owner		  = THIS_MODULE,
		.of_node	  = node,
		.get_direction	  = combiner_pin_get_direction,
		.direction_input  = combiner_pin_direction_input,
		.direction_output = combiner_pin_direction_output,
		.get		  = combiner_pin_get,
		.set		  = combiner_pin_set,
		.set_multiple	  = combiner_pin_set_multiple,
	};

	of_combiner->combiner.irq_chip = (struct irq_chip) {
		.irq_mask   = combiner_irq_chip_mask_irq,
		.irq_unmask = combiner_irq_chip_unmask_irq,
#ifdef CONFIG_SMP
		.irq_set_affinity = combiner_irq_chip_set_affinity,
#endif
	};

	if (of_property_read_string(node, "qcom,combiner-name",
				    &of_combiner->combiner.irq_chip.name) != 0)
		of_combiner->combiner.irq_chip.name = "qcom-irq-combiner";


	err = gpiochip_add(&of_combiner->combiner.gpio_chip);
	if (err) {
		pr_err("%s: Failed to register gpiochip\n", node->name);
		err = -EINVAL;
		goto out_free_combiner;
	}

	err = gpiochip_irqchip_add(&of_combiner->combiner.gpio_chip,
				   &of_combiner->combiner.irq_chip,
				   0, handle_level_irq, IRQ_TYPE_NONE);
	if (err) {
		pr_err("%s: Failed to add irqchip to gpiochip\n", node->name);
		goto out_remove;
	}

	for (i = 0; i < nr_irqs; i++)
		gpiochip_lock_as_irq(&of_combiner->combiner.gpio_chip, i);

	gpiochip_set_chained_irqchip(&of_combiner->combiner.gpio_chip,
				     &of_combiner->combiner.irq_chip,
				     of_combiner->combiner.parent_irq,
				     combiner_handle_irq);

	pr_info("%s: Initialized with [p=%d,n=%d,r=%p]\n", node->name,
		of_combiner->combiner.parent_irq, nr_irqs,
		of_combiner->combiner.regs[0].addr);

	return 0;

out_remove:
	gpiochip_remove(&of_combiner->combiner.gpio_chip);
out_free_combiner:
	kfree(of_combiner);
out_unmap:
	tcsr_unmap();
	return err;
}

OF_DECLARE_2(irqchip, qcom_irq_combiner, "qcom,irq-combiner", combiner_of_init);

#endif /* CONFIG_OF */

#ifdef CONFIG_ACPI

/*
 * ACPI initialization happens later in the boot process during device probing.
 */

/*
 * Combiner initialization and probing.
 */

static int __init combiner_probe(struct platform_device *pdev)
{
	int err, i;
	u32 nr_irqs = 0;
	u32 nr_regs;
	struct combiner *combiner;
	size_t alloc_sz;

	if (!tcsr_base)
		return -ENODEV;


	if (device_property_read_u32(&pdev->dev, "qcom,combiner-nr-irqs",
				     &nr_irqs)) {
		dev_err(&pdev->dev, "Error reading number of IRQs.\n");
		return -EINVAL;
	}

	if (nr_irqs == 0) {
		dev_err(&pdev->dev, "Invalid number of IRQs.\n");
		return -EINVAL;
	}

	nr_regs = nr_irqs_to_nr_regs(nr_irqs);
	alloc_sz = sizeof(*combiner) + sizeof(struct combiner_reg) * nr_regs;
	combiner = devm_kzalloc(&pdev->dev, alloc_sz, GFP_KERNEL);
	if (!combiner) {
		return -ENOMEM;
	} else {
		u32 regs[nr_regs];

		if (device_property_read_u32_array(
			    &pdev->dev, "qcom,combiner-regs", regs, nr_regs)) {
			dev_err(&pdev->dev, "Error reading register array.\n");
			return -EINVAL;
		}

		combiner->parent_irq = platform_get_irq(pdev, 0);
		if (combiner->parent_irq <= 0) {
			dev_err(&pdev->dev, "Error getting IRQ resource.\n");
			return -EINVAL;
		}

		combiner->nr_regs = nr_regs;
		for (i = 0; i < nr_regs; i++) {
			combiner->regs[i].addr = tcsr_base + regs[i];
			combiner->regs[i].mask = 0;
		}
	}

	combiner->gpio_chip = (struct gpio_chip) {
		.base		  = -1,
		.ngpio		  = nr_irqs,
		.label		  = dev_name(&pdev->dev),
		.parent		  = &pdev->dev,
		.owner		  = THIS_MODULE,
		.of_node	  = pdev->dev.of_node,
		.get_direction	  = combiner_pin_get_direction,
		.direction_input  = combiner_pin_direction_input,
		.direction_output = combiner_pin_direction_output,
		.get		  = combiner_pin_get,
		.set		  = combiner_pin_set,
		.set_multiple	  = combiner_pin_set_multiple,
	};

	combiner->irq_chip = (struct irq_chip) {
		.irq_mask   = combiner_irq_chip_mask_irq,
		.irq_unmask = combiner_irq_chip_unmask_irq,
#ifdef CONFIG_SMP
		.irq_set_affinity = combiner_irq_chip_set_affinity,
#endif
	};

	if (device_property_read_string(&pdev->dev, "qcom,combiner-name",
					&combiner->irq_chip.name) != 0)
		combiner->irq_chip.name = "qcom-irq-combiner";

	err = gpiochip_add(&combiner->gpio_chip);
	if (err) {
		dev_err(&pdev->dev, "Failed to register gpiochip\n");
		return err;
	}

	err = gpiochip_irqchip_add(&combiner->gpio_chip, &combiner->irq_chip,
				   0, handle_level_irq, IRQ_TYPE_NONE);
	if (err) {
		dev_err(&pdev->dev, "Failed to add irqchip to gpiochip\n");
		goto out_remove;
	}

	for (i = 0; i < nr_irqs; i++)
		gpiochip_lock_as_irq(&combiner->gpio_chip, i);

	gpiochip_set_chained_irqchip(&combiner->gpio_chip, &combiner->irq_chip,
				     combiner->parent_irq, combiner_handle_irq);

	dev_info(&pdev->dev, "Initialized with [p=%d,n=%d,r=%p]\n",
		 combiner->parent_irq, nr_irqs, combiner->regs[0].addr);

	return 0;

out_remove:
	gpiochip_remove(&combiner->gpio_chip);
	return err;
}

static const struct acpi_device_id qcom_irq_combiner_acpi_match[] = {
	{ "QCOM80B1", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, qcom_irq_combiner_acpi_match);

static struct platform_driver qcom_irq_combiner_probe = {
	.driver = {
		.name = "qcom-irq-combiner",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(qcom_irq_combiner_acpi_match),
	},
	.probe = combiner_probe,
};

static int __init register_qcom_irq_combiner(void)
{
	return platform_driver_register(&qcom_irq_combiner_probe);
}
arch_initcall(register_qcom_irq_combiner);

/*
 * TCSR initialization and probing.
 */

static int __init tcsr_probe(struct platform_device *pdev)
{
	struct resource *mr;

	mr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mr == NULL) {
		dev_err(&pdev->dev, "Error getting memory resource.\n");
		return -EINVAL;
	}

	tcsr_base = devm_ioremap_resource(&pdev->dev, mr);
	if (IS_ERR(tcsr_base)) {
		dev_err(&pdev->dev, "Error mapping memory resource\n");
		return PTR_ERR(tcsr_base);
	}

	dev_info(&pdev->dev, "Initialized TCSR combiner @%pa\n", &mr->start);

	return 0;
}

static const struct acpi_device_id qcom_tcsr_acpi_match[] = {
	{ "QCOM80B0", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, qcom_tcsr_acpi_match);

static struct platform_driver qcom_tcsr_probe = {
	.driver = {
		.name = "qcom-tcsr",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(qcom_tcsr_acpi_match),
	},
	.probe = tcsr_probe,
};

static int __init register_qcom_tcsr(void)
{
	return platform_driver_register(&qcom_tcsr_probe);
}
arch_initcall(register_qcom_tcsr);

#endif /* CONFIG_ACPI */
