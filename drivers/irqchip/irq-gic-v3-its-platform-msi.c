/*
 * Copyright (C) 2013-2015 ARM Limited, All Rights Reserved.
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/msi.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/iort.h>

static struct irq_chip its_pmsi_irq_chip = {
	.name			= "ITS-pMSI",
};

static int its_pmsi_prepare(struct irq_domain *domain, struct device *dev,
			    int nvec, msi_alloc_info_t *info)
{
	struct msi_domain_info *msi_info;
	u32 dev_id;
	int ret, index = 0;

	msi_info = msi_get_domain_info(domain->parent);

	/* Suck the DeviceID out of the msi-parent property */
	do {
		struct of_phandle_args args;

		ret = of_parse_phandle_with_args(dev->of_node,
						 "msi-parent", "#msi-cells",
						 index, &args);
		if (args.np == irq_domain_get_of_node(domain)) {
			if (WARN_ON(args.args_count != 1))
				return -EINVAL;
			dev_id = args.args[0];
			break;
		}
	} while (!ret);

	if (ret)
		return ret;

	/* ITS specific DeviceID, as the core ITS ignores dev. */
	info->scratchpad[0].ul = dev_id;

	return msi_info->ops->msi_prepare(domain->parent,
					  dev, nvec, info);
}

static struct msi_domain_ops its_pmsi_ops = {
	.msi_prepare	= its_pmsi_prepare,
};

static struct msi_domain_info its_pmsi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.ops	= &its_pmsi_ops,
	.chip	= &its_pmsi_irq_chip,
};

static struct of_device_id its_device_id[] = {
	{	.compatible	= "arm,gic-v3-its",	},
	{},
};

static int __init of_its_pmsi_init(void)
{
	struct device_node *np;
	struct irq_domain *parent;

	for (np = of_find_matching_node(NULL, its_device_id); np;
	     np = of_find_matching_node(np, its_device_id)) {
		if (!of_property_read_bool(np, "msi-controller"))
			continue;

		parent = irq_find_matching_host(np, DOMAIN_BUS_NEXUS);
		if (!parent || !msi_get_domain_info(parent)) {
			pr_err("%s: unable to locate ITS domain\n",
			       np->full_name);
			continue;
		}

		if (!platform_msi_create_irq_domain(of_node_to_fwnode(np),
						    &its_pmsi_domain_info,
						    parent)) {
			pr_err("%s: unable to create platform domain\n",
			       np->full_name);
			continue;
		}

		pr_info("Platform MSI: %s domain created\n", np->full_name);
	}

	return 0;
}

#ifdef CONFIG_ACPI
static struct irq_domain *its_pmsi_msi_default_domain;

static int its_pmsi_acpi_prepare(struct irq_domain *domain, struct device *dev,
				 int nvec, msi_alloc_info_t *info)
{
	struct msi_domain_info *msi_info;
	u32 dev_id;
	int ret;

	msi_info = msi_get_domain_info(domain->parent);

	/* Suck the DeviceID out of the device-id property */
	ret = device_property_read_u32(dev, "device-id", &dev_id);
	if (ret)
		return ret;

	/* ITS specific DeviceID, as the core ITS ignores dev. */
	info->scratchpad[0].ul = dev_id;

	return msi_info->ops->msi_prepare(domain->parent,
					   dev, nvec, info);
}

static struct msi_domain_ops its_pmsi_acpi_ops = {
	.msi_prepare	= its_pmsi_acpi_prepare,
};

static struct msi_domain_info its_pmsi_acpi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS),
	.ops	= &its_pmsi_acpi_ops,
	.chip	= &its_pmsi_irq_chip,
};

static int attach_notifier(struct notifier_block *nb,
			   unsigned long event, void *data)
{
	struct device *dev = data;
	u32 msi = 0;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (!dev)
		return NOTIFY_BAD;

	/* see what msi-support property reads */
	device_property_read_u32(dev, "msi-support", &msi);
	if (msi) {
		dev_set_msi_domain(dev, its_pmsi_msi_default_domain);
		dev_info(dev, "setting default platform MSI domain\n");
	}
	return NOTIFY_OK;
}

static struct notifier_block platform_nb = {
	.notifier_call  = attach_notifier,
};

static int __init acpi_parse_its_frame(struct acpi_subtable_header *header,
				       const unsigned long end)
{
	struct acpi_madt_generic_translator *its;
	struct irq_domain *parent;
	struct irq_domain *domain;
	struct fwnode_handle *domain_handle;

	its = (struct acpi_madt_generic_translator *)header;
	if (BAD_MADT_ENTRY(its, end))
		return -EINVAL;

	domain_handle = iort_find_domain_token(its->translation_id);
	if (!domain_handle) {
		pr_err("ITS-platform@0x%lx: Unable to locate ITS domain handle\n",
		       (long)its->base_address);
		return 0;
	}

	if (!its->base_address)
		return -EFAULT;

	parent = irq_find_matching_fwnode(domain_handle, DOMAIN_BUS_NEXUS);
	if (!parent || !msi_get_domain_info(parent)) {
		pr_err("Unable to locate ITS domain\n");
		return -ENXIO;
	}

	domain = platform_msi_create_irq_domain(domain_handle,
						&its_pmsi_acpi_domain_info,
						parent);
	if (!domain) {
		pr_err("its-platform @%pa: unable to create platform MSI domain\n",
			&its->base_address);
		return 0;
	}
	its_pmsi_msi_default_domain = domain;
	return 0;
}

static int __init acpi_its_pmsi_init(void)
{
	acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_TRANSLATOR,
			      acpi_parse_its_frame, 0);
	return 0;
}
#else
static int __init acpi_its_pmsi_init(void) { return 0; }
#endif

static int __init its_pmsi_arch_init(void)
{
	if (!acpi_disabled)
		bus_register_notifier(&platform_bus_type, &platform_nb);

	return 0;
}

static int __init its_pmsi_init(void)
{
	if (acpi_disabled)
		return of_its_pmsi_init();

	return acpi_its_pmsi_init();
}
early_initcall(its_pmsi_init);
arch_initcall(its_pmsi_arch_init);
