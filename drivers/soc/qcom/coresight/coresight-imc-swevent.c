/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>

#include "coresight.h"
#include "coresight-priv.h"
#include "coresight-property.h"

#define	NUM_TAG_BYTES	12
#define	TAG_STR_SIZE	50
#define	NUM_ENTITY_BYTES	32
#define	NUM_EVENT_BYTES	256

struct swevent_table {
	phys_addr_t	start;
	phys_addr_t	end;
};

struct imc_shared_memory {
	u64	magic_header;
	u8	version;
	u8	reserved_1;
	u8	reserved_2;
	u8	reserved_3;
	u8	trace_output;
	u8	reserved_4;
	u8	reserved_5;
	u8	reserved_6;
	u8	sw_event_tag[NUM_TAG_BYTES];
	u32	event_table_size;
/*
 * Entity and event control tables are a specific number of bytes in size,
 * but are addressed as u64. u64 are 8 bytes each, so we shift by 3
 * for the correct array size.
 */
	u64	entity_ctrl[NUM_ENTITY_BYTES >> 3];
	u64	event_ctrl[NUM_EVENT_BYTES >> 3];
};

struct swevent_drvdata {
	struct device				*dev;
	struct coresight_device			*csdev;
	struct mutex				mutex;
	int					sw_entity_grp;
	unsigned int				sw_entity_size;
	int					sw_event_grp;
	unsigned int				sw_event_size;
	struct swevent_table			table;
	struct imc_shared_memory __iomem	*mem_table;
};

void swevent_enable(struct swevent_drvdata *drvdata)
{
	mutex_lock(&drvdata->mutex);
	writeb(1, &drvdata->mem_table->trace_output);
	wmb();
	mutex_unlock(&drvdata->mutex);

	dev_dbg(drvdata->dev, "Software Event traces for IMC enabled\n");
}

void swevent_disable(struct swevent_drvdata *drvdata)
{
	mutex_lock(&drvdata->mutex);
	writeb(0, &drvdata->mem_table->trace_output);
	wmb();
	mutex_unlock(&drvdata->mutex);
}

static ssize_t show_enable(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u8 val;

	mutex_lock(&drvdata->mutex);
	val = readb(&drvdata->mem_table->trace_output);
	mutex_unlock(&drvdata->mutex);

	return scnprintf(buf, PAGE_SIZE, "%#x\n", val);
}

static ssize_t store_enable(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u8 val;

	if (kstrtou8(buf, 10, &val) != 0)
		return -EINVAL;
	if (val)
		swevent_enable(drvdata);
	else
		swevent_disable(drvdata);
	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_enable,
		   store_enable);

static ssize_t show_sw_entity_grp(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return scnprintf(buf, PAGE_SIZE, "%d\n", drvdata->sw_entity_grp);
}

static ssize_t store_sw_entity_grp(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	int val;

	if (kstrtoint(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	if ((drvdata->sw_entity_size / 8) <= val) {
		dev_err(dev, "Not a valid software entity group\n");
		mutex_unlock(&drvdata->mutex);
		return -EINVAL;
	}
	drvdata->sw_entity_grp = val;
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_entity_grp, S_IRUGO | S_IWUSR, show_sw_entity_grp,
		   store_sw_entity_grp);

static ssize_t show_sw_entity_mask(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	mutex_lock(&drvdata->mutex);
	val = readq(&drvdata->mem_table->entity_ctrl[drvdata->sw_entity_grp]);
	mutex_unlock(&drvdata->mutex);

	return scnprintf(buf, PAGE_SIZE, "%#llx\n", val);
}

static ssize_t store_sw_entity_mask(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	if (kstrtou64(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	writeq(val, &drvdata->mem_table->entity_ctrl[drvdata->sw_entity_grp]);
	wmb();
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_entity_mask, S_IRUGO | S_IWUSR, show_sw_entity_mask,
		   store_sw_entity_mask);

static ssize_t show_sw_event_grp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return scnprintf(buf, PAGE_SIZE, "%d\n", drvdata->sw_event_grp);
}

static ssize_t store_sw_event_grp(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	int val;

	if (kstrtoint(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	if ((drvdata->sw_event_size / 8) <= val) {
		dev_err(dev, "Not a valid software event group\n");
		mutex_unlock(&drvdata->mutex);
		return -EINVAL;
	}
	drvdata->sw_event_grp = val;
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_event_grp, S_IRUGO | S_IWUSR, show_sw_event_grp,
		   store_sw_event_grp);

static ssize_t show_sw_event_mask(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	mutex_lock(&drvdata->mutex);
	val = readq(&drvdata->mem_table->event_ctrl[drvdata->sw_event_grp]);
	mutex_unlock(&drvdata->mutex);

	return scnprintf(buf, PAGE_SIZE, "%#llx\n", val);
}

static ssize_t store_sw_event_mask(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	if (kstrtou64(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	writeq(val, &drvdata->mem_table->event_ctrl[drvdata->sw_event_grp]);
	wmb();
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_event_mask, S_IRUGO | S_IWUSR, show_sw_event_mask,
		   store_sw_event_mask);

static ssize_t show_sw_event_tag(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	int i;
	u8 tmp;
	char tag[TAG_STR_SIZE + 1] = "";
	char b_str[TAG_STR_SIZE + 1] = "";

	mutex_lock(&drvdata->mutex);
	for (i = 0; i < NUM_TAG_BYTES; ++i) {
		tmp = readb(&drvdata->mem_table->sw_event_tag[i]);
		scnprintf(b_str, TAG_STR_SIZE, "%02x", tmp);
		strlcat(tag, b_str, TAG_STR_SIZE);
	}
	mutex_unlock(&drvdata->mutex);

	return scnprintf(buf, PAGE_SIZE, "0x%s\n", tag);
}

static DEVICE_ATTR(sw_event_tag, S_IRUGO | S_IRUSR, show_sw_event_tag,
		   NULL);

static struct attribute *swevent_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_sw_entity_grp.attr,
	&dev_attr_sw_entity_mask.attr,
	&dev_attr_sw_event_grp.attr,
	&dev_attr_sw_event_mask.attr,
	&dev_attr_sw_event_tag.attr,
	NULL,
};

static struct attribute_group swevent_attr_grp = {
	.attrs = swevent_attrs,
};

static const struct attribute_group *swevent_attr_grps[] = {
	&swevent_attr_grp,
	NULL,
};

static int swevent_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct swevent_drvdata *drvdata;
	struct coresight_desc *desc;
	struct coresight_platform_data *pdata;
	struct resource *res;

	pdata = device_get_coresight_platform_data(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	pdev->dev.platform_data = pdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "cannot access software event resource\n");
		return -ENXIO;
	}
	drvdata->table.start = res->start;
	drvdata->table.end = res->end;

	drvdata->mem_table = devm_ioremap(drvdata->dev,
				drvdata->table.start,
				drvdata->table.end - drvdata->table.start);
	if (!drvdata->mem_table) {
		dev_err(drvdata->dev, "unable to map address 0x%llx\n",
			drvdata->table.start);
		return -ENOMEM;
	}

	mutex_init(&drvdata->mutex);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	desc->type = CORESIGHT_DEV_TYPE_NONE;
	desc->pdata = pdev->dev.platform_data;
	desc->dev = &pdev->dev;
	desc->groups = swevent_attr_grps;
	desc->owner = THIS_MODULE;
	drvdata->csdev = coresight_register(desc);
	if (IS_ERR(drvdata->csdev))
		return PTR_ERR(drvdata->csdev);

	drvdata->sw_entity_size = NUM_ENTITY_BYTES;
	drvdata->sw_event_size = NUM_EVENT_BYTES;

	dev_dbg(dev, "Software IMC Event driver initialized\n");

	return 0;
}

static int swevent_remove(struct platform_device *pdev)
{
	struct swevent_drvdata *drvdata = platform_get_drvdata(pdev);

	coresight_unregister(drvdata->csdev);
	return 0;
}

#if IS_ENABLED(CONFIG_ACPI)
static const struct acpi_device_id swevent_acpi_match[] = {
	{ "QCOM80F3" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, swevent_acpi_match);
#endif

static struct platform_driver swevent_driver = {
	.probe		= swevent_probe,
	.remove		= swevent_remove,
	.driver		= {
		.name	= "coresight-imc-swevent",
		.owner	= THIS_MODULE,
		.acpi_match_table = ACPI_PTR(swevent_acpi_match),
	},
};

module_platform_driver(swevent_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight IMC Software Event driver");
