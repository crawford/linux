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
#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/psci.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>

#include "coresight.h"
#include "coresight-priv.h"
#include "coresight-property.h"

#define	TAG_STR_SIZE	50

#define	TZ_SVC_QDSS			0xc2001600
#define	QDSS_QUERY_SWTRACE_STATE	0x1
#define	QDSS_FILTER_SWTRACE		0x2
#define	QDSS_QUERY_SWENTITY_STATE	0x3
#define	QDSS_FILTER_SWENTITY		0x4
#define	QDSS_QUERY_SWEVENT_TAG		0x5
#define	QDSS_QUERY_SWEVENT_STATE	0x6
#define	QDSS_FILTER_SWEVENT		0x7

typedef u64 (tz_smc_fn)(u64, u64, u64, u64);

asmlinkage tz_smc_fn __invoke_coresight_fn_smc;
static tz_smc_fn *invoke_tz_smc;


struct swevent_drvdata {
	struct device				*dev;
	struct coresight_device			*csdev;
	struct mutex				mutex;
	int					sw_entity_grp;
	int					sw_event_grp;
};

static void tz_swevent_enable(struct swevent_drvdata *drvdata)
{
	mutex_lock(&drvdata->mutex);
	invoke_tz_smc(TZ_SVC_QDSS, QDSS_FILTER_SWTRACE, 1, 0);
	mutex_unlock(&drvdata->mutex);

	dev_dbg(drvdata->dev, "Software Event traces for TZ enabled\n");
}

static void tz_swevent_disable(struct swevent_drvdata *drvdata)
{
	mutex_lock(&drvdata->mutex);
	invoke_tz_smc(TZ_SVC_QDSS, QDSS_FILTER_SWTRACE, 0, 0);
	mutex_unlock(&drvdata->mutex);
}

static ssize_t show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 ret, enable;
	register u64 result_1 __asm ("x1");

	mutex_lock(&drvdata->mutex);
	ret = invoke_tz_smc(TZ_SVC_QDSS, QDSS_QUERY_SWTRACE_STATE, 0, 0);
	enable = result_1;
	mutex_unlock(&drvdata->mutex);

	if (ret)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%#llx\n", enable);
}

static ssize_t store_enable(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u8 val;

	if (kstrtou8(buf, 10, &val))
		return -EINVAL;
	if (val)
		tz_swevent_enable(drvdata);
	else
		tz_swevent_disable(drvdata);
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

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
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
	u64 ret, mask;
	register u64 result_1 __asm ("x1");

	mutex_lock(&drvdata->mutex);
	ret = invoke_tz_smc(TZ_SVC_QDSS, QDSS_QUERY_SWENTITY_STATE,
		       drvdata->sw_entity_grp, 0);
	mask = result_1;
	mutex_unlock(&drvdata->mutex);

	if (ret)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%#llx\n", mask);
}

static ssize_t store_sw_entity_mask(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	if (kstrtou64(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	invoke_tz_smc(TZ_SVC_QDSS, QDSS_FILTER_SWENTITY,
		drvdata->sw_entity_grp, val);
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

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	drvdata->sw_event_grp = val;
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_event_grp, S_IRUGO | S_IWUSR, show_sw_event_grp,
		   store_sw_event_grp);

static ssize_t show_tz_sw_event_mask(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 ret, mask;
	register u64 result_1 __asm ("x1");

	mutex_lock(&drvdata->mutex);
	ret = invoke_tz_smc(TZ_SVC_QDSS, QDSS_QUERY_SWEVENT_STATE,
		       drvdata->sw_event_grp, 0);
	mask = result_1;
	mutex_unlock(&drvdata->mutex);

	if (ret)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%#llx\n", mask);
}

static ssize_t store_tz_sw_event_mask(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	u64 val;

	if (kstrtou64(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	invoke_tz_smc(TZ_SVC_QDSS, QDSS_FILTER_SWEVENT,
			drvdata->sw_event_grp, val);
	mutex_unlock(&drvdata->mutex);

	return size;
}

static DEVICE_ATTR(sw_event_mask, S_IRUGO | S_IWUSR, show_tz_sw_event_mask,
		   store_tz_sw_event_mask);

static ssize_t show_sw_event_tag(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct swevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	char tag[TAG_STR_SIZE + 1] = "";
	char b_str[TAG_STR_SIZE + 1] = "";
	u64 ret, high_bits, low_bits;
	register u64 result_1 __asm ("x1");
	register u64 result_2 __asm ("x2");

	mutex_lock(&drvdata->mutex);
	ret = invoke_tz_smc(TZ_SVC_QDSS, QDSS_QUERY_SWEVENT_TAG, 0, 0);
	high_bits = result_2;
	low_bits = result_1;
	mutex_unlock(&drvdata->mutex);

	if (ret)
		return -EINVAL;

	scnprintf(tag, TAG_STR_SIZE, "%08llx", high_bits);
	scnprintf(b_str, TAG_STR_SIZE, "%016llx", low_bits);
	strlcat(tag, b_str, TAG_STR_SIZE);

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

	pdata = device_get_coresight_platform_data(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	pdev->dev.platform_data = pdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	mutex_init(&drvdata->mutex);

	invoke_tz_smc = __invoke_coresight_fn_smc;

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

	dev_dbg(dev, "Software TZ Event driver initialized\n");

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
	{ "QCOM80F4" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, swevent_acpi_match);
#endif

static struct platform_driver swevent_driver = {
	.probe		= swevent_probe,
	.remove		= swevent_remove,
	.driver		= {
		.name	= "coresight-tz-swevent",
		.owner	= THIS_MODULE,
		.acpi_match_table = ACPI_PTR(swevent_acpi_match),
	},
};

module_platform_driver(swevent_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight TZ Software Event driver");
