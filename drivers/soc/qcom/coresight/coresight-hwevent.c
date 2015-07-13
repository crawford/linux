/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/acpi.h>

#include "coresight.h"
#include "coresight-priv.h"
#include "coresight-property.h"

struct hwevent_mux {
	phys_addr_t				start;
	phys_addr_t				end;
};

struct hwevent_drvdata {
	struct device				*dev;
	struct coresight_device			*csdev;
	struct clk				*clk;
	struct mutex				mutex;
	int					nr_hmux;
	struct hwevent_mux			*hmux;
	atomic_t				enable;
};

static int hwevent_enable(struct hwevent_drvdata *drvdata)
{
	atomic_set(&drvdata->enable, true);
	dev_dbg(drvdata->dev, "Qualcomm Hardware Event driver enabled\n");
	return 0;
}

static void hwevent_disable(struct hwevent_drvdata *drvdata)
{
	atomic_set(&drvdata->enable, false);
	dev_dbg(drvdata->dev, "Qualcomm Hardware Event driver disabled\n");
}

static ssize_t hwevent_show_enable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct hwevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = atomic_read(&drvdata->enable);

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t hwevent_store_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct hwevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;
	int ret = 0;

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	if (val)
		ret = hwevent_enable(drvdata);
	else
		hwevent_disable(drvdata);

	if (ret)
		return ret;
	return size;
}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, hwevent_show_enable,
		   hwevent_store_enable);

static ssize_t hwevent_store_setreg(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct hwevent_drvdata *drvdata = dev_get_drvdata(dev->parent);
	void *hwereg;
	unsigned long long addr;
	unsigned long val;
	int ret, i;

	if (sscanf(buf, "%llx %lx", &addr, &val) != 2)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (!atomic_read(&drvdata->enable)) {
		dev_err(dev, "Qualcomm Hardware Event driver not enabled\n");
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < drvdata->nr_hmux; i++) {
		if ((addr >= drvdata->hmux[i].start) &&
		    (addr < drvdata->hmux[i].end)) {
			hwereg = devm_ioremap(dev,
					      drvdata->hmux[i].start,
					      drvdata->hmux[i].end -
					      drvdata->hmux[i].start);
			if (!hwereg) {
				dev_err(dev, "unable to map address 0x%llx\n",
					addr);
				ret = -ENOMEM;
				goto err;
			}
			writel_relaxed(val, hwereg + addr -
				       drvdata->hmux[i].start);
			/* Ensure writes to hwevent control registers
			   are completed before unmapping the address
			*/
			mb();
			devm_iounmap(dev, hwereg);
			break;
		}
	}

	if (i == drvdata->nr_hmux) {
		ret = coresight_csr_hwctrl_set(addr, val);
		if (ret) {
			dev_err(dev, "invalid mux control register address\n");
			ret = -EINVAL;
			goto err;
		}
	}

	mutex_unlock(&drvdata->mutex);
	return size;
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR(setreg, S_IWUSR, NULL, hwevent_store_setreg);

static struct attribute *hwevent_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_setreg.attr,
	NULL,
};

static struct attribute_group hwevent_attr_grp = {
	.attrs = hwevent_attrs,
};

static const struct attribute_group *hwevent_attr_grps[] = {
	&hwevent_attr_grp,
	NULL,
};

static int hwevent_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hwevent_drvdata *drvdata;
	struct coresight_desc *desc;
	struct coresight_platform_data *pdata;
	struct resource *res;
	int res_num;

	if (coresight_fuse_access_disabled())
		return -EPERM;

	pdata = device_get_coresight_platform_data(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	pdev->dev.platform_data = pdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->hmux = devm_kcalloc(dev, pdev->num_resources,
				     sizeof(*drvdata->hmux),
				     GFP_KERNEL);
	if (!drvdata->hmux)
		return -ENOMEM;

	for (res_num = 0; res_num < pdev->num_resources; res_num++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM,
					    res_num);
		if (!res) {
			dev_dbg(dev, "failed to get memory resource\n");
			continue;
		}
		drvdata->hmux[res_num].start = res->start;
		drvdata->hmux[res_num].end = res->end;
		drvdata->nr_hmux++;
	}

	mutex_init(&drvdata->mutex);

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->type = CORESIGHT_DEV_TYPE_NONE;
	desc->pdata = pdev->dev.platform_data;
	desc->dev = &pdev->dev;
	desc->groups = hwevent_attr_grps;
	desc->owner = THIS_MODULE;
	drvdata->csdev = coresight_register(desc);
	if (IS_ERR(drvdata->csdev))
		return PTR_ERR(drvdata->csdev);

	dev_dbg(dev, "Qualcomm Hardware Event driver initialized\n");
	return 0;
}

static int hwevent_remove(struct platform_device *pdev)
{
	struct hwevent_drvdata *drvdata = platform_get_drvdata(pdev);

	coresight_unregister(drvdata->csdev);
	return 0;
}

static struct of_device_id hwevent_match[] = {
	{.compatible = "qcom,coresight-hwevent"},
	{}
};

#if IS_ENABLED(CONFIG_ACPI)
static const struct acpi_device_id hwevent_acpi_match[] = {
	{"QCOM80F2"},
	{},
};
MODULE_DEVICE_TABLE(acpi, hwevent_acpi_match);
#endif

static struct platform_driver hwevent_driver = {
	.probe		= hwevent_probe,
	.remove		= hwevent_remove,
	.driver		= {
		.name	= "coresight-hwevent",
		.owner	= THIS_MODULE,
		.of_match_table	= hwevent_match,
		.acpi_match_table = ACPI_PTR(hwevent_acpi_match),
	},
};

static int __init hwevent_init(void)
{
	return platform_driver_register(&hwevent_driver);
}
module_init(hwevent_init);

static void __exit hwevent_exit(void)
{
	platform_driver_unregister(&hwevent_driver);
}
module_exit(hwevent_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight Hardware Event driver");
