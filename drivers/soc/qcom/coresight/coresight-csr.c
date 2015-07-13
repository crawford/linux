/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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
#include <linux/of.h>
#include <linux/acpi.h>

#include "coresight-property.h"
#include "coresight.h"
#include "coresight-priv.h"

#define csr_writel(drvdata, val, off)	__raw_writel((val), drvdata->base + off)
#define csr_readl(drvdata, off)		__raw_readl(drvdata->base + off)

#define CSR_LOCK(drvdata)						\
do {									\
	mb();								\
	csr_writel(drvdata, 0x0, CORESIGHT_LAR);			\
} while (0)
#define CSR_UNLOCK(drvdata)						\
do {									\
	csr_writel(drvdata, CORESIGHT_UNLOCK, CORESIGHT_LAR);		\
	mb();								\
} while (0)

#define CSR_SWDBGPWRCTRL	(0x000)
#define CSR_SWDBGPWRACK		(0x004)
#define CSR_SWSPADREG0		(0x008)
#define CSR_SWSPADREG1		(0x00C)
#define CSR_STMTRANSCTRL	(0x010)
#define CSR_STMAWIDCTRL		(0x014)
#define CSR_STMCHNOFST0		(0x018)
#define CSR_STMCHNOFST1		(0x01C)
#define CSR_STMEXTHWCTRL0	(0x020)
#define CSR_STMEXTHWCTRL1	(0x024)
#define CSR_STMEXTHWCTRL2	(0x028)
#define CSR_STMEXTHWCTRL3	(0x02C)
#define CSR_USBBAMCTRL		(0x030)
#define CSR_USBFLSHCTRL		(0x034)
#define CSR_TIMESTAMPCTRL	(0x038)
#define CSR_AOTIMEVAL0		(0x03C)
#define CSR_AOTIMEVAL1		(0x040)
#define CSR_QDSSTIMEVAL0	(0x044)
#define CSR_QDSSTIMEVAL1	(0x048)
#define CSR_QDSSTIMELOAD0	(0x04C)
#define CSR_QDSSTIMELOAD1	(0x050)
#define CSR_DAPMSAVAL		(0x054)
#define CSR_QDSSCLKVOTE		(0x058)
#define CSR_QDSSCLKIPI		(0x05C)
#define CSR_QDSSPWRREQIGNORE	(0x060)
#define CSR_QDSSSPARE		(0x064)
#define CSR_IPCAT		(0x068)
#define CSR_BYTECNTVAL		(0x06C)

#define BLKSIZE_256		0
#define BLKSIZE_512		1
#define BLKSIZE_1024		2
#define BLKSIZE_2048		3

struct csr_drvdata {
	void __iomem		*base;
	phys_addr_t		pbase;
	struct device		*dev;
	struct coresight_device	*csdev;
	uint32_t		blksize;
};

static struct csr_drvdata *csrdrvdata;

void msm_qdss_csr_enable_bam_to_usb(void)
{
	struct csr_drvdata *drvdata = csrdrvdata;
	uint32_t usbbamctrl, usbflshctrl;

	CSR_UNLOCK(drvdata);

	usbbamctrl = csr_readl(drvdata, CSR_USBBAMCTRL);
	usbbamctrl = (usbbamctrl & ~0x3) | drvdata->blksize;
	csr_writel(drvdata, usbbamctrl, CSR_USBBAMCTRL);

	usbflshctrl = csr_readl(drvdata, CSR_USBFLSHCTRL);
	usbflshctrl = (usbflshctrl & ~0x3FFFC) | (0xFFFF << 2);
	csr_writel(drvdata, usbflshctrl, CSR_USBFLSHCTRL);
	usbflshctrl |= 0x2;
	csr_writel(drvdata, usbflshctrl, CSR_USBFLSHCTRL);

	usbbamctrl |= 0x4;
	csr_writel(drvdata, usbbamctrl, CSR_USBBAMCTRL);

	CSR_LOCK(drvdata);
}
EXPORT_SYMBOL(msm_qdss_csr_enable_bam_to_usb);

void msm_qdss_csr_disable_bam_to_usb(void)
{
	struct csr_drvdata *drvdata = csrdrvdata;
	uint32_t usbbamctrl;

	CSR_UNLOCK(drvdata);

	usbbamctrl = csr_readl(drvdata, CSR_USBBAMCTRL);
	usbbamctrl &= (~0x4);
	csr_writel(drvdata, usbbamctrl, CSR_USBBAMCTRL);

	CSR_LOCK(drvdata);
}
EXPORT_SYMBOL(msm_qdss_csr_disable_bam_to_usb);

void msm_qdss_csr_disable_flush(void)
{
	struct csr_drvdata *drvdata = csrdrvdata;
	uint32_t usbflshctrl;

	CSR_UNLOCK(drvdata);

	usbflshctrl = csr_readl(drvdata, CSR_USBFLSHCTRL);
	usbflshctrl &= ~0x2;
	csr_writel(drvdata, usbflshctrl, CSR_USBFLSHCTRL);

	CSR_LOCK(drvdata);
}
EXPORT_SYMBOL(msm_qdss_csr_disable_flush);

int coresight_csr_hwctrl_set(uint64_t addr, uint32_t val)
{
	struct csr_drvdata *drvdata = csrdrvdata;
	int ret = 0;

	CSR_UNLOCK(drvdata);

	if (addr == (drvdata->pbase + CSR_STMEXTHWCTRL0))
		csr_writel(drvdata, val, CSR_STMEXTHWCTRL0);
	else if (addr == (drvdata->pbase + CSR_STMEXTHWCTRL1))
		csr_writel(drvdata, val, CSR_STMEXTHWCTRL1);
	else if (addr == (drvdata->pbase + CSR_STMEXTHWCTRL2))
		csr_writel(drvdata, val, CSR_STMEXTHWCTRL2);
	else if (addr == (drvdata->pbase + CSR_STMEXTHWCTRL3))
		csr_writel(drvdata, val, CSR_STMEXTHWCTRL3);
	else
		ret = -EINVAL;

	CSR_LOCK(drvdata);

	return ret;
}
EXPORT_SYMBOL(coresight_csr_hwctrl_set);

void coresight_csr_set_byte_cntr(uint32_t count)
{
	struct csr_drvdata *drvdata = csrdrvdata;

	CSR_UNLOCK(drvdata);

	csr_writel(drvdata, count, CSR_BYTECNTVAL);
	mb();

	CSR_LOCK(drvdata);
}
EXPORT_SYMBOL(coresight_csr_set_byte_cntr);

static ssize_t csr_show_timestamp_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct csr_drvdata *drvdata = dev_get_drvdata(dev->parent);
	uint32_t tsctrl;

	CSR_UNLOCK(drvdata);
	tsctrl = csr_readl(drvdata, CSR_TIMESTAMPCTRL) & 0x2;
	CSR_LOCK(drvdata);

	return scnprintf(buf, PAGE_SIZE, "%#x\n", tsctrl);
}

static ssize_t csr_store_timestamp_enable(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct csr_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;
	uint32_t tsctrl;
	int retval = kstrtoul(buf, 0, &val);

	if (retval)
		return retval;

	CSR_UNLOCK(drvdata);
	tsctrl = csr_readl(drvdata, CSR_TIMESTAMPCTRL);
	if (val)
		tsctrl |= 0x2;
	else
		tsctrl &= ~0x2;
	csr_writel(drvdata, tsctrl, CSR_TIMESTAMPCTRL);
	CSR_LOCK(drvdata);

	return size;
}

static DEVICE_ATTR(timestamp_enable, S_IRUGO | S_IWUSR,
		   csr_show_timestamp_enable, csr_store_timestamp_enable);

static struct attribute *csr_attrs[] = {
	&dev_attr_timestamp_enable.attr,
	NULL,
};

static struct attribute_group csr_attr_grp = {
	.attrs = csr_attrs,
};

static const struct attribute_group *csr_attr_grps[] = {
	&csr_attr_grp,
	NULL,
};

static int csr_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct coresight_platform_data *pdata;
	struct csr_drvdata *drvdata;
	struct resource *res;
	struct coresight_desc *desc;

	if (coresight_fuse_access_disabled())
		return -EPERM;

	pdata = device_get_coresight_platform_data(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	pdev->dev.platform_data = pdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	/* Store the driver data pointer for use in exported functions */
	csrdrvdata = drvdata;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	drvdata->pbase = res->start;

	drvdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->base)
		return -ENOMEM;

	ret = device_property_read_u32(&pdev->dev, "qcom,blk-size",
				   &drvdata->blksize);
	if (ret)
		drvdata->blksize = BLKSIZE_256;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	desc->type = CORESIGHT_DEV_TYPE_NONE;
	desc->pdata = pdev->dev.platform_data;
	desc->dev = &pdev->dev;
	desc->groups = csr_attr_grps;
	desc->owner = THIS_MODULE;
	drvdata->csdev = coresight_register(desc);
	if (IS_ERR(drvdata->csdev))
		return PTR_ERR(drvdata->csdev);

	dev_info(dev, "CSR initialized\n");
	return 0;
}

static int csr_remove(struct platform_device *pdev)
{
	struct csr_drvdata *drvdata = platform_get_drvdata(pdev);

	coresight_unregister(drvdata->csdev);
	return 0;
}

static struct of_device_id csr_match[] = {
	{.compatible = "qcom,coresight-csr"},
	{}
};

#if IS_ENABLED(CONFIG_ACPI)
static const struct acpi_device_id csr_acpi_match[] = {
	{ "ARMHFFF4"  },
	{ },
};
MODULE_DEVICE_TABLE(acpi, csr_acpi_match);
#endif

static struct platform_driver csr_driver = {
	.probe          = csr_probe,
	.remove         = csr_remove,
	.driver         = {
		.name   = "coresight-csr",
		.owner	= THIS_MODULE,
		.of_match_table = csr_match,
		.acpi_match_table = ACPI_PTR(csr_acpi_match),
	},
};

static int __init csr_init(void)
{
	return platform_driver_register(&csr_driver);
}
module_init(csr_init);

static void __exit csr_exit(void)
{
	platform_driver_unregister(&csr_driver);
}
module_exit(csr_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight CSR driver");
