/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include "of_coresight.h"
#include "coresight-property.h"
#include "coresight.h"
#include "coresight-cti.h"
#include "coresight-priv.h"

#define tmc_writel(drvdata, val, off)	__raw_writel((val), drvdata->base + off)
#define tmc_readl(drvdata, off)		__raw_readl(drvdata->base + off)

#define TMC_LOCK(drvdata)						\
do {									\
	mb();								\
	tmc_writel(drvdata, 0x0, CORESIGHT_LAR);			\
} while (0)
#define TMC_UNLOCK(drvdata)						\
do {									\
	tmc_writel(drvdata, CORESIGHT_UNLOCK, CORESIGHT_LAR);		\
	mb();								\
} while (0)

#define TMC_RSZ				(0x004)
#define TMC_STS				(0x00C)
#define TMC_RRD				(0x010)
#define TMC_RRP				(0x014)
#define TMC_RWP				(0x018)
#define TMC_TRG				(0x01C)
#define TMC_CTL				(0x020)
#define TMC_RWD				(0x024)
#define TMC_MODE			(0x028)
#define TMC_LBUFLEVEL			(0x02C)
#define TMC_CBUFLEVEL			(0x030)
#define TMC_BUFWM			(0x034)
#define TMC_RRPHI			(0x038)
#define TMC_RWPHI			(0x03C)
#define TMC_AXICTL			(0x110)
#define TMC_DBALO			(0x118)
#define TMC_DBAHI			(0x11C)
#define TMC_FFSR			(0x300)
#define TMC_FFCR			(0x304)
#define TMC_PSCR			(0x308)
#define TMC_ITMISCOP0			(0xEE0)
#define TMC_ITTRFLIN			(0xEE8)
#define TMC_ITATBDATA0			(0xEEC)
#define TMC_ITATBCTR2			(0xEF0)
#define TMC_ITATBCTR1			(0xEF4)
#define TMC_ITATBCTR0			(0xEF8)

#define BYTES_PER_WORD			4
#define TMC_ETR_BAM_PIPE_INDEX		0
#define TMC_ETR_BAM_NR_PIPES		2

#define TMC_ETFETB_DUMP_MAGIC_OFF	(0)
#define TMC_ETFETB_DUMP_MAGIC		(0x5D1DB1BF)
#define TMC_ETFETB_DUMP_MAGIC_V2	(0x42445953)
#define TMC_ETFETB_DUMP_VER_OFF		(4)
#define TMC_ETFETB_DUMP_VER		(1)
#define TMC_REG_DUMP_MAGIC_OFF		(0)
#define TMC_REG_DUMP_MAGIC		(0x5D1DB1BF)
#define TMC_REG_DUMP_MAGIC_V2		(0x42445953)
#define TMC_REG_DUMP_VER_OFF		(4)
#define TMC_REG_DUMP_VER		(1)

#define TMC_ETR_SG_ENT_TO_BLK(phys_pte)	(((phys_addr_t)phys_pte >> 4)	\
					 << PAGE_SHIFT);
#define TMC_ETR_SG_ENT(phys_pte)	(((phys_pte >> PAGE_SHIFT) << 4) | 0x2);
#define TMC_ETR_SG_NXT_TBL(phys_pte)	(((phys_pte >> PAGE_SHIFT) << 4) | 0x3);
#define TMC_ETR_SG_LST_ENT(phys_pte)	(((phys_pte >> PAGE_SHIFT) << 4) | 0x1);

enum tmc_config_type {
	TMC_CONFIG_TYPE_ETB,
	TMC_CONFIG_TYPE_ETR,
	TMC_CONFIG_TYPE_ETF,
};

enum tmc_mode {
	TMC_MODE_CIRCULAR_BUFFER,
	TMC_MODE_SOFTWARE_FIFO,
	TMC_MODE_HARDWARE_FIFO,
};

enum tmc_etr_out_mode {
	TMC_ETR_OUT_MODE_NONE,
	TMC_ETR_OUT_MODE_MEM,
	TMC_ETR_OUT_MODE_USB,
};

static const char * const str_tmc_etr_out_mode[] = {
	[TMC_ETR_OUT_MODE_NONE]	= "none",
	[TMC_ETR_OUT_MODE_MEM]		= "mem",
	[TMC_ETR_OUT_MODE_USB]		= "usb",
};

enum tmc_etr_mem_type {
	TMC_ETR_MEM_TYPE_CONTIG,
	TMC_ETR_MEM_TYPE_SG,
};

static const char * const str_tmc_etr_mem_type[] = {
	[TMC_ETR_MEM_TYPE_CONTIG]	= "contig",
	[TMC_ETR_MEM_TYPE_SG]		= "sg",
};

enum tmc_mem_intf_width {
	TMC_MEM_INTF_WIDTH_32BITS	= 0x2,
	TMC_MEM_INTF_WIDTH_64BITS	= 0x3,
	TMC_MEM_INTF_WIDTH_128BITS	= 0x4,
	TMC_MEM_INTF_WIDTH_256BITS	= 0x5,
};

struct tmc_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct coresight_device	*csdev;
	struct miscdevice	miscdev;
	struct cdev		byte_cntr_dev;
	struct class		*byte_cntr_class;
	struct clk		*clk;
	spinlock_t		spinlock;
	struct coresight_cti	*cti_flush;
	struct coresight_cti	*cti_reset;
	struct mutex		read_lock;
	int			read_count;
	bool			reading;
	bool			aborting;
	char			*buf;
	dma_addr_t		paddr;
	void __iomem		*vaddr;
	uint32_t		size;
	struct mutex		usb_lock;
	enum tmc_etr_out_mode	out_mode;
	bool			enable;
	enum tmc_config_type	config_type;
	uint32_t		trigger_cntr;
	int			byte_cntr_irq;
	atomic_t		byte_cntr_irq_cnt;
	uint32_t		byte_cntr_value;
	struct mutex		byte_cntr_read_lock;
	struct mutex		byte_cntr_lock;
	uint32_t		byte_cntr_block_size;
	bool			byte_cntr_overflow;
	bool			byte_cntr_present;
	bool			byte_cntr_enable;
	uint32_t		byte_cntr_overflow_cnt;
	bool			byte_cntr_read_active;
	wait_queue_head_t	wq;
	char			*byte_cntr_node;
	uint32_t		mem_size;
	bool			sticky_enable;
	enum tmc_etr_mem_type	mem_type;
	enum tmc_etr_mem_type	memtype;
};

static void tmc_wait_for_ready(struct tmc_drvdata *drvdata)
{
	int count;

	/* Ensure formatter, unformatter and hardware fifo are empty */
	for (count = TIMEOUT_US; BVAL(tmc_readl(drvdata, TMC_STS), 2) != 1
				&& count > 0; count--)
		udelay(1);
	WARN(count == 0, "timeout while waiting for TMC ready, TMC_STS: %#x\n",
	     tmc_readl(drvdata, TMC_STS));
}

static void tmc_flush_and_stop(struct tmc_drvdata *drvdata)
{
	int count;
	uint32_t ffcr;

	ffcr = tmc_readl(drvdata, TMC_FFCR);
	ffcr |= BIT(12);
	tmc_writel(drvdata, ffcr, TMC_FFCR);
	ffcr |= BIT(6);
	tmc_writel(drvdata, ffcr, TMC_FFCR);
	/* Ensure flush completes */
	for (count = TIMEOUT_US; BVAL(tmc_readl(drvdata, TMC_FFCR), 6) != 0
				&& count > 0; count--)
		udelay(1);
	WARN(count == 0, "timeout while flushing TMC, TMC_FFCR: %#x\n",
	     tmc_readl(drvdata, TMC_FFCR));

	tmc_wait_for_ready(drvdata);
}

static void __tmc_enable(struct tmc_drvdata *drvdata)
{
	tmc_writel(drvdata, 0x1, TMC_CTL);
}

static void __tmc_disable(struct tmc_drvdata *drvdata)
{
	tmc_writel(drvdata, 0x0, TMC_CTL);
}

static uint32_t tmc_etr_get_write_ptr(struct tmc_drvdata *drvdata)
{
	uint32_t rwp = 0;

	TMC_UNLOCK(drvdata);

	rwp = tmc_readl(drvdata, TMC_RWP);

	TMC_LOCK(drvdata);

	return rwp;
}

static void tmc_etr_byte_cntr_start(struct tmc_drvdata *drvdata)
{
	if (!drvdata->byte_cntr_present)
		return;

	mutex_lock(&drvdata->byte_cntr_lock);
	atomic_set(&drvdata->byte_cntr_irq_cnt, 0);
	drvdata->byte_cntr_overflow = false;
	drvdata->byte_cntr_read_active = false;
	drvdata->byte_cntr_enable = true;
	if (drvdata->byte_cntr_value != 0)
		drvdata->byte_cntr_overflow_cnt = drvdata->size /
						 (drvdata->byte_cntr_value * 8);
	else
		drvdata->byte_cntr_overflow_cnt = 0;
	coresight_csr_set_byte_cntr(drvdata->byte_cntr_value);
	mutex_unlock(&drvdata->byte_cntr_lock);
}

static void tmc_etr_byte_cntr_stop(struct tmc_drvdata *drvdata)
{
	if (!drvdata->byte_cntr_present)
		return;

	mutex_lock(&drvdata->byte_cntr_lock);
	coresight_csr_set_byte_cntr(0);
	drvdata->byte_cntr_value = 0;
	drvdata->byte_cntr_enable = false;
	mutex_unlock(&drvdata->byte_cntr_lock);

	wake_up(&drvdata->wq);
}

static int tmc_etr_alloc_mem(struct tmc_drvdata *drvdata)
{
	int ret;

	if (!drvdata->vaddr) {
		if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG) {
			drvdata->vaddr = dma_zalloc_coherent(drvdata->dev,
							     drvdata->size,
							     &drvdata->paddr,
							     GFP_KERNEL);
			if (!drvdata->vaddr) {
				ret = -ENOMEM;
				goto err;
			}
		}
	}
	/*
	 * Need to reinitialize buf for each tmc enable session since it is
	 * getting modified during tmc etr dump.
	 */
	drvdata->buf = drvdata->vaddr;
	return 0;
err:
	dev_err(drvdata->dev, "etr ddr memory allocation failed\n");
	return ret;
}

static void tmc_etr_free_mem(struct tmc_drvdata *drvdata)
{
	if (drvdata->vaddr) {
		if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
			dma_free_coherent(drvdata->dev, drvdata->size,
					  drvdata->vaddr, drvdata->paddr);
	       drvdata->vaddr = 0;
	       drvdata->paddr = 0;
	}
}

static void tmc_etr_mem_reset(struct tmc_drvdata *drvdata)
{
	if (drvdata->vaddr) {
		if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
			memset(drvdata->vaddr, 0, drvdata->size);
	}
}

static void __tmc_etb_enable(struct tmc_drvdata *drvdata)
{
	/* Zero out the memory to help with debug */
	memset(drvdata->buf, 0, drvdata->size);

	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, TMC_MODE_CIRCULAR_BUFFER, TMC_MODE);
	tmc_writel(drvdata, 0x1133, TMC_FFCR);
	tmc_writel(drvdata, drvdata->trigger_cntr, TMC_TRG);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etr_enable_to_mem(struct tmc_drvdata *drvdata)
{
	uint32_t axictl;

	tmc_etr_mem_reset(drvdata);

	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, drvdata->size / BYTES_PER_WORD, TMC_RSZ);
	tmc_writel(drvdata, TMC_MODE_CIRCULAR_BUFFER, TMC_MODE);

	axictl = tmc_readl(drvdata, TMC_AXICTL);
	axictl |= (0xF << 8);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
		axictl &= ~(0x1 << 7);
	else
		axictl |= (0x1 << 7);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	axictl = (axictl & ~0x3) | 0x2;
	tmc_writel(drvdata, axictl, TMC_AXICTL);

	tmc_writel(drvdata, (uint32_t)drvdata->paddr, TMC_DBALO);
	tmc_writel(drvdata, (((uint64_t)drvdata->paddr) >> 32) & 0xFF,
		   TMC_DBAHI);
	tmc_writel(drvdata, 0x1133, TMC_FFCR);
	tmc_writel(drvdata, drvdata->trigger_cntr, TMC_TRG);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etf_enable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, TMC_MODE_HARDWARE_FIFO, TMC_MODE);
	tmc_writel(drvdata, 0x3, TMC_FFCR);
	tmc_writel(drvdata, 0x0, TMC_BUFWM);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static int tmc_enable(struct tmc_drvdata *drvdata, enum tmc_mode mode)
{
	int ret;
	unsigned long flags;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		return ret;

	mutex_lock(&drvdata->usb_lock);
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		coresight_cti_map_trigout(drvdata->cti_flush, 1, 0);
		coresight_cti_map_trigin(drvdata->cti_reset, 2, 0);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {

			/*
			 * ETR DDR memory is not allocated until user enables
			 * tmc at least once. If user specifies different ETR
			 * DDR size than the default size or switches between
			 * contiguous or scatter-gather memory type after
			 * enabling tmc; the new selection will be honored from
			 * next tmc enable session.
			 */
			if (drvdata->size != drvdata->mem_size ||
			    drvdata->memtype != drvdata->mem_type) {
				tmc_etr_free_mem(drvdata);
				drvdata->size = drvdata->mem_size;
				drvdata->memtype = drvdata->mem_type;
			}
			ret = tmc_etr_alloc_mem(drvdata);
			if (ret)
				goto err0;

			tmc_etr_byte_cntr_start(drvdata);
			coresight_cti_map_trigout(drvdata->cti_flush,
						  3, 0);
			coresight_cti_map_trigin(drvdata->cti_reset,
						 2, 0);
		}
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			coresight_cti_map_trigout(drvdata->cti_flush, 1, 0);
			coresight_cti_map_trigin(drvdata->cti_reset, 2, 0);
		}
	}

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading) {
		ret = -EBUSY;
		goto err1;
	}

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_enable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_enable_to_mem(drvdata);
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_enable(drvdata);
		else
			__tmc_etf_enable(drvdata);
	}
	drvdata->enable = true;

	/*
	 * sticky_enable prevents users from reading tmc dev node before
	 * enabling tmc at least once.
	 */
	drvdata->sticky_enable = true;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	mutex_unlock(&drvdata->usb_lock);

	dev_info(drvdata->dev, "TMC enabled\n");
	return 0;
err1:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
err0:
	mutex_unlock(&drvdata->usb_lock);
	clk_disable_unprepare(drvdata->clk);
	return ret;
}

static int tmc_enable_sink(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	return tmc_enable(drvdata, TMC_MODE_CIRCULAR_BUFFER);
}

static int tmc_enable_link(struct coresight_device *csdev, int inport,
			   int outport)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	return tmc_enable(drvdata, TMC_MODE_HARDWARE_FIFO);
}

static void __tmc_etb_dump(struct tmc_drvdata *drvdata)
{
	enum tmc_mem_intf_width memwidth;
	uint8_t memwords;
	char *hdr;
	char *bufp;
	uint32_t read_data;

	hdr = drvdata->buf - PAGE_SIZE;

	memwidth = BMVAL(tmc_readl(drvdata, CORESIGHT_DEVID), 8, 10);
	if (memwidth == TMC_MEM_INTF_WIDTH_32BITS)
		memwords = 1;
	else if (memwidth == TMC_MEM_INTF_WIDTH_64BITS)
		memwords = 2;
	else if (memwidth == TMC_MEM_INTF_WIDTH_128BITS)
		memwords = 4;
	else
		memwords = 8;

	bufp = drvdata->buf;
	while (1) {
		read_data = tmc_readl(drvdata, TMC_RRD);
		if (read_data == 0xFFFFFFFF)
			return;
		if ((bufp - drvdata->buf) >= drvdata->size) {
			dev_err(drvdata->dev, "ETF-ETB end marker missing\n");
			return;
		}
		memcpy(bufp, &read_data, BYTES_PER_WORD);
		bufp += BYTES_PER_WORD;
	}
}

static void __tmc_etb_disable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_etb_dump(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etr_dump(struct tmc_drvdata *drvdata)
{
	uint32_t rwp, rwphi;

	rwp = tmc_readl(drvdata, TMC_RWP);
	rwphi = tmc_readl(drvdata, TMC_RWPHI);

	if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG) {
		if (BVAL(tmc_readl(drvdata, TMC_STS), 0))
			drvdata->buf = drvdata->vaddr + rwp - drvdata->paddr;
		else
			drvdata->buf = drvdata->vaddr;
	}
}

static void __tmc_etr_disable_to_mem(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_etr_dump(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etf_disable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void tmc_disable(struct tmc_drvdata *drvdata, enum tmc_mode mode)
{
	unsigned long flags;

	mutex_lock(&drvdata->usb_lock);
	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_disable_to_mem(drvdata);
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_disable(drvdata);
		else
			__tmc_etf_disable(drvdata);
	}
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		coresight_cti_unmap_trigin(drvdata->cti_reset, 2, 0);
		coresight_cti_unmap_trigout(drvdata->cti_flush, 1, 0);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {
			tmc_etr_byte_cntr_stop(drvdata);
			coresight_cti_unmap_trigin(drvdata->cti_reset,
						   2, 0);
			coresight_cti_unmap_trigout(drvdata->cti_flush,
						    3, 0);
		}
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			coresight_cti_unmap_trigin(drvdata->cti_reset, 2, 0);
			coresight_cti_unmap_trigout(drvdata->cti_flush, 1, 0);
		}
	}
	mutex_unlock(&drvdata->usb_lock);

	clk_disable_unprepare(drvdata->clk);

	dev_info(drvdata->dev, "TMC disabled\n");
	return;
out:
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	mutex_unlock(&drvdata->usb_lock);

	clk_disable_unprepare(drvdata->clk);

	dev_info(drvdata->dev, "TMC disabled\n");
}

static void tmc_disable_sink(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	tmc_disable(drvdata, TMC_MODE_CIRCULAR_BUFFER);
}

static void tmc_disable_link(struct coresight_device *csdev, int inport,
			     int outport)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	tmc_disable(drvdata, TMC_MODE_HARDWARE_FIFO);
}

static void tmc_abort(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);
	unsigned long flags;
	enum tmc_mode mode;

	drvdata->aborting = true;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading)
		goto out0;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_disable_to_mem(drvdata);
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_disable(drvdata);
		else
			goto out1;
	}
out0:
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC aborted\n");
	return;
out1:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
}

static const struct coresight_ops_sink tmc_sink_ops = {
	.enable		= tmc_enable_sink,
	.disable	= tmc_disable_sink,
	.abort		= tmc_abort,
};

static const struct coresight_ops_link tmc_link_ops = {
	.enable		= tmc_enable_link,
	.disable	= tmc_disable_link,
};

static const struct coresight_ops tmc_etb_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
};

static const struct coresight_ops tmc_etr_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
};

static const struct coresight_ops tmc_etf_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
	.link_ops	= &tmc_link_ops,
};

static int tmc_read_prepare(struct tmc_drvdata *drvdata)
{
	int ret;
	unsigned long flags;
	enum tmc_mode mode;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (!drvdata->sticky_enable) {
		dev_err(drvdata->dev, "enable tmc once before reading\n");
		ret = -EPERM;
		goto err;
	}
	if (!drvdata->enable)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {
			__tmc_etr_disable_to_mem(drvdata);
		} else {
			ret = -ENODEV;
			goto err;
		}
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			__tmc_etb_disable(drvdata);
		} else {
			ret = -ENODEV;
			goto err;
		}
	}
out:
	drvdata->reading = true;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read start\n");
	return 0;
err:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	return ret;
}

static void tmc_read_unprepare(struct tmc_drvdata *drvdata)
{
	unsigned long flags;
	enum tmc_mode mode;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (!drvdata->enable)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_enable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_enable_to_mem(drvdata);
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_enable(drvdata);
	}
out:
	drvdata->reading = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read end\n");
}

static int tmc_open(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	int ret = 0;

	mutex_lock(&drvdata->read_lock);
	if (drvdata->read_count++)
		goto out;

	ret = tmc_read_prepare(drvdata);
	if (ret)
		goto err;
out:
	mutex_unlock(&drvdata->read_lock);
	nonseekable_open(inode, file);

	dev_dbg(drvdata->dev, "%s: successfully opened\n", __func__);
	return 0;
err:
	drvdata->read_count--;
	mutex_unlock(&drvdata->read_lock);
	return ret;
}

static ssize_t tmc_read(struct file *file, char __user *data, size_t len,
			loff_t *ppos)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	char *bufp = drvdata->buf + *ppos;
	char *end = (char *)(drvdata->vaddr + drvdata->size);

	if (*ppos + len > drvdata->size)
		len = drvdata->size - *ppos;

	/*
	 * We do not expect len to become zero after this point. Hence bail out
	 * from here if len is zero
	 */
	if (len == 0)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG) {
			if (bufp == end)
				bufp = drvdata->vaddr;
			else if (bufp > end)
				bufp -= drvdata->size;
			if ((bufp + len) > end)
				len = end - bufp;
		}
	}

	if (copy_to_user(data, bufp, len)) {
		dev_dbg(drvdata->dev, "%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}

	*ppos += len;
out:
	dev_dbg(drvdata->dev, "%s: %zu bytes copied, %d bytes left\n",
		__func__, len, (int) (drvdata->size - *ppos));
	return len;
}

static int tmc_release(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);

	mutex_lock(&drvdata->read_lock);
	if (--drvdata->read_count) {
		if (drvdata->read_count < 0) {
			WARN_ONCE(1, "mismatched close\n");
			drvdata->read_count = 0;
		}
		goto out;
	}

	tmc_read_unprepare(drvdata);
out:
	mutex_unlock(&drvdata->read_lock);
	dev_dbg(drvdata->dev, "%s: released\n", __func__);
	return 0;
}

static const struct file_operations tmc_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_open,
	.read		= tmc_read,
	.release	= tmc_release,
	.llseek		= no_llseek,
};

static int tmc_etr_byte_cntr_open(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(inode->i_cdev,
						   struct tmc_drvdata,
						   byte_cntr_dev);

	if (drvdata->out_mode != TMC_ETR_OUT_MODE_MEM ||
	    !drvdata->byte_cntr_enable)
		return -EPERM;

	if (!mutex_trylock(&drvdata->byte_cntr_read_lock))
		return -EPERM;

	file->private_data = drvdata;
	nonseekable_open(inode, file);
	drvdata->byte_cntr_block_size = drvdata->byte_cntr_value * 8;
	drvdata->byte_cntr_read_active = true;
	dev_dbg(drvdata->dev, "%s: successfully opened\n", __func__);
	return 0;
}

static void tmc_etr_read_bytes(struct tmc_drvdata *drvdata, loff_t *ppos,
			       size_t bytes, size_t *len)
{
	if (*len >= bytes) {
		atomic_dec(&drvdata->byte_cntr_irq_cnt);
		*len = bytes;
	} else {
		if (((uint32_t)*ppos % bytes) + *len > bytes)
				*len = bytes - ((uint32_t)*ppos % bytes);
		if ((*len + (uint32_t)*ppos) % bytes == 0)
			atomic_dec(&drvdata->byte_cntr_irq_cnt);
	}
}

static size_t tmc_etr_flush_bytes(struct tmc_drvdata *drvdata, loff_t *ppos,
				  size_t bytes)
{
	uint32_t rwp = 0;
	size_t len = bytes;

	rwp = tmc_etr_get_write_ptr(drvdata);
	if (rwp >= (drvdata->paddr + *ppos)) {
		if (len > (rwp - drvdata->paddr - *ppos))
			len = rwp - drvdata->paddr - *ppos;
	}
	return len;
}

static ssize_t tmc_etr_byte_cntr_read(struct file *file, char __user *data,
				  size_t len, loff_t *ppos)
{
	struct tmc_drvdata *drvdata = file->private_data;
	char *bufp = drvdata->vaddr + *ppos;
	size_t bytes = drvdata->byte_cntr_block_size;
	int ret = 0;

	if (!data)
		return -EINVAL;
	if (drvdata->byte_cntr_overflow)
		return -EIO;

	mutex_lock(&drvdata->byte_cntr_lock);
	/* In case the byte counter is enabled and disabled multiple times
	 * prevent unexpected data from being given to the user
	 */
	if (!drvdata->byte_cntr_read_active)
		goto read_err0;

	if (!drvdata->byte_cntr_enable) {
		if (!atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			/* Read the last 'block' of data which might be needed
			 * to be read partially. If already read, return 0
			 */
			if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
				len = tmc_etr_flush_bytes(drvdata, ppos, bytes);

			if (!len)
				goto read_err0;
		} else {
			/* Keep reading until you reach the last block of data
			 */
			if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
				tmc_etr_read_bytes(drvdata, ppos, bytes, &len);
		}
	} else {
		if (!atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			mutex_unlock(&drvdata->byte_cntr_lock);
			if (wait_event_interruptible(drvdata->wq,
			    (atomic_read(&drvdata->byte_cntr_irq_cnt) > 0) ||
			    !drvdata->byte_cntr_enable)) {
				ret = -ERESTARTSYS;
				goto read_err1;
			}
			mutex_lock(&drvdata->byte_cntr_lock);
			if (!drvdata->byte_cntr_read_active) {
				ret = 0;
				goto read_err0;
			}
		}
		if (drvdata->byte_cntr_overflow) {
			ret = -EIO;
			goto read_err0;
		}
		if (!drvdata->byte_cntr_enable &&
		    !atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
				len = tmc_etr_flush_bytes(drvdata, ppos, bytes);

			if (!len) {
				ret = 0;
				goto read_err0;
			}
		} else {
			if (drvdata->memtype == TMC_ETR_MEM_TYPE_CONTIG)
				tmc_etr_read_bytes(drvdata, ppos, bytes, &len);
		}
	}

	if (copy_to_user(data, bufp, len)) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		dev_dbg(drvdata->dev, "%s: copy_to_user failed\n", __func__);
		ret = -EFAULT;
		goto read_err1;
	}
	mutex_unlock(&drvdata->byte_cntr_lock);

	if (*ppos + len >= drvdata->size)
		*ppos = 0;
	else
		*ppos += len;

	dev_dbg(drvdata->dev, "%s: %zu bytes copied, %d bytes left\n",
		__func__, len, (int) (drvdata->size - *ppos));
	return len;

read_err0:
	mutex_unlock(&drvdata->byte_cntr_lock);
read_err1:
	return ret;
}

static int tmc_etr_byte_cntr_release(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = file->private_data;

	mutex_lock(&drvdata->byte_cntr_lock);
	drvdata->byte_cntr_read_active = false;
	mutex_unlock(&drvdata->byte_cntr_lock);
	mutex_unlock(&drvdata->byte_cntr_read_lock);
	dev_dbg(drvdata->dev, "%s: released\n", __func__);
	return 0;
}

static const struct file_operations byte_cntr_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_etr_byte_cntr_open,
	.read		= tmc_etr_byte_cntr_read,
	.release	= tmc_etr_byte_cntr_release,
	.llseek		= no_llseek,
};

static ssize_t tmc_show_trigger_cntr(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->trigger_cntr;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_store_trigger_cntr(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	drvdata->trigger_cntr = val;
	return size;
}
static DEVICE_ATTR(trigger_cntr, S_IRUGO | S_IWUSR, tmc_show_trigger_cntr,
		   tmc_store_trigger_cntr);

static ssize_t tmc_etr_show_byte_cntr_value(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->byte_cntr_value;

	if (!drvdata->byte_cntr_present)
		return -EPERM;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_etr_store_byte_cntr_value(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	mutex_lock(&drvdata->byte_cntr_lock);
	if (!drvdata->byte_cntr_present || drvdata->byte_cntr_enable) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		return -EPERM;
	}
	if (sscanf(buf, "%lx", &val) != 1) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		return -EINVAL;
	}
	if ((drvdata->size / 8) < val) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		return -EINVAL;
	}
	if (val && drvdata->size % (val * 8) != 0) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		return -EINVAL;
	}

	drvdata->byte_cntr_value = val;
	mutex_unlock(&drvdata->byte_cntr_lock);
	return size;
}
static DEVICE_ATTR(byte_cntr_value, S_IRUGO | S_IWUSR,
		   tmc_etr_show_byte_cntr_value, tmc_etr_store_byte_cntr_value);

static ssize_t tmc_etr_show_mem_size(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->mem_size;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_etr_store_mem_size(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	mutex_lock(&drvdata->usb_lock);
	if (sscanf(buf, "%lx", &val) != 1) {
		mutex_unlock(&drvdata->usb_lock);
		return -EINVAL;
	}

	drvdata->mem_size = val;
	mutex_unlock(&drvdata->usb_lock);
	return size;
}
static DEVICE_ATTR(mem_size, S_IRUGO | S_IWUSR,
		   tmc_etr_show_mem_size, tmc_etr_store_mem_size);

static struct attribute *tmc_attrs[] = {
	&dev_attr_trigger_cntr.attr,
	NULL,
};

static struct attribute_group tmc_attr_grp = {
	.attrs = tmc_attrs,
};

static struct attribute *tmc_etr_attrs[] = {
	&dev_attr_byte_cntr_value.attr,
	&dev_attr_mem_size.attr,
	NULL,
};

static struct attribute_group tmc_etr_attr_grp = {
	.attrs = tmc_etr_attrs,
};

static const struct attribute_group *tmc_etb_attr_grps[] = {
	&tmc_attr_grp,
	NULL,
};

static const struct attribute_group *tmc_etr_attr_grps[] = {
	&tmc_attr_grp,
	&tmc_etr_attr_grp,
	NULL,
};

static const struct attribute_group *tmc_etf_attr_grps[] = {
	&tmc_attr_grp,
	NULL,
};

static irqreturn_t tmc_etr_byte_cntr_irq(int irq, void *data)
{
	struct tmc_drvdata *drvdata = data;

	atomic_inc(&drvdata->byte_cntr_irq_cnt);
	if (atomic_read(&drvdata->byte_cntr_irq_cnt) >
			drvdata->byte_cntr_overflow_cnt) {
		dev_err_ratelimited(drvdata->dev, "Byte counter overflow\n");
		drvdata->byte_cntr_overflow = true;
	}
	wake_up(&drvdata->wq);
	return IRQ_HANDLED;
}

static int tmc_etr_byte_cntr_dev_register(struct tmc_drvdata *drvdata)
{
	int ret;
	struct device *device;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, drvdata->byte_cntr_node);
	if (ret)
		goto err0;

	cdev_init(&drvdata->byte_cntr_dev, &byte_cntr_fops);

	drvdata->byte_cntr_dev.owner = THIS_MODULE;
	drvdata->byte_cntr_dev.ops = &byte_cntr_fops;
	ret = cdev_add(&drvdata->byte_cntr_dev, dev, 1);
	if (ret)
		goto err1;

	drvdata->byte_cntr_class = class_create(THIS_MODULE,
						drvdata->byte_cntr_node);
	if (IS_ERR(drvdata->byte_cntr_class)) {
		ret = PTR_ERR(drvdata->byte_cntr_class);
		goto err2;
	}

	device = device_create(drvdata->byte_cntr_class, NULL,
			       drvdata->byte_cntr_dev.dev, drvdata,
			       drvdata->byte_cntr_node);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto err3;
	}

	return 0;
err3:
	class_destroy(drvdata->byte_cntr_class);
err2:
	cdev_del(&drvdata->byte_cntr_dev);
err1:
	unregister_chrdev_region(drvdata->byte_cntr_dev.dev, 1);
err0:
	return ret;
}

static void tmc_etr_byte_cntr_dev_deregister(struct tmc_drvdata *drvdata)
{
	device_destroy(drvdata->byte_cntr_class, drvdata->byte_cntr_dev.dev);
	class_destroy(drvdata->byte_cntr_class);
	cdev_del(&drvdata->byte_cntr_dev);
	unregister_chrdev_region(drvdata->byte_cntr_dev.dev, 1);
}

static int tmc_etr_byte_cntr_init(struct platform_device *pdev,
				  struct tmc_drvdata *drvdata)
{
	int ret = 0;
	size_t node_size = strlen("-stream") + 1;
	char *node_name = (char *)((struct coresight_platform_data *)
			(pdev->dev.platform_data))->name;

	if (!drvdata->byte_cntr_present) {
		dev_info(&pdev->dev, "Byte Counter feature absent\n");
		goto out;
	}

	init_waitqueue_head(&drvdata->wq);

	drvdata->byte_cntr_irq = platform_get_irq_byname(pdev,
							"byte-cntr-irq");
	if (drvdata->byte_cntr_irq < 0) {
		/* Even though this is an error condition, we do not fail
		 * the probe as the byte counter feature is optional
		 */
		dev_err(&pdev->dev, "Byte-cntr-irq not specified\n");
		goto err;
	}

	ret = devm_request_irq(&pdev->dev, drvdata->byte_cntr_irq,
			tmc_etr_byte_cntr_irq,
			IRQF_TRIGGER_RISING | IRQF_SHARED,
			node_name, drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Request irq failed\n");
		goto err;
	}

	node_size += strlen(node_name);

	drvdata->byte_cntr_node = devm_kzalloc(&pdev->dev,
					       node_size, GFP_KERNEL);
	if (!drvdata->byte_cntr_node) {
		dev_err(&pdev->dev, "Byte cntr node name allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	strlcpy(drvdata->byte_cntr_node, node_name, node_size);
	strlcat(drvdata->byte_cntr_node, "-stream", node_size);

	ret = tmc_etr_byte_cntr_dev_register(drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Byte cntr node not registered\n");
		goto err;
	}

	dev_info(&pdev->dev, "Byte Counter feature enabled\n");
	return 0;
err:
	drvdata->byte_cntr_present = false;
out:
	return ret;
}

static void tmc_etr_byte_cntr_exit(struct tmc_drvdata *drvdata)
{
	if (drvdata->byte_cntr_present)
		tmc_etr_byte_cntr_dev_deregister(drvdata);
}

static int tmc_probe(struct platform_device *pdev)
{
	int ret;
	uint32_t devid;
	struct device *dev = &pdev->dev;
	struct coresight_platform_data *pdata;
	struct tmc_drvdata *drvdata;
	struct resource *res;
	uint32_t reg_size;
	static int etfetb_count;
	static int count;
	struct coresight_cti_data *ctidata;
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
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	reg_size = resource_size(res);

	drvdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->base)
		return -ENOMEM;

	spin_lock_init(&drvdata->spinlock);
	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->usb_lock);
	mutex_init(&drvdata->byte_cntr_lock);
	mutex_init(&drvdata->byte_cntr_read_lock);
	atomic_set(&drvdata->byte_cntr_irq_cnt, 0);

	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		drvdata->clk = devm_clk_get(dev, "core_clk");
		if (IS_ERR(drvdata->clk))
			return PTR_ERR(drvdata->clk);
	}

	ret = clk_set_rate(drvdata->clk, CORESIGHT_CLK_RATE_TRACE);
	if (ret)
		return ret;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		return ret;

	devid = tmc_readl(drvdata, CORESIGHT_DEVID);
	drvdata->config_type = BMVAL(devid, 6, 7);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		ret = device_property_read_u32(&pdev->dev, "qcom,memory-size",
					       &drvdata->size);
		if (ret) {
			clk_disable_unprepare(drvdata->clk);
			return ret;
		}

		drvdata->mem_size = drvdata->size;
	} else {
		drvdata->size = tmc_readl(drvdata, TMC_RSZ) * BYTES_PER_WORD;
	}

	clk_disable_unprepare(drvdata->clk);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		drvdata->out_mode = TMC_ETR_OUT_MODE_MEM;

		drvdata->memtype = TMC_ETR_MEM_TYPE_CONTIG;

		drvdata->mem_type = drvdata->memtype;

		ret = dma_coerce_mask_and_coherent(&pdev->dev,
						   DMA_BIT_MASK(64));
		if (ret)
			goto err0;

		drvdata->byte_cntr_present = !device_property_read_bool
					     (&pdev->dev,
					     "qcom,byte-cntr-absent");

		ret = tmc_etr_byte_cntr_init(pdev, drvdata);
		if (ret)
			goto err0;
	} else {
		drvdata->buf = devm_kzalloc(dev, drvdata->size,
					    GFP_KERNEL);
		if (!drvdata->buf)
			return -ENOMEM;

		etfetb_count++;
	}

	count++;

	if (pdev->dev.of_node) {
		ctidata = of_get_coresight_cti_data(dev, pdev->dev.of_node);
		if (IS_ERR(ctidata)) {
			dev_err(dev, "invalid cti data\n");
		} else if (ctidata && ctidata->nr_ctis == 2) {
			drvdata->cti_flush = coresight_cti_get(
							ctidata->names[0]);
			if (IS_ERR(drvdata->cti_flush))
				dev_err(dev, "failed to get flush cti\n");

			drvdata->cti_reset = coresight_cti_get(
							ctidata->names[1]);
			if (IS_ERR(drvdata->cti_reset))
				dev_err(dev, "failed to get reset cti\n");
		}
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto err1;
	}
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		desc->type = CORESIGHT_DEV_TYPE_SINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->ops = &tmc_etb_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etb_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err1;
		}
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		desc->type = CORESIGHT_DEV_TYPE_SINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->ops = &tmc_etr_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etr_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err1;
		}
	} else {
		desc->type = CORESIGHT_DEV_TYPE_LINKSINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->subtype.link_subtype = CORESIGHT_DEV_SUBTYPE_LINK_FIFO;
		desc->ops = &tmc_etf_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etf_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err1;
		}
	}

	drvdata->miscdev.name = ((struct coresight_platform_data *)
				 (pdev->dev.platform_data))->name;
	drvdata->miscdev.minor = MISC_DYNAMIC_MINOR;
	drvdata->miscdev.fops = &tmc_fops;
	ret = misc_register(&drvdata->miscdev);
	if (ret)
		goto err2;

	dev_info(dev, "TMC initialized\n");
	return 0;
err2:
	coresight_unregister(drvdata->csdev);
err1:
	tmc_etr_byte_cntr_exit(drvdata);
err0:
	return ret;
}

static int tmc_remove(struct platform_device *pdev)
{
	struct tmc_drvdata *drvdata = platform_get_drvdata(pdev);

	tmc_etr_byte_cntr_exit(drvdata);
	misc_deregister(&drvdata->miscdev);
	coresight_unregister(drvdata->csdev);
	tmc_etr_free_mem(drvdata);

	return 0;
}

static struct of_device_id tmc_match[] = {
	{.compatible = "arm,coresight-tmc"},
	{}
};

#if IS_ENABLED(CONFIG_ACPI)
static const struct acpi_device_id tmc_acpi_match[] = {
	{ "ARMHFFF0"  },
	{ },
};
MODULE_DEVICE_TABLE(acpi, tmc_acpi_match);
#endif

static struct platform_driver tmc_driver = {
	.probe          = tmc_probe,
	.remove         = tmc_remove,
	.driver         = {
		.name   = "coresight-tmc",
		.owner	= THIS_MODULE,
		.of_match_table = tmc_match,
		.acpi_match_table = ACPI_PTR(tmc_acpi_match),
	},
};

static int __init tmc_init(void)
{
	return platform_driver_register(&tmc_driver);
}
module_init(tmc_init);

static void __exit tmc_exit(void)
{
	platform_driver_unregister(&tmc_driver);
}
module_exit(tmc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight Trace Memory Controller driver");
