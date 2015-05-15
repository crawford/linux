/*
 * Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>

#include "qiommu.h"
#include "qiommu_hw.h"
#include "qiommu_regs.h"

#define qiommu_get_field(in, reg, field) ({ \
	(((in)&reg##__##field##__M)>>reg##__##field##__S); \
})

#define qiommu_set_field(io, reg, field, val) ({ \
	io &= ~reg##__##field##__M; \
	io |= ((val)<<reg##__##field##__S)&reg##__##field##__M; \
	(io); \
})

/* Allocate a page worth of commands by default [log2(sizeof(cmd))=4] */
#define DEFAULT_CMDQ_LOG2SIZE (PAGE_SHIFT - 4)

/* Allocate a page worth of events by default [log2(sizeof(evt))=5] */
#define DEFAULT_EVTQ_LOG2SIZE (PAGE_SHIFT - 5)

struct qiommu_hw {
	struct qiommu_device *qmmu;
	void __iomem *base;

	uint32_t idr[6];

	struct qiommu_ste *stes;
	struct qiommu_cmd *cmdq;
	struct qiommu_evt *evtq;

	spinlock_t strtab_lock;
	spinlock_t atos_lock;

	/* Command Queue */
	spinlock_t cmdq_lock;
	unsigned long cmdq_lock_flags;
	uint64_t raw_cmd_idx;
	uint32_t cmds_available;
	uint8_t cmdq_log2size;
	bool cmdq_sync_needed;

	/* Command queue usage data */
	uint64_t total_syncs;
	uint64_t total_syncs_skipped;
	uint64_t total_flushes;
	uint64_t total_flushes_from_alloc;
	uint64_t max_cmds_allocated;

	/* Event Queue */
	uint8_t evtq_log2size;

	/* Event Queue usage data */
	uint64_t total_evts_handled;
};

static void handle_qiommu_evt(struct qiommu_hw *hw, uint32_t idx)
{
	struct qiommu_device *qmmu = hw->qmmu;
	struct qiommu_evt *evt = hw->evtq+idx;

	/* For now just print some basic information about the event */
	qiommu_info(qmmu, "event[%llu] : Type 0x%02llX : SID 0x%llX : VA 0x%llX : {0x%016llX 0x%016llX 0x%016llX 0x%016llX}\n",
		    hw->total_evts_handled++, rd_evt__Type(evt),
		    rd_evt__SID(evt), rd_evt__VA(evt), evt->raw[0], evt->raw[1],
		    evt->raw[2], evt->raw[3]);
}

uint32_t qiommu_hw_handle_eventq_interrupt(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint32_t events_processed = 0;
	uint32_t cons_idx, max_evtq_idx = (1<<hw->evtq_log2size)-1;
	const uint32_t evtq_pos_mask = EVENTQ_CONS__RD_WRAP__M |
				       EVENTQ_CONS__RD__M;
	uint32_t prod = readl_relaxed(hw->base + QIOMMU_EVENTQ_PROD);
	uint32_t cons = readl_relaxed(hw->base + QIOMMU_EVENTQ_CONS);

	/* Make sure there are events to process */
	if (prod == cons)
		return 0;

	qiommu_info(qmmu, "Got EventQ Interrupt [prod = 0x%08X] [cons = 0x%08X]\n",
		    prod, cons);

	/* Iterate until prod and cons point to the same evtq position */
	while ((prod^cons)&evtq_pos_mask) {
		cons_idx = qiommu_get_field(cons, EVENTQ_CONS, RD);
		handle_qiommu_evt(hw, cons_idx);
		events_processed++;

		/* Toggle cons.rd_wrap if necessary */
		if (++cons_idx > max_evtq_idx) {
			cons_idx = 0;
			cons ^= EVENTQ_CONS__RD_WRAP__M;
		}

		/* Inform the QIOMMU that the event slot is free */
		qiommu_set_field(cons, EVENTQ_CONS, RD, cons_idx);
		writel(cons, hw->base + QIOMMU_EVENTQ_CONS);

		prod = readl_relaxed(hw->base + QIOMMU_EVENTQ_PROD);
	}

	/* If there was an overflow, condition acknowledge it */
	if ((prod^cons)&EVENTQ_CONS__OVACKFLG__M) {
		qiommu_info(qmmu, "EventQ overflow was detected!\n");
		cons ^= EVENTQ_CONS__OVACKFLG__M;
		writel_relaxed(cons, hw->base + QIOMMU_EVENTQ_CONS);
	}

	return events_processed;
}

uint32_t qiommu_hw_handle_gerror_interrupt(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint32_t gerror, ef, efack;

	/* Check for GERROR : only valid if EF != EFACK */
	gerror = readl(hw->base + QIOMMU_GERROR);
	ef = qiommu_get_field(gerror, GERROR, EF);
	efack = qiommu_get_field(gerror, GERROR, EFACK);
	if (ef != efack) {
		uint32_t data[4];

		data[0] = readl_relaxed(hw->base + QIOMMU_GERROR_DATA_0);
		data[1] = readl_relaxed(hw->base + QIOMMU_GERROR_DATA_1);
		data[2] = readl_relaxed(hw->base + QIOMMU_GERROR_DATA_2);
		data[3] = readl_relaxed(hw->base + QIOMMU_GERROR_DATA_3);

		/* Set EFACK==EF to clear the error */
		writel(ef<<GERROR__EFACK__S, hw->base + QIOMMU_GERROR);

		qiommu_err(qmmu, "GERROR 0x%08X : %08X %08X %08X %08X\n",
			   gerror, data[0], data[1], data[2], data[3]);
		return 1; /* Processed a single gerror */
	}

	return 0;
}

int qiommu_hw_handle_event_interrupt(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint32_t evtq_ret = qiommu_hw_handle_eventq_interrupt(hw);
	uint32_t gerror_ret = qiommu_hw_handle_gerror_interrupt(hw);

	/* The handlers return the number of items processed */
	if (evtq_ret == 0 && gerror_ret == 0)
		qiommu_info(qmmu, "Interrupted but evtq and gerror empty!\n");

	return 0;
}

int qiommu_hw_handle_rst_inform_interrupt(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;

	pr_debug("-> %s (%s)\n", __func__, qmmu->name);

	/*
	 * When in global bypass mode, rst_inform interrupts should never be
	 * triggered.  If one has arrived, it means there is likely an error
	 * in the ACPI method performing an async reset for one of the masters.
	 */
	if (qmmu->use_global_bypass) {
		qiommu_err(qmmu, "Invalid rst_inform irq received while in bypass mode!!!\n");
		BUG();
	}

	if (readl(hw->base + QIOMMU_CR0) & CR0__QIOMMUEN__M)
		qiommu_hw_reset_prepare(qmmu);
	else
		qiommu_hw_reset_done(qmmu);

	return 0;
}

static int init_cmdq(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint64_t base_phys;
	uint32_t *cmdq_base_hl = (uint32_t *)&base_phys;
	uint8_t log2size = DEFAULT_CMDQ_LOG2SIZE;

	/* The command queue may have already been allocated */
	if (hw->cmdq)
		goto skip_cmdq_alloc;
	hw->cmdq = dmam_alloc_coherent(qmmu->dev,
				       (1<<log2size)*sizeof(*(hw->cmdq)),
				       &base_phys, GFP_KERNEL);
	if (!hw->cmdq) {
		qiommu_err(qmmu, "dmam_alloc_coherent failed for cmdq\n");
		return -ENOMEM;
	}

	qiommu_set_field(cmdq_base_hl[0], CMDQ_BASE_L, LOG2SIZE, log2size);
	writel_relaxed(cmdq_base_hl[0], hw->base + QIOMMU_CMDQ_BASE_L);
	writel_relaxed(cmdq_base_hl[1], hw->base + QIOMMU_CMDQ_BASE_H);
skip_cmdq_alloc:

	hw->raw_cmd_idx = 0;
	hw->cmdq_log2size = log2size;
	hw->cmds_available = 1 << log2size;
	hw->cmdq_sync_needed = false;

	hw->total_syncs = 0;
	hw->total_syncs_skipped = 0;
	hw->total_flushes = 0;
	hw->total_flushes_from_alloc = 0;
	hw->max_cmds_allocated = 0;

	return 0;
}

static int init_evtq(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint64_t base_phys;
	uint32_t *evtq_base_hl = (uint32_t *)&base_phys;
	uint8_t log2size = DEFAULT_EVTQ_LOG2SIZE;
	uint32_t cfg, err;

	/* The event queue may have already been allocated */
	if (hw->evtq)
		goto skip_evtq_alloc;
	hw->evtq = dmam_alloc_coherent(qmmu->dev,
				       (1<<log2size)*sizeof(*(hw->evtq)),
				       &base_phys, GFP_KERNEL);
	if (!hw->cmdq) {
		qiommu_err(qmmu, "dmam_alloc_coherent failed for evtq\n");
		return -ENOMEM;
	}

	qiommu_set_field(evtq_base_hl[0], EVENTQ_BASE_L, LOG2SIZE, log2size);
	writel_relaxed(evtq_base_hl[0], hw->base + QIOMMU_EVENTQ_BASE_L);
	writel_relaxed(evtq_base_hl[1], hw->base + QIOMMU_EVENTQ_BASE_H);
skip_evtq_alloc:

	memset(hw->evtq, 0, (1<<log2size)*sizeof(*(hw->evtq)));

	/* Enable the event queue interrupts */
	writel(EVENTQ_IRQ_CFG0_L__ENABLE__M,
	hw->base + QIOMMU_EVENTQ_IRQ_CFG0_L);
	writel(EVENTQ_IRQ_CFG0_H__UPDATE__M,
	hw->base + QIOMMU_EVENTQ_IRQ_CFG0_H);

	err = readl_poll_timeout_atomic(hw->base + QIOMMU_EVENTQ_IRQ_CFG0_H,
					cfg,
					!(cfg & EVENTQ_IRQ_CFG0_H__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for EVENTQ_IRQ_CFG.UPDATE to clear\n");
		return err;
	}

	hw->evtq_log2size = log2size;
	hw->total_evts_handled = 0;

	return 0;
}

static int init_stream_table(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint8_t sidsize;
	size_t strtab_size;
	uint64_t base_phys;
	uint32_t *base_phys_hl = (uint32_t *)&base_phys;
	uint32_t cfg = 0;

	/* Currently only linear stream table is supported */
	sidsize = qiommu_get_field(hw->idr[1], IDR1, SIDSIZE);
	strtab_size = (1<<sidsize)*sizeof(*(hw->stes));
	hw->stes = dmam_alloc_coherent(qmmu->dev, strtab_size,
				       &base_phys, GFP_KERNEL);
	if (!hw->stes) {
		qiommu_err(qmmu, "dmam_alloc_coherent failed for stream table\n");
		return -ENOMEM;
	}
	memset(hw->stes, 0, strtab_size);

	writel_relaxed(base_phys_hl[0], hw->base + QIOMMU_STRTAB_BASE_L);
	writel_relaxed(base_phys_hl[1], hw->base + QIOMMU_STRTAB_BASE_H);

	qiommu_set_field(cfg, STRTAB_BASE_CFG, LOG2SIZE, sidsize);
	writel_relaxed(cfg, hw->base + QIOMMU_STRTAB_BASE_CFG);

	return 0;
}

/* Clear any stale QIOMMU configuration and reset to POR state */
int qiommu_force_por_reg_state(struct qiommu_hw *hw)
{
	struct qiommu_device *qmmu = hw->qmmu;
	uint32_t reg;
	int err;

	/* If the QIOMMU is currently enabled, turn it off */
	reg = readl_relaxed(hw->base + QIOMMU_CR0);
	if (reg & CR0__QIOMMUEN__M) {
		qiommu_info(qmmu, "QIOMMU was already enabled! [CR0.QIOMMUEN=1]\n");
		writel(CR0__UPDATE__M, hw->base + QIOMMU_CR0);
		err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR0, reg,
					!(reg & CR0__UPDATE__M),
					0, 1000000);
		if (err) {
			qiommu_err(qmmu, "timed out waiting for CR0.UPDATE to clear in %s()\n",
				   __func__);
			return err;
		}
	}

	/* Clobber CR1 */
	writel(CR1__UPDATE__M, hw->base + QIOMMU_CR1);
	err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR1, reg,
					!(reg & CR1__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for CR1.UPDATE to clear in %s()\n",
			   __func__);
		return err;
	}

	/* Clear stale STRTAB, CMDQ, and EVENTQ config */
	writel(0, hw->base + QIOMMU_STRTAB_BASE_CFG);
	writel(0, hw->base + QIOMMU_STRTAB_BASE_L);
	writel(0, hw->base + QIOMMU_STRTAB_BASE_H);
	writel(0, hw->base + QIOMMU_CMDQ_BASE_L);
	writel(0, hw->base + QIOMMU_CMDQ_BASE_H);
	writel(0, hw->base + QIOMMU_EVENTQ_BASE_L);
	writel(0, hw->base + QIOMMU_EVENTQ_BASE_H);
	writel(0, hw->base + QIOMMU_EVENTQ_IRQ_CFG0_L);
	writel(EVENTQ_IRQ_CFG0_H__UPDATE__M,
	       hw->base + QIOMMU_EVENTQ_IRQ_CFG0_H);
	err = readl_poll_timeout_atomic(hw->base + QIOMMU_EVENTQ_IRQ_CFG0_H,
					reg,
					!(reg & EVENTQ_IRQ_CFG0_H__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for EVENTQ_IRQ_CFG0.UPDATE to clear in %s()\n",
			   __func__);
		return err;
	}

	/* Turn off the config cache and TLBs */
	writel(0, hw->base + QIOMMU_CFG_CACHE_CR);
	writel(TLBCR__POR, hw->base + QIOMMU_TLBCR);

	return 0;
}

int qiommu_set_global_bypass(struct qiommu_device *qmmu)
{
	struct qiommu_hw *hw = qmmu->hw;
	uint32_t reg;
	int err;

	writel(qmmu->global_bypass_attributes, hw->base + QIOMMU_GBPA_L);
	writel(GBPA_H__UPDATE__M, hw->base + QIOMMU_GBPA_H);
	err = readl_poll_timeout_atomic(hw->base + QIOMMU_GBPA_H, reg,
					!(reg & GBPA_H__UPDATE__M),
					0, 1000000);
	if (err)
		qiommu_err(qmmu, "timed out waiting for GBPA.UPDATE to clear in %s()\n",
			   __func__);

	return err;
}

int qiommu_hw_init(struct qiommu_device *qmmu,
			void __iomem *base)
{
	int err = 0;
	uint32_t cr0, cr1, cfg;
	struct qiommu_hw *hw;
	int i;

	BUG_ON(!qmmu || !base);

	hw = devm_kzalloc(qmmu->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;
	qmmu->hw = hw;

	spin_lock_init(&hw->strtab_lock);
	spin_lock_init(&hw->atos_lock);
	spin_lock_init(&hw->cmdq_lock);

	hw->base = base;
	hw->qmmu = qmmu;

	/* Read the QIOMMU ID registers */
	hw->idr[0] = readl(hw->base + QIOMMU_IDR0);
	hw->idr[1] = readl(hw->base + QIOMMU_IDR1);
	hw->idr[2] = readl(hw->base + QIOMMU_IDR2);
	hw->idr[3] = readl(hw->base + QIOMMU_IDR3);
	hw->idr[4] = readl(hw->base + QIOMMU_IDR4);
	hw->idr[5] = readl(hw->base + QIOMMU_IDR5);

	/* Make sure the QIOMMU is being initialized from a clean state */
	err = qiommu_force_por_reg_state(hw);
	if (err)
		return err;

	/* Apply impdef reg settings that may have been requested in ACPI/DT */
	for (i = 0; i < qmmu->num_impdef_reg_settings; i++) {
		uint32_t offset = qmmu->impdef_reg_settings[2*i];
		uint32_t val = qmmu->impdef_reg_settings[2*i+1];

		writel(val, hw->base + offset);
	}

	/* Put the QIOMMU into global bypass mode if it has been requested */
	if (qmmu->use_global_bypass)
		return qiommu_set_global_bypass(qmmu);

	/* Enable the global error interrupt */
	writel_relaxed(GERROR_IRQ_CFG0_L__ENABLE__M,
	       hw->base + QIOMMU_GERROR_IRQ_CFG0_L);
	writel(GERROR_IRQ_CFG0_H__UPDATE__M,
	       hw->base + QIOMMU_GERROR_IRQ_CFG0_H);

	err = readl_poll_timeout_atomic(hw->base + QIOMMU_GERROR_IRQ_CFG0_H,
					cfg,
					!(cfg & GERROR_IRQ_CFG0_H__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for GERROR_IRQ_CFG.UPDATE to clear\n");
		return err;
	}

	/* configure stream table */
	err = init_stream_table(hw);
	if (err)
		return err;

	/* configure command queue */
	err = init_cmdq(hw);
	if (err)
		return err;

	/* configure the event queue */
	err = init_evtq(hw);
	if (err)
		return err;

	/* Turn on the config cache */
	if (!qmmu->config_cache_disable)
		writel(CFG_CACHE_CR__CC_EN__M, hw->base + QIOMMU_CFG_CACHE_CR);

	/* Turn on the TLB */
	if (!qmmu->tlb_disable)
		writel(TLBCR__POR | TLBCR__TLB_EN__M, hw->base + QIOMMU_TLBCR);

	/* Configure the cacheability and shareability for the tables/queues */
	cr1 = readl_relaxed(hw->base + QIOMMU_CR1);
	if (is_device_dma_coherent(qmmu->dev)) {
		qiommu_set_field(cr1, CR1, QUEUE_IC, 0x1);
		qiommu_set_field(cr1, CR1, QUEUE_OC, 0x1);
		qiommu_set_field(cr1, CR1, QUEUE_SH, 0x3);
		qiommu_set_field(cr1, CR1, TABLE_IC, 0x1);
		qiommu_set_field(cr1, CR1, TABLE_OC, 0x1);
		qiommu_set_field(cr1, CR1, TABLE_SH, 0x3);
	} else {
		qiommu_set_field(cr1, CR1, QUEUE_IC, 0x0);
		qiommu_set_field(cr1, CR1, QUEUE_OC, 0x0);
		qiommu_set_field(cr1, CR1, QUEUE_SH, 0x2);
		qiommu_set_field(cr1, CR1, TABLE_IC, 0x0);
		qiommu_set_field(cr1, CR1, TABLE_OC, 0x0);
		qiommu_set_field(cr1, CR1, TABLE_SH, 0x2);
	}
	qiommu_set_field(cr1, CR1, UPDATE, 0x1);
	writel(cr1, hw->base + QIOMMU_CR1);

	err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR1, cr1,
					!(cr1 & CR1__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for CR1.UPDATE to clear in %s()\n",
			   __func__);
		return err;
	}

	/* Enable the CMDQ, EVTQ, and QIOMMU */
	cr0 = readl_relaxed(hw->base + QIOMMU_CR0);
	qiommu_set_field(cr0, CR0, EVQEN, 0x1);
	qiommu_set_field(cr0, CR0, CMDEN, 0x1);
	qiommu_set_field(cr0, CR0, QIOMMUEN, 0x1);
	qiommu_set_field(cr0, CR0, UPDATE, 0x1);
	writel(cr0, hw->base + QIOMMU_CR0);

	err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR0, cr0,
					!(cr0 & CR0__UPDATE__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "timed out waiting for CR0.UPDATE to clear in %s()\n",
			   __func__);
		return err;
	}

	/* Clear the TLB and CFG_CACHE */
	qiommu_hw_inv_all(qmmu);
	qiommu_hw_tlbi_all(qmmu);
	qiommu_hw_sync(qmmu, true);

	return 0;
}

phys_addr_t qiommu_hw_iova_to_phys(struct qiommu_master *master,
				   dma_addr_t iova)
{
	struct qiommu_device *qmmu = master->qmmu;
	uint16_t sid = master->sids[0];
	struct qiommu_hw *hw = qmmu->hw;
	uint32_t addr_l, addr_h;
	uint32_t par_l, par_h;
	uint64_t pa = 0;
	unsigned long flags;
	int err;

	spin_lock_irqsave(&hw->atos_lock, flags);

	addr_l = (uint32_t)iova;
	qiommu_set_field(addr_l, GATOS_ADDR_L, TYPE, 0x1); /* stage-1 lookup */
	addr_h = (uint32_t)(iova>>32);

	writel_relaxed(sid, hw->base + QIOMMU_GATOS_SID_L);
	writel_relaxed(addr_h, hw->base + QIOMMU_GATOS_ADDR_H);
	writel_relaxed(addr_l, hw->base + QIOMMU_GATOS_ADDR_L);

	qiommu_set_field(addr_l, GATOS_ADDR_L, RUN, 0x1);
	writel(addr_l, hw->base + QIOMMU_GATOS_ADDR_L);

	err = readl_poll_timeout_atomic(hw->base + QIOMMU_GATOS_ADDR_L, addr_l,
					!(addr_l & GATOS_ADDR_L__RUN__M),
					0, 1000000);
	if (err) {
		qiommu_err(qmmu, "master %s: timed out waiting for GATOS_ADDR.RUN to clear\n",
			   master->name);
		return ~0;
	}

	par_l = readl_relaxed(hw->base + QIOMMU_GATOS_PAR_L);
	par_h = readl_relaxed(hw->base + QIOMMU_GATOS_PAR_H);

	spin_unlock_irqrestore(&hw->atos_lock, flags);

	qiommu_dbg(qmmu, "master %s: GATOS(0x%llX,%u) -> PAR : %08X %08X\n",
		    master->name, iova, sid, par_h, par_l);

	if (qiommu_get_field(par_l, GATOS_PAR_L, FAULT)) {
		qiommu_err(qmmu, "master %s: FAULT detected during GATOS(0x%llX, 0x%X) : PAR = %08X %08X\n",
			   master->name, iova, sid, par_h, par_l);
		return ~0;
	}

	pa = par_h;
	pa <<= 32;
	pa |= par_l;
	pa &= PAGE_MASK;
	pa |= (iova & ~PAGE_MASK);

	return pa;
}

static void configure_ste(uint16_t sid, struct qiommu_master *master)
{
	struct qiommu_domain *dom = master->dom;
	struct qiommu_device *qmmu = master->qmmu;
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_ste *ste = hw->stes+sid;

	unsigned long flags;

	spin_lock_irqsave(&hw->strtab_lock, flags);

	/* Make sure the ste is clean */
	if (ste->raw[0] || ste->raw[1] || ste->raw[2] || ste->raw[3]) {
		uint16_t sid = ste - hw->stes;

		qiommu_warn(qmmu, "clobbering ste[sid=0x%X] : %016llX %016llX %016llX %016llX\n",
			    sid, ste->raw[0], ste->raw[1], ste->raw[2],
			    ste->raw[3]);
	}

	wr_ste__Config(ste, 0x5); /* S1Translate_S2Bypass */
	wr_ste__S1ContextPtr(ste, dom->ctx_phys);
	wr_ste__CONT(ste, 0x0);
	wr_ste__HD(ste, 0x0); /* HTTU disable */
	wr_ste__HA(ste, 0x0); /* HTTU disable */
	wr_ste__PIPA(ste, 0x0); /* Ignored if Config[1]=0 */
	wr_ste__DSS(ste, 0x0); /* Ignored if CDMax=0 */

	if (is_device_dma_coherent(qmmu->dev)) {
		wr_ste__CIR(ste, 0x1); /* Normal, WB Cacheable, read allocate */
		wr_ste__COR(ste, 0x1); /* Normal, WB Cacheable, read allocate */
		wr_ste__CSH(ste, 0x3); /* Inner Shareable */
	} else {
		wr_ste__CIR(ste, 0x0); /* Normal - NC */
		wr_ste__COR(ste, 0x0); /* Normal - NC */
		wr_ste__CSH(ste, 0x2); /* Default if CIR==0 && COR==0 */
	}

	wr_ste__CDMax(ste, 0x0); /* Single context (no substreams) */
	wr_ste__S1Fmt(ste, 0x0); /* Ignored if CDMax==0 */
	wr_ste__TTFmt(ste, 0x1); /* V8L Translation Tables */
	wr_ste__WXN(ste, 0x0); /* Instruction read is allowed... */
	wr_ste__UWXN(ste, 0x0); /* Instruction read is allowed... */
	wr_ste__S(ste, 0x0); /* Only valid for S2 */
	wr_ste__R(ste, 0x0); /* Only valid for S2 */
	wr_ste__A(ste, 0x0); /* Only valid for S2 */
	wr_ste__STALLD(ste, 0x1); /* Disable stalling.  Faults terminate. */
	wr_ste__EATS(ste, 0x0); /* Disable ATS */
	wr_ste__STRW(ste, 0x0); /* It's a mystery what this does */
	wr_ste__MemAttr(ste, 0xF); /* Inner Outer WB */
	wr_ste__MTCFG(ste, 0x1); /* MemAttr has inbound type/cacheability */
	wr_ste__ALLOCCFG(ste, 0xE); /* R/W Allocate */
	wr_ste__SHCFG(ste, 0x2); /* Inner Shareable */
	wr_ste__NSCFG(ste, 0x0); /* Use incoming */
	wr_ste__PRIVCFG(ste, 0x3); /* Incoming transactions are privileged */
	wr_ste__INSTCFG(ste, 0x0); /* Use incoming */
	wr_ste__S1_VMID(ste, 0x0); /* Does this matter for s1 only mappings? */

	/* Flush the ste configuration prior to setting it Valid */
	wmb();
	wr_ste__V(ste, 1); /* Valid */

	spin_unlock_irqrestore(&hw->strtab_lock, flags);
}

int qiommu_hw_reset_prepare(struct qiommu_device *qmmu)
{
	struct qiommu_hw *hw = qmmu->hw;
	uint32_t cr0;
	int err;

	/* Disable the QIOMMU in prep of a possible reset */
	cr0 = readl_relaxed(hw->base + QIOMMU_CR0);
	qiommu_set_field(cr0, CR0, EVQEN, 0x0);
	qiommu_set_field(cr0, CR0, CMDEN, 0x0);
	qiommu_set_field(cr0, CR0, QIOMMUEN, 0x0);
	qiommu_set_field(cr0, CR0, UPDATE, 0x1);
	writel(cr0, hw->base + QIOMMU_CR0);

	/*
	 * If we are in interrupt context, it means the request was triggered
	 * by the QIOMMU's ACPI RSTI() method which will take care of polling
	 * CR0 for us.
	 */
	if (!in_interrupt()) {
		err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR0, cr0,
						!(cr0 & CR0__UPDATE__M),
						0, 1000000);
		if (err) {
			qiommu_err(qmmu, "timed out waiting for CR0.UPDATE to clear in %s()\n",
				__func__);
			return err;
		}
	}

	qiommu_info(qmmu, "Disabled QIOMMU in case of possible reset\n");
	return 0;
}

int qiommu_hw_reset_done(struct qiommu_device *qmmu)
{
	struct qiommu_hw *hw = qmmu->hw;
	uint32_t cr0;
	int err;

	/* If CR0.QIOMMUEN is not set, the QIOMMU needs partial reinit */
	cr0 = readl(hw->base + QIOMMU_CR0);
	if (cr0 & CR0__QIOMMUEN__M)
		return 0;

	/* Reinitialize the cmdq and evtq */
	init_cmdq(hw);
	init_evtq(hw);

	/* Re-enable the QIOMMU */
	qiommu_set_field(cr0, CR0, EVQEN, 0x1);
	qiommu_set_field(cr0, CR0, CMDEN, 0x1);
	qiommu_set_field(cr0, CR0, QIOMMUEN, 0x1);
	qiommu_set_field(cr0, CR0, UPDATE, 0x1);
	writel(cr0, hw->base + QIOMMU_CR0);

	/*
	 * If we are in interrupt context, it means the request was triggered
	 * by the QIOMMU's ACPI RSTD() method which will take care of polling
	 * CR0 for us.
	 */
	if (!in_interrupt()) {
		err = readl_poll_timeout_atomic(hw->base + QIOMMU_CR0, cr0,
						!(cr0 & CR0__UPDATE__M),
						0, 1000000);
		if (err) {
			qiommu_err(qmmu, "timed out waiting for CR0.UPDATE to clear in %s()\n",
				__func__);
			return err;
		}
	}

	qiommu_info(qmmu, "Re-enabled QIOMMU following possible reset\n");

	return 0;
}

int qiommu_hw_cfg_master_stes(struct qiommu_master *master)
{
	struct qiommu_device *qmmu = master->qmmu;
	int i;

	for (i = 0; i < master->num_sids; i++) {
		qiommu_trc(master->qmmu, "master %s: configuring stream 0x%X\n",
			   master->name, master->sids[i]);
		configure_ste(master->sids[i], master);
	}

	/* CMD_INV_STE is not currently supported so use CMD_INV_ALL instead */
	mb(); /* Ensure ste updates happen prior to invalidation */
	qiommu_hw_inv_all(qmmu);
	qiommu_hw_sync(qmmu, true);

	return 0;
}

void qiommu_hw_free_master_stes(struct qiommu_master *master)
{
	struct qiommu_device *qmmu = master->qmmu;
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_ste *ste;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&hw->strtab_lock, flags);

	for (i = 0; i < master->num_sids; i++) {
		ste = hw->stes + master->sids[i];
		memset(ste, 0, sizeof(*ste));

		qiommu_trc(master->qmmu, "master %s: freeing stream 0x%X\n",
			   master->name, master->sids[i]);
	}

	/* CMD_INV_STE is not currently supported so use CMD_INV_ALL instead */
	mb(); /* Ensure ste updates happen prior to invalidation */
	qiommu_hw_inv_all(qmmu);
	qiommu_hw_sync(qmmu, true);

	spin_unlock_irqrestore(&hw->strtab_lock, flags);
}

int qiommu_hw_cfg_domain_ctx(struct qiommu_domain *dom)
{
	const struct io_pgtable_cfg *cfg = &dom->iopt_cfg;
	struct qiommu_ctx *ctx = dom->ctx;
	uint64_t tg0 = ((cfg->arm_lpae_s1_cfg.tcr)>>14)&0x3;

	/* Allocate the context */
	ctx = dmam_alloc_coherent(dom->dma_dev, sizeof(*ctx),
				  &dom->ctx_phys, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	dom->ctx = ctx;

	/* Configure the context based on the domain settings */
	wr_ctx__T0SZ(ctx, 64 - cfg->ias);
	wr_ctx__TG0(ctx, tg0);
	if (dom->qiommus_coherent) {
		wr_ctx__IR0(ctx, 0x1); /* Normal, WB Cacheable, read allocate */
		wr_ctx__OR0(ctx, 0x1); /* Normal, WB Cacheable, read allocate */
		wr_ctx__SH0(ctx, 0x3); /* Inner Shareable */
	} else {
		wr_ctx__IR0(ctx, 0); /* NonCacheable - Walks */
		wr_ctx__OR0(ctx, 0); /* NonCacheable - Walks */
		wr_ctx__SH0(ctx, 0x2); /* OSH - Default if IR0==OR0==NC */
	}

	wr_ctx__EPD0(ctx, 0); /* Perform TTWalks using TT0 */
	wr_ctx__ENDI(ctx, 0); /* Little Endian */
	wr_ctx__EPD1(ctx, 1); /* Disable walks using TT1 */
	wr_ctx__IPS(ctx, 0x5); /* 48-bit IPA */
	wr_ctx__AFFD(ctx, 1); /* Disable AF faults */
	wr_ctx__TBI(ctx, 0); /* does this matter?? */
	wr_ctx__PAN(ctx, 0); /* Enable privileged accesses */
	wr_ctx__S(ctx, 0); /* Never stall on fault */
	wr_ctx__R(ctx, 1); /* Record faults in the event log */
	wr_ctx__A(ctx, 1); /* Report aborts to upstream device */
	wr_ctx__ASET(ctx, 1); /* ASID entries in non-shared (with CPU) set */
	wr_ctx__ASID(ctx, dom->asid); /* ASID - just made up currently */
	wr_ctx__NSCFG0(ctx, 0); /* Only matters for secure streams */
	wr_ctx__HAD0(ctx, 0); /* Allow hierarchical attributes */
	wr_ctx__TTB0(ctx, cfg->arm_lpae_s1_cfg.ttbr[0]);
	wr_ctx__MAIR(ctx, cfg->arm_lpae_s1_cfg.mair[0]);

	/* Flush the ctx configuration prior to setting it Valid */
	wmb();
	wr_ctx__V(ctx, 1); /* Valid */

	return 0;
}

void qiommu_hw_free_domain_ctx(struct qiommu_domain *dom)
{
	struct qiommu_ctx *ctx = dom->ctx;

	dmam_free_coherent(dom->dma_dev, sizeof(*ctx), ctx, dom->ctx_phys);
	dom->ctx = NULL;
	dom->ctx_phys = ~(phys_addr_t)0;
}

#define CIRCBUF_IDX(raw_idx, log2size) \
	((raw_idx) & (~0ULL >> (64-(log2size))))
#define CIRCBUF_WRAP(raw_idx, log2size) \
	(((raw_idx) >> (log2size)) & 0x1)

#define CMDQ_POS(cons) \
	((cons)&(CMDQ_PROD__WR_WRAP__M|CMDQ_PROD__WR__M))
#define CMDQ_ERR(cons) \
	(qiommu_get_field((cons), CMDQ_CONS, ERR))
#define CMDQ_EMPTY(cons, prod) \
	(CMDQ_POS((cons)) == (prod))

static void acquire_cmdq(struct qiommu_hw *hw)
{
	spin_lock_irqsave(&hw->cmdq_lock, hw->cmdq_lock_flags);
}

static void release_cmdq(struct qiommu_hw *hw)
{
	spin_unlock_irqrestore(&hw->cmdq_lock, hw->cmdq_lock_flags);
}

static struct qiommu_cmd *clear_cmd(struct qiommu_cmd *cmd)
{
	cmd->raw[0] = cmd->raw[1] = 0;
	return cmd;
}

static int flush_cmdq(struct qiommu_hw *hw)
{
	uint32_t log2size = hw->cmdq_log2size;
	uint32_t raw_idx = hw->raw_cmd_idx;
	uint32_t cons, prod;
	int err;
	uint32_t cmds_allocated;

	prod = CIRCBUF_IDX(raw_idx, log2size);
	prod |= CIRCBUF_WRAP(raw_idx, log2size) << CMDQ_PROD__WR_WRAP__S;
	writel(prod, hw->base + QIOMMU_CMDQ_PROD);

	/*
	 * Wait until all the commands have been processed.  If there is an
	 * error while processing the commands (CONS.ERR!=0x0), readl_poll
	 * will timeout.  The extra wait is no big deal since the behavior in
	 * case of error is currently to BUG().  This will be updated once the
	 * proper CMDQ error handling procedure becomes available.
	 */
	err = readl_poll_timeout_atomic(hw->base + QIOMMU_CMDQ_CONS, cons,
					CMDQ_EMPTY(cons, prod), 0, 1000000);
	if (err) {
		qiommu_err(hw->qmmu,
			   "%s : timed out waiting for CMDQ to empty [PROD=0x%08X] [CONS=0x%08X]\n",
			   __func__, prod, cons);
		BUG();
	}

	cmds_allocated = (1<<log2size) - hw->cmds_available;
	if (cmds_allocated > hw->max_cmds_allocated)
		hw->max_cmds_allocated = cmds_allocated;
	hw->cmds_available = 1<<log2size;
	hw->total_flushes++;

	return err;
}

static struct qiommu_cmd *alloc_cmd(struct qiommu_hw *hw)
{
	uint32_t idx;

	/* If there aren't any commands currently available, flush the cmdq */
	if (0 == hw->cmds_available) {
		hw->total_flushes_from_alloc++;
		if (flush_cmdq(hw))
			return NULL;
	}

	idx = CIRCBUF_IDX(hw->raw_cmd_idx, hw->cmdq_log2size);
	hw->raw_cmd_idx++;
	hw->cmds_available--;
	return clear_cmd(hw->cmdq + idx);
}

int qiommu_hw_sync(struct qiommu_device *qmmu, bool force)
{
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_cmd *cmd;

	acquire_cmdq(hw);
	if (!force && !hw->cmdq_sync_needed) {
		hw->total_syncs_skipped++;
		goto done;
	}
	hw->total_syncs++;
	cmd = alloc_cmd(hw);
	wr_cmd__OPCODE(cmd, CMD_SYNC);
	flush_cmdq(hw);
done:
	hw->cmdq_sync_needed = false;
	release_cmdq(hw);

	return 0;
}

int qiommu_hw_inv_all(struct qiommu_device *qmmu)
{
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_cmd *cmd;

	acquire_cmdq(hw);
	cmd = alloc_cmd(hw);
	wr_cmd__OPCODE(cmd, CMD_INV_ALL);
	hw->cmdq_sync_needed = true;
	release_cmdq(hw);

	return 0;
}

int qiommu_hw_tlbi_all(struct qiommu_device *qmmu)
{
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_cmd *cmd;

	acquire_cmdq(hw);
	cmd = alloc_cmd(hw);
	wr_cmd__OPCODE(cmd, CMD_TLBI_ALL);
	hw->cmdq_sync_needed = true;
	release_cmdq(hw);

	return 0;
}

int qiommu_hw_tlbi_asid(struct qiommu_device *qmmu, uint16_t asid)
{
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_cmd *cmd;

	acquire_cmdq(hw);
	cmd = alloc_cmd(hw);
	wr_cmd__OPCODE(cmd, CMD_TLBI_ASID);
	wr_cmd__ASID(&hw->cmdq[0], asid);
	hw->cmdq_sync_needed = true;
	release_cmdq(hw);

	return 0;
}

int qiommu_hw_tlbi_va(struct qiommu_device *qmmu, uint16_t asid, uint64_t iova)
{
	struct qiommu_hw *hw = qmmu->hw;
	struct qiommu_cmd *cmd;

	acquire_cmdq(hw);
	cmd = alloc_cmd(hw);
	wr_cmd__OPCODE(cmd, CMD_TLBI_VA);
	wr_cmd__ASID(&hw->cmdq[0], asid);
	wr_cmd__ADDR(&hw->cmdq[0], iova);
	hw->cmdq_sync_needed = true;
	release_cmdq(hw);

	return 0;
}
