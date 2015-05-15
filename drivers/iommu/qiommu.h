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

#ifndef __QIOMMU_H__
#define __QIOMMU_H__

#include <linux/iommu.h>
#include "io-pgtable.h"

#define MAX_STREAMS_PER_MASTER 16

struct qiommu_domain;
struct qiommu_master;
struct qiommu_hw;
struct qiommu_ctx;

struct qiommu_device {
	struct list_head list; /* qiommu's list entry in qiommu_devices */

	struct device *dev;
	struct qiommu_hw *hw;
	struct qiommu_top_data *top;

	/* QIOMMU config options that may be specified in DT/ACPI */
	const char *name;
	bool use_global_bypass;
	uint32_t global_bypass_attributes;
	bool tlb_disable;
	bool config_cache_disable;
	bool enable_delayed_hw_sync_after_tlbi;
	uint64_t *static_mappings;
	uint32_t num_static_mappings;
	uint32_t *impdef_reg_settings;
	uint32_t num_impdef_reg_settings;
	uint32_t num_iova_bits;
	bool identity_map_enable;
	bool is_reserved;
	bool tracing_enabled;
};

/*
 * This structure is used to create qiommu_device lists for qiommu_domains.
 * We cant just use a list_head in the qiommu_device structure itself because
 * a single qiommu_device may be used by many qiommu_domains.
 */
struct qiommu_list_node {
	struct list_head list;
	struct qiommu_device *qmmu;
	uint16_t dups; /* # duplicate qiommus in the list */
};

struct qiommu_domain {
	struct iommu_domain iommu_dom;

	struct io_pgtable_cfg iopt_cfg;
	struct io_pgtable_ops *iopt_ops;
	bool tlb_dirty; /* If set, a hw_sync is needed before map */

	struct qiommu_ctx *ctx;
	phys_addr_t ctx_phys;
	uint16_t asid;

	struct list_head qmmu_list; /* head for list of qiommu_list_node(s) */
	uint32_t qmmu_list_length;

	/*
	 * When a qiommu_domain is used by more than one qiommu_device, the
	 * decision of whose dev to use for dma routines requiring a struct
	 * device gets tricky. Using the first device in the qmmu_list is
	 * problematic because the first device in the list when allocating a
	 * buffer may not be the first device in the list when that same buffer
	 * is free'd. To eliminate any such potentially inconsistent scenarios,
	 * dma_dev should be used when calling dma routines relevant to the
	 * domain itself. The most prominent examples of this being allocating
	 * or freeing ctx and also flushing PTEs in non-coherent domains.
	 */
	struct device *dma_dev;
	bool qiommus_coherent;
	bool masters_coherent;

	spinlock_t iopt_and_list_lock; /* Protects page tables and qmmu_list */
	struct mutex mtx; /* Protects remaining member data */

	bool init_done;
};

static inline
struct qiommu_domain *to_qiommu_domain(struct iommu_domain *domain)
{
	return container_of(domain, struct qiommu_domain, iommu_dom);
}

struct qiommu_master {
	struct device *dev;
	struct qiommu_device *qmmu;
	struct qiommu_domain *dom;
	struct qiommu_domain *default_dom;

	uint16_t sids[MAX_STREAMS_PER_MASTER];
	uint16_t num_sids;

	const char *name;

	bool use_identity_mapping;
};

/* The master for a struct device will be stored in archdata.iommu */
#define get_dev_master(d) ((d)->archdata.iommu)
#define set_dev_master(d, m) (((d)->archdata.iommu) = (m))

int qiommu_hw_init(struct qiommu_device *qmmu, void __iomem *base);
int qiommu_hw_cfg_domain_ctx(struct qiommu_domain *dom);
void qiommu_hw_free_domain_ctx(struct qiommu_domain *dom);
int qiommu_hw_cfg_master_stes(struct qiommu_master *master);
void qiommu_hw_free_master_stes(struct qiommu_master *master);
phys_addr_t qiommu_hw_iova_to_phys(struct qiommu_master *master,
				   dma_addr_t iova);

int qiommu_hw_handle_event_interrupt(struct qiommu_hw *hw);
int qiommu_hw_handle_rst_inform_interrupt(struct qiommu_hw *hw);
int qiommu_hw_sync(struct qiommu_device *qmmu, bool force);
int qiommu_hw_inv_all(struct qiommu_device *qmmu);
int qiommu_hw_tlbi_all(struct qiommu_device *qmmu);
int qiommu_hw_tlbi_asid(struct qiommu_device *qmmu, uint16_t asid);
int qiommu_hw_tlbi_va(struct qiommu_device *qmmu, uint16_t asid, uint64_t iova);

int qiommu_hw_reset_prepare(struct qiommu_device *qmmu);
int qiommu_hw_reset_done(struct qiommu_device *qmmu);

struct qiommu_master *qiommu_top_alloc_master(struct device *dev);
struct qiommu_master *qiommu_top_find_master(struct device *dev);
void qiommu_top_free_master(struct qiommu_master *master);
int qiommu_top_add_qiommu(struct qiommu_device *qmmu);

/*
 * The following macros are equivalent to the dev_{err, info, etc..} macros
 * except they use the qmmu or domain's friendly name instead of the difficult
 * to understand name produced by dev_name.
 */
#define qiommu_dev_printk_emit(lvl, qmmu, fmt, args...) \
({ \
	struct qiommu_device *_qmmu = (qmmu); \
	struct device *_dev = _qmmu->dev; \
	dev_printk_emit((lvl), _dev, "%s %s: " fmt, dev_driver_string(_dev), \
			_qmmu->name, ##args); \
})

#define qiommu_err(qmmu, fmt, args...) \
	qiommu_dev_printk_emit(LOGLEVEL_ERR, (qmmu), fmt, ##args)
#define qiommu_warn(qmmu, fmt, args...) \
	qiommu_dev_printk_emit(LOGLEVEL_WARNING, (qmmu), fmt, ##args)
#define qiommu_info(qmmu, fmt, args...) \
	qiommu_dev_printk_emit(LOGLEVEL_INFO, (qmmu), fmt, ##args)
#define qiommu_dbg(qmmu, fmt, args...) \
	qiommu_dev_printk_emit(LOGLEVEL_DEBUG, (qmmu), fmt, ##args)

#define qiommu_trc(qmmu, fmt, args...) \
({ \
	struct qiommu_device *_qmmu = (qmmu); \
	struct device *_dev = _qmmu->dev; \
	if (_qmmu->tracing_enabled) { \
		dev_printk_emit(LOGLEVEL_INFO, _dev, "%s %s: " fmt, \
				dev_driver_string(_dev), \
				_qmmu->name, ##args); \
	} \
})

#endif /* __QIOMMU_H__ */
