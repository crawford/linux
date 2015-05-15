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

#include <linux/acpi.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "io-pgtable.h"
#include "qiommu.h"

/*
 * This driver currently supports:
 *	- stage-1 translations with stage-2 bypass
 *	- v8l descriptor format
 *	- 4k granule with 39-bit VA(3-level) and 48-bit VA(4-level) supported
 *	- 64k granule with 42-bit VA(2-level) and 48-bit VA(3-level) supported
 *	- creating cacheable and non-cacheable mappings
 *	- translation and configuration fault reporting with GERROR
 *	- iova->phys using the HW GATOS interface
 *	- global bypass mode supported
 *	- TLB and configuration cache support
 *	- CMD queue support
 *	- Basic TLB and config cache support
 *	- 48 bit support with 4 level walks
 *	- translation and configuration fault reporting using the EVENT queue
 *	- QIOMMU reading page tables from the cache
 *	- QIOMMU reading config structures from the cache
 *	- Attaching devices belonging to different QIOMMUs to a single domain
 *
 * The following items are notably missing and will be added in the future:
 *	- stage-2 only mappings
 *	- Concatenated page support
 */

static struct iommu_ops qiommu_ops;

static atomic_t num_qiommu_devices = ATOMIC_INIT(0);
static atomic_t qiommu_domain_asids = ATOMIC_INIT(0);
static struct device *qiommu_domain_coherent_dev;
static struct device *qiommu_domain_noncoherent_dev;

static atomic_t num_identity_map_qiommus = ATOMIC_INIT(0);
struct io_pgtable_cfg identity_map_iopt_cfg;
struct io_pgtable_ops *identity_map_iopt_ops;
static phys_addr_t gits_translater_phys;
static phys_addr_t gicm_setspi_nsr_phys;

static unsigned int num_identity_map_qiommu_strs;
static char *identity_map_qiommu_strs[32];
module_param_array_named(identity_map_qiommus, identity_map_qiommu_strs,
			 charp, &num_identity_map_qiommu_strs, 0444);
MODULE_PARM_DESC(identity_map_qiommus,
		 "Names of qiommus whose masters should use identity map");

static char *identity_map_blocksize_str;
module_param_named(identity_map_blocksize, identity_map_blocksize_str,
			 charp, 0444);
MODULE_PARM_DESC(identity_map_blocksize,
		 "Block size to use when identity mapping System RAM");

static unsigned int num_reserved_qiommu_strs;
static char *reserved_qiommu_strs[32];
module_param_array_named(reserved_qiommus, reserved_qiommu_strs,
			 charp, &num_reserved_qiommu_strs, 0444);
MODULE_PARM_DESC(reserved_qiommus,
		 "Names of qiommus that should not be probed");

static unsigned int num_trace_qiommu_strs;
static char *trace_qiommu_strs[32];
module_param_array_named(trace_qiommus, trace_qiommu_strs,
			 charp, &num_trace_qiommu_strs, 0444);
MODULE_PARM_DESC(trace_qiommus,
		 "Names of qiommus for whom tracing output is desired");

static unsigned int debug_domain;
module_param(debug_domain, uint, 0644);
MODULE_PARM_DESC(debug_domain,
		 "enable verbose debugging for domain ops(default: 0)");

#define TRC_DOMAIN(...) \
	do { \
		if (unlikely(debug_domain)) \
			pr_info(__VA_ARGS__); \
	} while (0)

static bool
strcmp_qiommu_name(struct qiommu_device *qmmu, const char *match_str)
{
	size_t match_len = strlen(match_str);

	/* A '*' character at the end of match_str is treated as a wildcard */
	if (match_len && match_str[match_len-1] == '*')
		return strncmp(qmmu->name, match_str, match_len - 1);

	return strcmp(qmmu->name, match_str);
}

static const char *
match_qiommu_name(struct qiommu_device *qmmu,
		  char **match_strs, uint32_t num_match_strs)
{
	uint32_t idx;

	for (idx = 0; idx < num_match_strs; idx++)
		if (!strcmp_qiommu_name(qmmu, match_strs[idx]))
			return match_strs[idx];

	return NULL;
}

/*
 * For now just alloc global monotonically increasing ASIDs. It will take
 * quite a few removals/adds before we run out since 16-bit ASIDs are supported.
 */
static uint16_t alloc_asid_for_domain(void)
{
	int asid = atomic_inc_return(&qiommu_domain_asids);

	BUG_ON(asid >= 0x10000);
	return (uint16_t)asid;
}

static void qiommu_tlb_flush_all(void *cookie)
{
	struct qiommu_domain *qmmu_domain = cookie;

	pr_debug("-> %s: dom 0x%X\n", __func__, qmmu_domain->asid);

	/* tlb_flush_all already handled by detach_dev */
}

static void qiommu_tlb_add_flush(unsigned long iova, size_t size,
				 size_t granule, bool leaf, void *cookie)
{
	struct qiommu_domain *qmmu_domain = cookie;

	pr_debug("-> %s: dom 0x%X: iova 0x%08lX : size %d : leaf %d\n",
		 __func__, qmmu_domain->asid, iova, (int)size, leaf);

	/* tlb_add_flush already handled by qiommu_map/unmap */
}

static void qiommu_tlb_sync(void *cookie)
{
	struct qiommu_domain *qmmu_domain = cookie;

	pr_debug("-> %s: dom 0x%X\n", __func__, qmmu_domain->asid);

	/* tlb_sync already handled by qiommu_map/unmap */
}

struct iommu_gather_ops qiommu_gather_ops = {
	.tlb_flush_all	= qiommu_tlb_flush_all,
	.tlb_add_flush	= qiommu_tlb_add_flush,
	.tlb_sync	= qiommu_tlb_sync,
};

static int qiommu_group_notifier_fn(struct notifier_block *nb,
				    unsigned long action, void *_dev)
{
	struct device *dev = _dev;
	struct qiommu_master *master = get_dev_master(dev);
	struct qiommu_device *qmmu;

	if (WARN_ON(!master))
		return NOTIFY_DONE;

	qmmu = master->qmmu;
	if (action == IOMMU_GROUP_NOTIFY_ADD_DEVICE) {
		qiommu_info(qmmu, "%s: [ADD_DEVICE]\n", master->name);
	} else if (action == IOMMU_GROUP_NOTIFY_DEL_DEVICE) {
		qiommu_info(qmmu, "%s: [DEL_DEVICE]\n", master->name);
	} else if (action == IOMMU_GROUP_NOTIFY_BIND_DRIVER) {
		qiommu_info(qmmu, "%s: [BIND_DRIVER]\n", master->name);
	} else if (action == IOMMU_GROUP_NOTIFY_BOUND_DRIVER) {
		qiommu_info(qmmu, "%s: [BOUND_DRIVER] : %s\n", master->name,
			dev->driver->name);
	} else if (action == IOMMU_GROUP_NOTIFY_UNBIND_DRIVER) {
		qiommu_info(qmmu, "%s: [UNBIND_DRIVER] : %s\n", master->name,
			dev->driver->name);
	} else if (action == IOMMU_GROUP_NOTIFY_UNBOUND_DRIVER) {
		qiommu_info(qmmu, "%s: [UNBOUND_DRIVER]\n", master->name);
	} else {
		qiommu_info(qmmu, "%s: [UNKNOWN_ACTION] : %ld\n", master->name,
			action);
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

struct notifier_block qiommu_group_notifier = {
	.notifier_call = qiommu_group_notifier_fn
};

static void qiommu_setup_dma_ops(struct qiommu_master *master)
{
	struct device *dev = master->dev;
	uint64_t iova_region_start, iova_region_end, iova_region_size;

	/*
	 * Configure the master to use qiommu_ops for dma routines. IOVAs
	 * allocated for the master will come from the region starting at
	 * PAGE_SIZE and limited by num_iova_bits for the parent qiommu.
	 */
	iova_region_start = PAGE_SIZE; /* guard page */
	iova_region_end = 1ULL << master->qmmu->num_iova_bits;
	iova_region_size = iova_region_end - iova_region_start;
	arch_setup_dma_ops(dev, iova_region_start, iova_region_size,
			   &qiommu_ops, is_device_dma_coherent(dev));
}

static struct iommu_group *qiommu_device_group(struct device *dev)
{
	struct qiommu_master *master;
	struct qiommu_device *qmmu;
	struct iommu_group *group;
	int err = 0;

	/* Check the QIOMMU topology and allocate a master for dev */
	master = qiommu_top_alloc_master(dev);
	if (!master)
		return ERR_PTR(-ENODEV);
	set_dev_master(dev, master);
	qmmu = master->qmmu;

	/* Give up now if the QIOMMU is in global bypass mode */
	if (qmmu->use_global_bypass) {
		qiommu_info(qmmu, "not adding master %s because qiommu is in global bypass mode\n",
			    master->name);
		err = -ENODEV;
		goto cleanup_master;
	}

	group = generic_device_group(dev);
	if (!group) {
		qiommu_err(qmmu, "master %s: generic_device_group() failed!\n",
			   master->name);
		err = -ENOMEM;
		goto cleanup_master;
	}

	if (qmmu->tracing_enabled) {
		err = iommu_group_register_notifier(group,
						    &qiommu_group_notifier);
		if (err) {
			qiommu_err(qmmu, "master %s: iommu_group_register_notifier() err=%d\n",
				master->name, err);
			goto cleanup_group;
		}
	}
	return group;

cleanup_group:
	iommu_group_put(group);

cleanup_master:
	set_dev_master(dev, NULL);
	qiommu_top_free_master(master);

	return ERR_PTR(err);
}

static int qiommu_add_device(struct device *dev)
{
	struct qiommu_master *master;
	struct qiommu_device *qmmu;
	struct iommu_group *group;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	master = get_dev_master(dev);
	qmmu = master->qmmu;

	/* Check if the master can use identity mapping */
	master->use_identity_mapping =
		qmmu->identity_map_enable &&
		identity_map_iopt_ops &&
		is_device_dma_coherent(dev) &&
		is_device_dma_coherent(qmmu->dev);

	if (master->use_identity_mapping && iommu_request_dm_for_dev(dev)) {
		qiommu_err(qmmu, "Failed to enable identity mapping for %s\n",
			   master->name);
		master->use_identity_mapping = false;
	}

	/* If master is using identity mapping, keep non-iommu dma_ops */
	if (!master->use_identity_mapping)
		qiommu_setup_dma_ops(master);

	iommu_group_put(group);

	return 0;
}

static void qiommu_remove_device(struct device *dev)
{
	struct qiommu_master *master = get_dev_master(dev);

	TRC_DOMAIN("-> %s(%s)\n", __func__, dev_name(dev));

	/* Handle (unexpected) attachment to non-default domains */
	if (master->dom && master->dom != master->default_dom) {
		qiommu_err(master->qmmu, "Master [%s] should have been removed from non-default domain [0x%X], prior to removal!\n",
			   master->name, master->dom->asid);
		iommu_detach_device(&master->dom->iommu_dom, dev);
		WARN_ON(1);
	}

	/* Take master off the default domain */
	if (master->dom) {
		qiommu_info(master->qmmu, "Detaching master [%s] from domain [0x%X] prior to removal\n",
			    master->name, master->dom->asid);
		qiommu_ops.detach_dev(&master->dom->iommu_dom, dev);
	}

	/* Device removal should cleanup the default domain */
	iommu_group_remove_device(dev);
}

static struct iommu_domain *qiommu_domain_alloc(unsigned iommu_domain_type)
{
	struct qiommu_domain *qmmu_domain;
	struct iommu_domain_geometry *geo;
	struct io_pgtable_cfg *cfg;
	struct io_pgtable_ops *ops;
	bool domain_is_identity = (iommu_domain_type == IOMMU_DOMAIN_IDENTITY);

	TRC_DOMAIN("-> %s(type=0x%X)\n", __func__, iommu_domain_type);

	/* Check pre-requisites for identity map domains */
	if (domain_is_identity && !identity_map_iopt_ops) {
		pr_err("%s(): missing page tables for identity map domains\n",
		       __func__);
		WARN_ON(1);
		return NULL;
	}

	qmmu_domain = kzalloc(sizeof(*qmmu_domain), GFP_KERNEL);
	if (!qmmu_domain)
		return NULL;
	qmmu_domain->init_done = false;

	/* Initialize qiommu_domain protection mechanisms */
	spin_lock_init(&qmmu_domain->iopt_and_list_lock);
	mutex_init(&qmmu_domain->mtx);

	/* Setup the geometry limits for the iommu_domain */
	geo = &qmmu_domain->iommu_dom.geometry;
	geo->aperture_start = PAGE_SIZE;
	geo->aperture_end = DMA_BIT_MASK(CONFIG_ARM64_VA_BITS);
	geo->force_aperture = true;

	/* Initialize the list of qiommu devices */
	INIT_LIST_HEAD(&qmmu_domain->qmmu_list);
	qmmu_domain->qmmu_list_length = 0;

	/*
	 * Prior to any devices being attached to the domain, dma_dev will
	 * only be used for flushing the page tables when setting up the
	 * domain's io_pgtable members. This can safely be done using the
	 * noncoherent dma_dev even if the domain ends up using coherent
	 * qiommus.
	 */
	qmmu_domain->dma_dev = qiommu_domain_noncoherent_dev;
	qmmu_domain->qiommus_coherent = false;
	qmmu_domain->masters_coherent = false;

	/* Initialize the domain page tables and operators */
	qmmu_domain->tlb_dirty = false;
	qmmu_domain->asid = domain_is_identity ? 0 : alloc_asid_for_domain();

	if (domain_is_identity) {
		qmmu_domain->iopt_cfg = identity_map_iopt_cfg;
		qmmu_domain->iopt_ops = identity_map_iopt_ops;
	} else {
		cfg = &qmmu_domain->iopt_cfg;
		cfg->quirks = 0;
		cfg->ias = CONFIG_ARM64_VA_BITS;
		cfg->oas = 48;
		cfg->pgsize_bitmap = platform_bus_type.iommu_ops->pgsize_bitmap;
		cfg->tlb = &qiommu_gather_ops;
		cfg->iommu_dev = qmmu_domain->dma_dev;

		ops = alloc_io_pgtable_ops(ARM_64_LPAE_S1, cfg, qmmu_domain);
		if (!ops) {
			pr_err("%s(): alloc_io_pgtable_ops failed for domain 0x%X!\n",
			__func__, qmmu_domain->asid);
			kfree(qmmu_domain);
			return NULL;
		}
		qmmu_domain->iopt_ops = ops;
	}

	/* Setup the iova allocator for DMA API domains */
	if (iommu_domain_type & IOMMU_DOMAIN_DMA)
		iommu_get_dma_cookie(&qmmu_domain->iommu_dom);

	return &qmmu_domain->iommu_dom;
}

static void qiommu_domain_free(struct iommu_domain *domain)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	unsigned long flags;

	TRC_DOMAIN("-> %s(domain 0x%X)\n", __func__, qmmu_domain->asid);

	/* Make sure no qiommus are still using the domain */
	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);
	if (qmmu_domain->qmmu_list_length) {
		pr_err("%s() : cannot free domain 0x%X because it is still being used by %d qiommus!\n",
		       __func__, qmmu_domain->asid,
		       qmmu_domain->qmmu_list_length);
		BUG();
	}
	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);

	qiommu_hw_free_domain_ctx(qmmu_domain);

	/* Cleanup the page tables unless they contain the identity mappings */
	if (qmmu_domain->iopt_ops != identity_map_iopt_ops)
		free_io_pgtable_ops(qmmu_domain->iopt_ops);

	/* Cleanup the iova allocator for DMA API domains */
	if (qmmu_domain->iommu_dom.type & IOMMU_DOMAIN_DMA)
		iommu_put_dma_cookie(&qmmu_domain->iommu_dom);

	kfree(qmmu_domain);
}

static int reserve_page_in_iovad(struct iova_domain *iovad, dma_addr_t addr)
{
	unsigned long pfn = iova_pfn(iovad, addr);

	return !reserve_iova(iovad, pfn, pfn);
}

static int create_static_mapping(struct qiommu_master *master,
				 dma_addr_t iova, phys_addr_t pa, int prot)
{
	struct qiommu_domain *qmmu_domain = master->dom;
	struct iommu_domain *domain = &qmmu_domain->iommu_dom;
	struct iova_domain *iovad = domain->iova_cookie;
	phys_addr_t cur_phys;

	/*
	 * Make sure the address is not already mapped to a different page. If
	 * it was, we BUG() as it indicates a mistake was made in the DT/ACPI
	 * config which can cause subtle yet serious issues.
	 */
	cur_phys = iommu_iova_to_phys(domain, iova);
	if (cur_phys == pa)
		return 0;
	else if (cur_phys) {
		qiommu_err(master->qmmu, "%s(%s, 0x%llX, 0x%llX) : ERROR : existing mapping, 0x%llX, exists for iova in domain 0x%X!\n",
			   __func__, master->name, iova, pa, cur_phys,
			   qmmu_domain->asid);
		BUG();
		return -EEXIST;
	} else if (domain->type == IOMMU_DOMAIN_IDENTITY) {
		qiommu_err(master->qmmu, "%s(%s, 0x%llX, 0x%llX) : ERROR : cannot add mappings to IDENTITY domains!\n",
			   __func__, master->name, iova, pa);
		WARN_ON(1);
		return -EINVAL;
	}

	/*
	 * The input iova needs to be reserved to prevent it from being
	 * allocated and mapped at runtime causing a conflict. Unfortunately,
	 * the address can only be reserved in IOMMU_DOMAIN_DMA domains as
	 * we cannot be sure the iova_cookie pointer in other types of domains
	 * is an iova_domain.
	 */
	if (domain->type != IOMMU_DOMAIN_DMA ||
	    reserve_page_in_iovad(iovad, iova))
		qiommu_warn(master->qmmu, "creating untracked mapping (0x%llX -> 0x%llX) in domain 0x%X\n",
			    iova, pa, qmmu_domain->asid);

	return iommu_map(domain, iova, pa, PAGE_SIZE, prot);
}

static int add_qiommu_to_domain(struct qiommu_domain *qmmu_domain,
				struct qiommu_device *qmmu)
{
	struct qiommu_list_node *node;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);

	/* Don't create duplicate entries for qmmus already in the list */
	list_for_each_entry(node, &qmmu_domain->qmmu_list, list) {
		if (node->qmmu == qmmu) {
			node->dups++;
			goto done;
		}
	}

	/*
	 * If we are here, a new list node is needed for the qmmu. Allocate
	 * using GFP_NOWAIT since this is being done while holding a spinlock.
	 */
	node = kzalloc(sizeof(*node), GFP_NOWAIT);
	if (!node) {
		ret = -ENOMEM;
		goto done;
	}

	INIT_LIST_HEAD(&node->list);
	node->qmmu = qmmu;
	node->dups = 0;

	list_add_tail(&node->list, &qmmu_domain->qmmu_list);
	qmmu_domain->qmmu_list_length++;

done:
	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);
	return ret;
}

static int qiommu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	struct qiommu_device *qmmu;
	struct qiommu_master *master;
	int i, err = 0;
	bool swap_needed, swap_permissible;

	TRC_DOMAIN("-> %s(%s)\n", __func__, dev_name(dev));

	master = get_dev_master(dev);
	if (!master) {
		pr_err("%s(%s): cannot attach device without a master!\n",
		       __func__, dev_name(dev));
		return -ENODEV;
	}
	qmmu = master->qmmu;

	mutex_lock(&qmmu_domain->mtx);

	/*
	 * Make sure that the domain is appropriate for this master:
	 * - The master's coherency must match that of all other masters in the
	 *   domain.
	 * - The coherency of the master's qiommu must match that of the qiommus
	 *   already in the domain.
	 */
	if (qmmu_domain->init_done) {
		bool master_coherent = is_device_dma_coherent(master->dev);
		bool qiommu_coherent = is_device_dma_coherent(qmmu->dev);

		if (qmmu_domain->masters_coherent != master_coherent) {
			qiommu_err(qmmu, "cannot attach %s because domain 0x%X only supports %scoherent masters",
				   master->name, qmmu_domain->asid,
				   (qmmu_domain->masters_coherent?"":"non-"));
			err = -EINVAL;
		}
		if (qmmu_domain->qiommus_coherent != qiommu_coherent) {
			qiommu_err(qmmu, "cannot attach %s because domain 0x%X only supports %scoherent qiommus",
				   master->name, qmmu_domain->asid,
				   (qmmu_domain->qiommus_coherent?"":"non-"));
			err = -EINVAL;
		}
		if (err) {
			mutex_unlock(&qmmu_domain->mtx);
			return err;
		}
	}

	/* Domain swaps can only be to or from the master's default domain */
	swap_needed = (master->dom != NULL);
	swap_permissible = (master->dom == master->default_dom) ||
			   (domain == &master->default_dom->iommu_dom);

	if (swap_needed && !swap_permissible) {
		qiommu_err(qmmu, "master %s must be detached from domain 0x%X before attaching to domain 0x%X\n",
			   master->name, master->dom->asid, qmmu_domain->asid);
		BUG();
	} else if (swap_needed) {
		qiommu_ops.detach_dev(&master->dom->iommu_dom, dev);
	}

	/* Finish initializing the domain */
	if (!qmmu_domain->init_done) {
		qmmu_domain->masters_coherent = is_device_dma_coherent(dev);
		qmmu_domain->qiommus_coherent =
			is_device_dma_coherent(qmmu->dev);

		if (qmmu_domain->qiommus_coherent) {
			qmmu_domain->dma_dev = qiommu_domain_coherent_dev;
			qmmu_domain->iopt_cfg.iommu_dev = qmmu_domain->dma_dev;
		}

		err = qiommu_hw_cfg_domain_ctx(qmmu_domain);
		if (err) {
			mutex_unlock(&qmmu_domain->mtx);
			return err;
		}
		qmmu_domain->init_done = true;
	}

	/* Add master to the domain and setup master's stes */
	master->dom = qmmu_domain;
	add_qiommu_to_domain(qmmu_domain, qmmu);
	qiommu_hw_cfg_master_stes(master);

	mutex_unlock(&qmmu_domain->mtx);

	/*
	 * The master's default domain will initially be set to the first
	 * DOMAIN_DMA to which it is attached. If the master is later attached
	 * to a DOMAIN_IDENTITY, that will become the master's default domain.
	 */
	if ((domain->type == IOMMU_DOMAIN_DMA && !master->default_dom) ||
	    (domain->type == IOMMU_DOMAIN_IDENTITY))
		master->default_dom = qmmu_domain;

	/* Create static mappings */
	for (i = 0; i < qmmu->num_static_mappings; i++) {
		err = create_static_mapping(master,
					    qmmu->static_mappings[2*i],
					    qmmu->static_mappings[2*i+1],
					    IOMMU_WRITE | IOMMU_READ);
		if (err) {
			qiommu_err(qmmu, "failed to attach %s to domain 0x%X due to create_static_mapping() error %d\n",
				   master->name, qmmu_domain->asid, err);
			return err;
		}
	}

	qiommu_info(qmmu, "attached %scoherent master %s to domain 0x%X %s\n",
		    (is_device_dma_coherent(dev)?"":"non-"), master->name,
		    qmmu_domain->asid, (qmmu_domain->asid ? "" : "[IDENTITY]"));

	return err;
}

static int remove_qiommu_from_domain(struct qiommu_domain *qmmu_domain,
				     struct qiommu_device *qmmu)
{
	struct qiommu_list_node *node, *found = NULL;
	unsigned long flags;

	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);

	/* Find the qmmu in the list */
	list_for_each_entry(node, &qmmu_domain->qmmu_list, list) {
		if (node->qmmu == qmmu) {
			found = node;
			break;
		}
	}

	if (!found)
		goto done;

	/* If there are no duplicates, cleanup the node and remove it */
	if (node->dups == 0) {
		qmmu_domain->qmmu_list_length--;
		list_del(&node->list);
		kfree(node);
	} else
		node->dups--;

done:
	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);
	return  found ? 0x0 : -ENODEV;
}

static void qiommu_detach_dev(struct iommu_domain *domain,
			      struct device *dev)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	struct qiommu_master *master = get_dev_master(dev);

	TRC_DOMAIN("-> %s(%s)\n", __func__, dev_name(dev));

	/* Cleanup the master's stes and remove it from the domain */
	qiommu_hw_free_master_stes(master);
	remove_qiommu_from_domain(qmmu_domain, master->qmmu);
	master->dom = NULL;

	qiommu_info(master->qmmu, "detached master %s from domain 0x%X\n",
		    master->name, qmmu_domain->asid);
}

static int qiommu_map(struct iommu_domain *domain, unsigned long iova,
		      phys_addr_t paddr, size_t size, int prot)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	struct qiommu_list_node *node;
	struct io_pgtable_ops *ops = qmmu_domain->iopt_ops;
	unsigned long flags;
	int ret;

	pr_debug("-> %s: dom 0x%X: 0x%08lX ++ %zu -> %pa : cpu_va 0x%p : prot 0x%X\n",
		 __func__, qmmu_domain->asid,
		 iova, size, &paddr, __va(paddr), prot);

	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);

	/* If delayed_hw_sync is enabled, the TLB may be dirty */
	if (qmmu_domain->tlb_dirty) {
		list_for_each_entry(node, &qmmu_domain->qmmu_list, list)
			qiommu_hw_sync(node->qmmu, false);
		qmmu_domain->tlb_dirty = false;
	}

	ret = ops->map(ops, iova, paddr, size, prot);

	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);

	return ret;
}

static size_t qiommu_unmap(struct iommu_domain *domain,
			   unsigned long iova, size_t size)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	struct qiommu_list_node *node;
	struct io_pgtable_ops *ops = qmmu_domain->iopt_ops;
	unsigned long flags;
	size_t ret;

	pr_debug("-> %s: dom 0x%X: 0x%08lX ++ %zu\n", __func__,
		 qmmu_domain->asid, iova, size);

	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);

	ret = ops->unmap(ops, iova, size);
	list_for_each_entry(node, &qmmu_domain->qmmu_list, list) {
		struct qiommu_device *qmmu = node->qmmu;

		if (!qmmu->tlb_disable) {
			qiommu_hw_tlbi_va(qmmu, qmmu_domain->asid, iova);
			if (!qmmu->enable_delayed_hw_sync_after_tlbi)
				qiommu_hw_sync(qmmu, false);
			else
				qmmu_domain->tlb_dirty = true;
		}
	}

	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);

	return ret;
}

static phys_addr_t qiommu_iova_to_phys(struct iommu_domain *domain,
				       dma_addr_t iova)
{
	struct qiommu_domain *qmmu_domain = to_qiommu_domain(domain);
	struct io_pgtable_ops *ops = qmmu_domain->iopt_ops;
	phys_addr_t phys;
	unsigned long flags;

	pr_debug("-> %s: dom 0x%X: 0x%llX\n", __func__,
		 qmmu_domain->asid, iova);

	spin_lock_irqsave(&qmmu_domain->iopt_and_list_lock, flags);
	phys = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&qmmu_domain->iopt_and_list_lock, flags);

	return phys;
}

static bool qiommu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	case IOMMU_CAP_INTR_REMAP:
		return true;
	default:
		return false;
	}
}

static struct iommu_ops qiommu_ops = {
	.capable        = qiommu_capable,
	.device_group   = qiommu_device_group,
	.add_device     = qiommu_add_device,
	.remove_device  = qiommu_remove_device,
	.domain_alloc   = qiommu_domain_alloc,
	.domain_free    = qiommu_domain_free,
	.attach_dev     = qiommu_attach_dev,
	.detach_dev     = qiommu_detach_dev,
	.map            = qiommu_map,
	.unmap          = qiommu_unmap,
	.map_sg         = default_iommu_map_sg,
	.iova_to_phys   = qiommu_iova_to_phys,
#if PAGE_SIZE == SZ_4K
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G,
#elif PAGE_SIZE == SZ_16K
	.pgsize_bitmap = SZ_16K | SZ_32M,
#elif PAGE_SIZE == SZ_64K
	.pgsize_bitmap = SZ_64K | SZ_512M,
#endif
};

static irqreturn_t qiommu_event_irq(int irq, void *dev)
{
	struct qiommu_device *qmmu = dev;

	qiommu_hw_handle_event_interrupt(qmmu->hw);
	return IRQ_HANDLED;
}

static irqreturn_t qiommu_rst_inform_irq(int irq, void *dev)
{
	struct qiommu_device *qmmu = dev;

	qiommu_hw_handle_rst_inform_interrupt(qmmu->hw);
	return IRQ_HANDLED;
}

static void get_qiommu_device_config(struct qiommu_device *qmmu)
{
	uint32_t tmp;
	int num, err;
	uint64_t *u64_arr;
	uint32_t *u32_arr;
	const char *tmp_str;

	/* Get the pretty name for the QIOMMU if one was provided */
	tmp = device_property_read_string(qmmu->dev, "qiommu-name",
					  &qmmu->name);
	if (tmp)
		qmmu->name = dev_name(qmmu->dev);
	else
		dev_info(qmmu->dev, "renaming to %s\n", qmmu->name);

	/* Check if this qiommu instance should be reserved */
	tmp_str = match_qiommu_name(qmmu, reserved_qiommu_strs,
				    num_reserved_qiommu_strs);
	qmmu->is_reserved = (tmp_str != NULL);
	if (qmmu->is_reserved) {
		qiommu_info(qmmu, "QIOMMU is reserved - matched \"%s\" in qiommu.reserved_qiommus[]\n",
			    tmp_str);
		return;
	}

	/* Check if this qiommu should display tracing messages */
	tmp_str = match_qiommu_name(qmmu, trace_qiommu_strs,
				    num_trace_qiommu_strs);
	qmmu->tracing_enabled = (tmp_str != NULL);
	if (qmmu->tracing_enabled) {
		qiommu_info(qmmu, "tracing enabled - matched \"%s\" in qiommu.trace_qiommus[]\n",
			    tmp_str);
	}

	/* Get global bypass configuration if specified */
	qmmu->use_global_bypass = device_property_present(qmmu->dev,
							  "global-bypass-attributes");
	if (qmmu->use_global_bypass)
		if (device_property_read_u32(qmmu->dev,
					     "global-bypass-attributes",
					     &qmmu->global_bypass_attributes))
			qmmu->global_bypass_attributes = 0;

	/* Check for tlb-disable. If not set or 0, TLB is enabled */
	if (device_property_read_u32(qmmu->dev, "tlb-disable", &tmp))
		qmmu->tlb_disable = false;
	else
		qmmu->tlb_disable = !!tmp;

	/* Check config-cache-disable. If not set or 0, config cache enabled */
	if (device_property_read_u32(qmmu->dev, "config-cache-disable", &tmp))
		qmmu->config_cache_disable = false;
	else
		qmmu->config_cache_disable = !!tmp;

	/* If set, hw_sync and flush may be delayed until next map call */
	if (device_property_read_u32(qmmu->dev,
				     "enable-delayed-hw-sync-after-tlbi", &tmp))
		qmmu->enable_delayed_hw_sync_after_tlbi = false;
	else
		qmmu->enable_delayed_hw_sync_after_tlbi = !!tmp;

	/* Get static-mappings to be created on all of the qiommu's domains */
	if (!device_property_present(qmmu->dev, "static-mappings"))
		goto skip_static_mappings;

	num = device_property_read_u64_array(qmmu->dev, "static-mappings",
					     NULL, 0);
	if (num <= 0 || num & 0x1) {
		qiommu_err(qmmu, "invalid array length %d for static-mappings property\n",
			   num);
		BUG();
	}

	u64_arr = devm_kzalloc(qmmu->dev, num * sizeof(*u64_arr), GFP_KERNEL);
	BUG_ON(!u64_arr);

	err = device_property_read_u64_array(qmmu->dev, "static-mappings",
					     u64_arr, num);
	if (err < 0) {
		qiommu_err(qmmu, "failed to read_u64_array(static-mappings) [err=%d]\n",
			   err);
		BUG();
	}

	qmmu->num_static_mappings = num / 2;
	qmmu->static_mappings = u64_arr;
skip_static_mappings:

	/* Get any special configuration required for the impdef regs */
	if (!device_property_present(qmmu->dev, "impdef-reg-settings"))
		goto skip_impdef_reg_settings;

	num = device_property_read_u32_array(qmmu->dev, "impdef-reg-settings",
					     NULL, 0);
	if (num <= 0 || num & 0x1) {
		qiommu_err(qmmu, "invalid array length %d for impdef-reg-settings property\n",
			   num);
		BUG();
	}

	u32_arr = devm_kzalloc(qmmu->dev, num * sizeof(*u32_arr), GFP_KERNEL);
	BUG_ON(!u32_arr);

	err = device_property_read_u32_array(qmmu->dev, "impdef-reg-settings",
					     u32_arr, num);
	if (err < 0) {
		qiommu_err(qmmu, "failed to read_u32_array(impdef-reg-settings) [err=%d]\n",
			   err);
		BUG();
	}

	qmmu->num_impdef_reg_settings = num / 2;
	qmmu->impdef_reg_settings = u32_arr;
skip_impdef_reg_settings:

	/* Check the number of bits supported for iovas */
	if (device_property_read_u32(qmmu->dev, "num-iova-bits", &tmp) < 0) {
		qmmu->num_iova_bits = 32;
	} else if (tmp > CONFIG_ARM64_VA_BITS || tmp < 32) {
		qmmu->num_iova_bits = tmp < 32 ? 32 : CONFIG_ARM64_VA_BITS;
		qiommu_warn(qmmu, "setting num-iova-bits to %u because requested value, %u, is outside valid range, 32 -- %u (ARM64_VA_BITS)\n",
			    qmmu->num_iova_bits, tmp, CONFIG_ARM64_VA_BITS);
	} else {
		qmmu->num_iova_bits = tmp;
	}

	/* If set, DDR will be 1-to-1 mapped for masters */
	err = device_property_read_u32(qmmu->dev,
				       "identity-map-enable", &tmp);
	qmmu->identity_map_enable = (err == 0) && (tmp != 0);

	/* identity-map-enable may also be specified as a module param */
	tmp_str = match_qiommu_name(qmmu, identity_map_qiommu_strs,
				    num_identity_map_qiommu_strs);

	if (tmp_str) {
		qmmu->identity_map_enable = true;
		qiommu_info(qmmu, "Identity mapping enabled - matched \"%s\" in qiommu.identity_map_qiommus[]\n",
			    tmp_str);
	} else if (qmmu->identity_map_enable) {
		qiommu_info(qmmu, "Identity mapping enabled\n");
	}

	/* Identity mapping is currently only supported for coherent QIOMMUs */
	if (qmmu->identity_map_enable && !is_device_dma_coherent(qmmu->dev)) {
		qiommu_warn(qmmu, "although identity-map-enable was set, cannot use identity mapping because qiommu is non-coherent!\n");
		qmmu->identity_map_enable = false;
	}

	if (qmmu->identity_map_enable)
		atomic_inc(&num_identity_map_qiommus);

	/* Print any special configuration worth noting */
	if (qmmu->use_global_bypass)
		qiommu_info(qmmu, "Globally Bypassing QIOMMU [0x%08X]\n",
			    qmmu->global_bypass_attributes);
	if (qmmu->tlb_disable)
		qiommu_info(qmmu, "TLB Disabled\n");
	if (qmmu->config_cache_disable)
		qiommu_info(qmmu, "Config Cache Disabled\n");
	if (qmmu->enable_delayed_hw_sync_after_tlbi)
		qiommu_info(qmmu, "Delayed HW Sync after TLBI Enabled\n");
	if (qmmu->num_static_mappings) {
		qiommu_info(qmmu, "%d static mappings requested",
			    qmmu->num_static_mappings);
		u64_arr = qmmu->static_mappings;
		for (tmp = 0; tmp < qmmu->num_static_mappings; tmp++)
			qiommu_dbg(qmmu, "mapping %u : 0x%llX -> 0x%llX\n",
				   tmp, u64_arr[2*tmp], u64_arr[2*tmp+1]);
	}

	if (qmmu->num_impdef_reg_settings) {
		qiommu_info(qmmu, "%d impdef reg settings requested\n",
			    qmmu->num_impdef_reg_settings);
		u32_arr = qmmu->impdef_reg_settings;
		for (tmp = 0; tmp < qmmu->num_impdef_reg_settings; tmp++)
			qiommu_dbg(qmmu, "impdef setting %u : *(base + 0x%08X) = 0x%08X\n",
				   tmp, u32_arr[2*tmp], u32_arr[2*tmp+1]);
	}
}

static int qiommu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct qiommu_device *qmmu;
	struct device *dev = &pdev->dev;
	int err = 0;
	int num_irqs = 0, irq;
	void __iomem *base = NULL;

	dev->coherent_dma_mask = DMA_BIT_MASK(sizeof(dma_addr_t) * 8);
	dev->dma_mask = &dev->coherent_dma_mask;

	qmmu = devm_kzalloc(dev, sizeof(*qmmu), GFP_KERNEL);
	if (!qmmu)
		return -ENOMEM;

	qmmu->dev = dev;
	INIT_LIST_HEAD(&qmmu->list);

	/* Retrieve any special QIOMMU config specified in DT/ACPI */
	get_qiommu_device_config(qmmu);

	if (qmmu->is_reserved) {
		qiommu_warn(qmmu, "Discontinuing probe of reserved QIOMMU\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		qiommu_err(qmmu, "devm_ioremap_resource failed : err %d\n",
			   err);
		goto probe_fail;
	}

	while (0 < (irq = platform_get_irq(pdev, num_irqs))) {
		irqreturn_t (*irq_fn)(int, void *) = qiommu_event_irq;
		const char *irq_type_str = "event";
		char *irq_name;

		if (num_irqs == 1) {
			irq_fn = qiommu_rst_inform_irq;
			irq_type_str = "inform-rst";
		} else if (num_irqs > 1) {
			qiommu_err(qmmu, "too many interrupts requested! Only event and inform-rst supported\n");
			goto probe_fail;
		}

		irq_name = kasprintf(GFP_KERNEL, "%s-%s-irq", qmmu->name,
				     irq_type_str);
		err = devm_request_irq(dev, irq, irq_fn,
				       IRQF_SHARED | IRQF_TRIGGER_RISING,
				       irq_name, qmmu);
		if (err < 0) {
			qiommu_err(qmmu, "request_irq(%d, %s) : err %d\n", irq,
				   irq_name, err);
			goto probe_fail;
		}
		num_irqs++;
	}

	if (!num_irqs)
		qiommu_warn(qmmu, "no interrupts configured!!\n");

	err = qiommu_hw_init(qmmu, base);
	if (err) {
		qiommu_err(qmmu, "hw_init failed : err %d\n", err);
		goto probe_fail;
	}

	/* Add the toplogy for the current QIOMMU */
	qiommu_top_add_qiommu(qmmu);

	atomic_inc(&num_qiommu_devices);

#ifdef CONFIG_ACPI
	/* Enable masters with an ACPI-enforced dependency on this qiommu */
	if (!acpi_disabled)
		acpi_walk_dep_device_list(ACPI_HANDLE(qmmu->dev));
#endif

	return 0;

probe_fail:
	pr_err("%s: %s: probe failed : [err=%d]\n", __func__, qmmu->name, err);
	return err;
}

static const struct of_device_id qiommu_of_match[] = {
	{ .compatible = "qcom,qiommu", },
	{}
};

static const struct acpi_device_id qiommu_acpi_ids[] = {
	{ "QCOM80A0" },
	{ },
};

static struct platform_driver qiommu_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "qiommu",
		.of_match_table = of_match_ptr(qiommu_of_match),
		.acpi_match_table = qiommu_acpi_ids,
	},
	.probe  = qiommu_probe,
};

static struct device *alloc_qiommu_domain_dma_dev(bool coherent)
{
	struct device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	device_initialize(dev);
	arch_setup_dma_ops(dev, 0, 0, NULL, coherent);
	dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));

	return dev;
}

static int get_system_ram_region(uint64_t start, uint64_t end, void *data)
{
	uint64_t *region = data;

	region[0] = min(start, region[0]);
	region[1] = max(end, region[1]);

	return 0;
}

#if defined(CONFIG_ACPI)
static int __init get_gits_translater_phys(struct acpi_subtable_header *header,
					   const unsigned long end)
{
	struct acpi_madt_generic_translator *its =
		(struct acpi_madt_generic_translator *)header;

	if (!BAD_MADT_ENTRY(its, end) && its->base_address)
		gits_translater_phys = its->base_address + 0x10040;

	return 0;
}

static int __init get_gicm_setspi_nsr_phys(struct acpi_subtable_header *header,
					   const unsigned long end)
{
	struct acpi_madt_generic_msi_frame *msi =
		(struct acpi_madt_generic_msi_frame *)header;

	if (!BAD_MADT_ENTRY(msi, end) && msi->base_address)
		gicm_setspi_nsr_phys = msi->base_address + 0x40;

	return 0;
}
#endif /* CONFIG_ACPI */

static void __init dt_parse_msi_addrs(void)
{
	struct device_node *np;
	const struct of_device_id gic_msi_device_ids[] = {
		{.compatible = "arm,gic-v3-its",}, {},
		{.compatible = "arm,gic-v2m-frame",}, {}
	};

	/* Look for gits_translater_phys */
	np = of_find_matching_node(NULL, gic_msi_device_ids);
	if (np &&
	    of_property_read_bool(np, "msi-controller") &&
	    !of_property_read_u64(np, "reg", &gits_translater_phys))
		gits_translater_phys += 0x10040;

	/* Look for gicm_setspi_nsr_phys */
	np = of_find_matching_node(NULL, gic_msi_device_ids + 2);
	if (np &&
	    of_property_read_bool(np, "msi-controller") &&
	    !of_property_read_u64(np, "reg", &gicm_setspi_nsr_phys))
		gicm_setspi_nsr_phys += 0x40;
}

static void __init setup_identity_map_blocksize(void)
{
	struct io_pgtable_cfg *cfg = &identity_map_iopt_cfg;
	uint64_t blocksize;

	/* Default to 4K granule with 1G blocks */
	cfg->pgsize_bitmap = SZ_1G | SZ_4K;
	if (!identity_map_blocksize_str)
		return;

	blocksize = memparse(identity_map_blocksize_str, NULL);

	/* Validate blocksize and set the translation granule */
	if (blocksize == SZ_1G || blocksize == SZ_2M || blocksize == SZ_4K)
		cfg->pgsize_bitmap = blocksize | SZ_4K;
	else if (blocksize == SZ_32M || blocksize == SZ_16K)
		cfg->pgsize_bitmap = blocksize | SZ_16K;
	else if (blocksize == SZ_512M || blocksize == SZ_64K)
		cfg->pgsize_bitmap = blocksize | SZ_64K;
	else
		pr_info("%s() : Invalid blocksize, %s, requested. Default value will be used.\n",
			__func__, identity_map_blocksize_str);
}

static int __init qiommu_identity_map_init(void)
{
	struct io_pgtable_cfg *cfg = &identity_map_iopt_cfg;
	struct io_pgtable_ops *ops;
	uint64_t system_ram_region[2] = {~0ULL, 0};
	phys_addr_t max_phys;
	uint64_t blocksize, blockmask, addr;
	int prot;
	int ret = 0;

	/* Get System RAM region */
	walk_system_ram_res(0x0, ~0ULL, &system_ram_region,
			    get_system_ram_region);

	/* Get MSI addrs */
	if (!acpi_disabled) {
#if defined(CONFIG_ACPI)
		acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_TRANSLATOR,
				      get_gits_translater_phys, 0);
		acpi_table_parse_madt(ACPI_MADT_TYPE_GENERIC_MSI_FRAME,
				      get_gicm_setspi_nsr_phys, 0);
#endif /* CONFIG_ACPI */
	} else {
		dt_parse_msi_addrs();
	}

	/* Figure out the max phys addr that will have to be mapped */
	max_phys = system_ram_region[1];
	if (max_phys < gits_translater_phys)
		max_phys = gits_translater_phys;
	if (max_phys < gicm_setspi_nsr_phys)
		max_phys = gicm_setspi_nsr_phys;

	/* Setup the identity mapping ops */
	cfg->quirks = 0;
	cfg->ias = __fls(max_phys) + 1;
	cfg->oas = 48;
	cfg->tlb = &qiommu_gather_ops;
	cfg->iommu_dev = qiommu_domain_coherent_dev;
	setup_identity_map_blocksize();

	ops = alloc_io_pgtable_ops(ARM_64_LPAE_S1, cfg, NULL);
	if (!ops) {
		pr_err("%s(): alloc_io_pgtable_ops failed!\n", __func__);
		return -EINVAL;
	}

	/* Map System RAM using the largest configured blocksize */
	blocksize = 1 << __fls(cfg->pgsize_bitmap);
	blockmask = ~(blocksize-1);
	prot = IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE;
	addr = system_ram_region[0] & blockmask;
	while (addr <= system_ram_region[1]) {
		ret = ops->map(ops, addr, addr, blocksize, prot);
		if (ret) {
			pr_err("%s() : map failed for address 0x%llX [ret=%d], disabling identity mapping!",
			       __func__, addr, ret);
			goto fail_cleanup;
		}
		addr += blocksize;
	}

	/* Map MSI addresses using a page sized mapping */
	blocksize = 1 << __ffs(cfg->pgsize_bitmap);
	blockmask = ~(blocksize-1);
	prot = IOMMU_READ | IOMMU_WRITE;

	if (gicm_setspi_nsr_phys) {
		addr = gicm_setspi_nsr_phys & blockmask;
		ret = ops->map(ops, addr, addr, blocksize, prot);
		if (ret) {
			pr_err("%s() : map failed for gicm_setspi_nsr 0x%llX [ret=%d], disabling identity mapping!",
			       __func__, addr, ret);
			goto fail_cleanup;
		}
	}

	if (gits_translater_phys) {
		addr = gits_translater_phys & blockmask;
		ret = ops->map(ops, addr, addr, blocksize, prot);
		if (ret) {
			pr_err("%s() : map failed for gits_translater 0x%llX [ret=%d], disabling identity mapping!",
			       __func__, addr, ret);
			goto fail_cleanup;
		}
	}

	pr_info("%s() : identity map domain ready : 0x%llX ... 0x%llX : 0x%08lX\n",
		__func__, system_ram_region[0], system_ram_region[1],
		cfg->pgsize_bitmap);

	identity_map_iopt_ops = ops;

	return 0;

fail_cleanup:
	free_io_pgtable_ops(ops);

	return ret;
}

static int __init qiommu_init(void)
{
	int ret;

	/* Allocate qiommu_domain dummy dma devices */
	qiommu_domain_coherent_dev = alloc_qiommu_domain_dma_dev(true);
	if (!qiommu_domain_coherent_dev)
		return -ENOMEM;

	qiommu_domain_noncoherent_dev = alloc_qiommu_domain_dma_dev(false);
	if (!qiommu_domain_noncoherent_dev) {
		kfree(qiommu_domain_coherent_dev);
		return -ENOMEM;
	}

	ret = platform_driver_register(&qiommu_driver);
	if (ret)
		return ret;

	/*
	 * We only register qiommu_ops with the various bus types if a qiommu
	 * was successfully probed. This check is important because there is no
	 * mechanism to switch iommu_ops for a bus once they have been assigned.
	 */
	if (atomic_read(&num_qiommu_devices) > 0) {
		if (atomic_read(&num_identity_map_qiommus))
			qiommu_identity_map_init();

		if (iommu_present(&platform_bus_type)) {
			pr_err("%s() : ops != qiommu_ops were assigned to the platform bus, but %d qiommus were probed!\n",
			       __func__, atomic_read(&num_qiommu_devices));
			BUG();
		}
		bus_set_iommu(&platform_bus_type, &qiommu_ops);

#if defined(CONFIG_PCI)
		if (iommu_present(&pci_bus_type)) {
			pr_err("%s() : ops != qiommu_ops were assigned to the pci bus, but %d qiommus were probed!\n",
			       __func__, atomic_read(&num_qiommu_devices));
			BUG();
		}
		bus_set_iommu(&pci_bus_type, &qiommu_ops);
#endif
	}

	return 0;
}

static void __exit qiommu_exit(void)
{
	return platform_driver_unregister(&qiommu_driver);
}

subsys_initcall(qiommu_init);
module_exit(qiommu_exit);
