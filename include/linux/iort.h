/*
 * Copyright (C) 2016, Semihalf
 *	Author: Tomasz Nowicki <tn@semihalf.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __IORT_H__
#define __IORT_H__

#include <linux/acpi.h>

struct fwnode_handle;
int iort_register_domain_token(int trans_id, struct fwnode_handle *fw_node);
void iort_deregister_domain_token(int trans_id);
struct fwnode_handle *iort_find_domain_token(int trans_id);
#ifdef CONFIG_IORT_TABLE
bool iort_node_match(u8 type);
u32 iort_msi_map_rid(struct device *dev, u32 req_id);
struct irq_domain *iort_get_device_domain(struct device *dev, u32 req_id);

/* IOMMU interface */
const struct iommu_ops *iort_iommu_configure(struct device *dev);
#else
static inline bool iort_node_match(u8 type) { return false; }
static inline u32 iort_msi_map_rid(struct device *dev, u32 req_id)
{ return req_id; }
static inline struct irq_domain *
iort_get_device_domain(struct device *dev, u32 req_id) { return NULL; }

/* IOMMU interface */
static inline const struct iommu_ops *
iort_iommu_configure(struct device *dev) { return NULL; }
#endif
int iort_smmu_set_ops(struct acpi_iort_node *node,
		      const struct iommu_ops *ops,
		      int (*iommu_xlate)(struct device *dev,
					 u32 streamid,
					 struct acpi_iort_node *node));

struct iort_iommu_config {
	const char *name;
	int (*iommu_init)(struct acpi_iort_node *node);
	bool (*iommu_is_coherent)(struct acpi_iort_node *node);
	int (*iommu_count_resources)(struct acpi_iort_node *node);
	void (*iommu_init_resources)(struct resource *res,
				     struct acpi_iort_node *node);
};

static inline const struct iort_iommu_config *
iort_get_iommu_config(struct acpi_iort_node *node)
{
	return NULL;
}

#endif /* __IORT_H__ */
