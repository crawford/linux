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
 * This file implements early detection/parsing of I/O mapping
 * reported to OS through firmware via I/O Remapping Table (IORT)
 * IORT document number: ARM DEN 0049A
 */

#define pr_fmt(fmt)	"ACPI: IORT: " fmt

#include <linux/export.h>
#include <linux/iommu.h>
#include <linux/iort.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct iort_its_msi_chip {
	struct list_head	list;
	struct fwnode_handle	*fw_node;
	u32			translation_id;
};

static int iort_dev_match(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const char *name = data;

	return !strcmp(pdev->name, name);
}

/**
 * iort_find_iommu_device- Retrieve IOMMU platform_device associated with
 *			   IORT node
 *
 * @node: IORT table node associated with the device
 *
 * Returns: device on success
 *          NULL on failure
 */
struct platform_device *iort_find_iommu_device(struct acpi_iort_node *node)
{
	struct acpi_iort_node *curr;
	struct device *dev = NULL;
	const struct iort_iommu_config *cfg = iort_get_iommu_config(node);

	if (!cfg)
		return NULL;

	while ((dev = bus_find_device(&platform_bus_type, dev,
				      (void *)cfg->name, iort_dev_match))) {
		curr = *(struct acpi_iort_node **) dev_get_platdata(dev);
		if (curr == node)
			return to_platform_device(dev);
	}

	return NULL;
}

struct iort_ops_node {
	struct list_head list;
	struct acpi_iort_node *node;
	const struct iommu_ops *ops;
	int (*iommu_xlate)(struct device *dev, u32 streamid,
			   struct acpi_iort_node *node);
};
static LIST_HEAD(iort_iommu_ops);
static DEFINE_SPINLOCK(iort_ops_lock);

/**
 * iort_smmu_set_ops - Create iort_ops_node and use it to register
 *		       iommu data in the iort_iommu_ops list.
 *
 * @node: IORT table node associated with the IOMMU
 * @ops: IOMMU operations associated with the IORT node
 * @iommu_xlate: iommu translate function to be used to carry out stream id
 *		 translation
 *
 * Returns: 0 on success
 *          -ENOMEM on failure
 */
int iort_smmu_set_ops(struct acpi_iort_node *node,
		       const struct iommu_ops *ops,
		       int (*iommu_xlate)(struct device *dev, u32 streamid,
					  struct acpi_iort_node *node))
{
	struct iort_ops_node *iommu = kzalloc(sizeof(*iommu), GFP_KERNEL);

	if (WARN_ON(!iommu))
		return -ENOMEM;

	INIT_LIST_HEAD(&iommu->list);
	iommu->node = node;
	iommu->ops = ops;
	iommu->iommu_xlate = iommu_xlate;

	spin_lock(&iort_ops_lock);
	list_add_tail(&iommu->list, &iort_iommu_ops);
	spin_unlock(&iort_ops_lock);

	return 0;
}

/**
 * iort_smmu_get_ops_node - Retrieve iort_ops_node associated with an
 *			    IORT node.
 *
 * @node: IORT table node to be looked-up
 *
 * Returns: iort_ops_node pointer on success
 *          NULL on failure
*/
const struct iort_ops_node *iort_smmu_get_ops_node(struct acpi_iort_node *node)
{
	struct iort_ops_node *curr, *iommu_node = NULL;

	spin_lock(&iort_ops_lock);
	list_for_each_entry(curr, &iort_iommu_ops, list) {
		if (curr->node == node) {
			iommu_node = curr;
			break;
		}
	}
	spin_unlock(&iort_ops_lock);

	return iommu_node;
}

typedef acpi_status (*iort_find_node_callback)
	(struct acpi_iort_node *node, void *context);

/* Root pointer to the mapped IORT table */
static struct acpi_table_header *iort_table;

static LIST_HEAD(iort_msi_chip_list);
static DEFINE_SPINLOCK(iort_msi_chip_lock);

/**
 * iort_register_domain_token() - register domain token and related ITS ID
 * to the list from where we can get it back later on.
 * @translation_id: ITS ID.
 * @token: Domain token.
 *
 * Returns: 0 on success, -ENOMEM if no memory when allocating list element
 */
int iort_register_domain_token(int trans_id, struct fwnode_handle *fw_node)
{
	struct iort_its_msi_chip *its_msi_chip;

	its_msi_chip = kzalloc(sizeof(*its_msi_chip), GFP_KERNEL);
	if (!its_msi_chip)
		return -ENOMEM;

	its_msi_chip->fw_node = fw_node;
	its_msi_chip->translation_id = trans_id;

	spin_lock(&iort_msi_chip_lock);
	list_add(&its_msi_chip->list, &iort_msi_chip_list);
	spin_unlock(&iort_msi_chip_lock);

	return 0;
}

/**
 * iort_deregister_domain_token() - Deregister domain token based on ITS ID
 * @translation_id: ITS ID.
 *
 * Returns: none.
 */
void iort_deregister_domain_token(int trans_id)
{
	struct iort_its_msi_chip *its_msi_chip, *t;

	spin_lock(&iort_msi_chip_lock);
	list_for_each_entry_safe(its_msi_chip, t, &iort_msi_chip_list, list) {
		if (its_msi_chip->translation_id == trans_id) {
			list_del(&its_msi_chip->list);
			kfree(its_msi_chip);
			break;
		}
	}
	spin_unlock(&iort_msi_chip_lock);
}

/**
 * iort_find_domain_token() - Find domain token based on given ITS ID
 * @translation_id: ITS ID.
 *
 * Returns: domain token when find on the list, NULL otherwise
 */
struct fwnode_handle *iort_find_domain_token(int trans_id)
{
	struct fwnode_handle *fw_node = NULL;
	struct iort_its_msi_chip *its_msi_chip;

	spin_lock(&iort_msi_chip_lock);
	list_for_each_entry(its_msi_chip, &iort_msi_chip_list, list) {
		if (its_msi_chip->translation_id == trans_id) {
			fw_node = its_msi_chip->fw_node;
			break;
		}
	}
	spin_unlock(&iort_msi_chip_lock);

	return fw_node;
}

static struct acpi_iort_node *
iort_scan_node(enum acpi_iort_node_type type,
	       iort_find_node_callback callback, void *context)
{
	struct acpi_iort_node *iort_node, *iort_end;
	struct acpi_table_iort *iort;
	int i;

	/* Get the first IORT node */
	iort = (struct acpi_table_iort *)iort_table;
	iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort,
				 iort->node_offset);
	iort_end = ACPI_ADD_PTR(struct acpi_iort_node, iort_table,
				iort_table->length);

	for (i = 0; i < iort->node_count; i++) {
		if (WARN_TAINT(iort_node >= iort_end, TAINT_FIRMWARE_WORKAROUND,
			       "IORT node pointer overflows, bad table!\n"))
			return NULL;

		if (iort_node->type == type) {
			if (ACPI_SUCCESS(callback(iort_node, context)))
				return iort_node;
		}

		iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort_node,
					 iort_node->length);
	}

	return NULL;
}

static struct acpi_iort_node *
iort_find_parent_node(struct acpi_iort_node *node)
{
	struct acpi_iort_id_mapping *id;

	if (!node || !node->mapping_offset || !node->mapping_count)
		return NULL;

	id = ACPI_ADD_PTR(struct acpi_iort_id_mapping, node,
			  node->mapping_offset);

	if (!id->output_reference) {
		pr_err(FW_BUG "[node %p type %d] ID map has NULL parent reference\n",
		       node, node->type);
		return NULL;
	}

	node = ACPI_ADD_PTR(struct acpi_iort_node, iort_table,
			    id->output_reference);

	return node;
}

static acpi_status
iort_match_callback(struct acpi_iort_node *node, void *context)
{
	return AE_OK;
}

bool iort_node_match(u8 type)
{
	struct acpi_iort_node *node;

	node = iort_scan_node(type, iort_match_callback, NULL);

	return node != NULL;
}

static acpi_status
iort_match_node_callback(struct acpi_iort_node *node, void *context)
{
	struct device *dev = context;

	switch (node->type) {
	case ACPI_IORT_NODE_NAMED_COMPONENT: {
		struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
		struct acpi_device *adev = to_acpi_device_node(dev->fwnode);
		struct acpi_iort_named_component *ncomp;

		if (!adev)
			break;

		ncomp = (struct acpi_iort_named_component *)node->node_data;

		if (ACPI_FAILURE(acpi_get_name(adev->handle,
					       ACPI_FULL_PATHNAME, &buffer))) {
			dev_warn(dev, "Can't get device full path name\n");
			break;
		}

		if (!strcmp(ncomp->device_name, (char *)buffer.pointer))
			return AE_OK;

		break;
	}
	case ACPI_IORT_NODE_PCI_ROOT_COMPLEX: {
		struct acpi_iort_root_complex *pci_rc;
		struct pci_bus *bus;

		bus = to_pci_bus(dev);
		pci_rc = (struct acpi_iort_root_complex *)node->node_data;

		/*
		 * It is assumed that PCI segment numbers maps one-to-one
		 * with root complexes. Each segment number can represent only
		 * one root complex.
		 */
		if (pci_rc->pci_segment_number == pci_domain_nr(bus))
			return AE_OK;

		break;
	}
	}

	return AE_NOT_FOUND;
}

static struct acpi_iort_node *
iort_node_map_rid(struct acpi_iort_node *node, u32 rid_in,
		  u32 *rid_out, u8 type)
{

	if (!node)
		goto out;

	/* Go upstream */
	while (node->type != type) {
		struct acpi_iort_id_mapping *id;
		int i, found = 0;

		/* Exit when no mapping array */
		if (!node->mapping_offset || !node->mapping_count)
			return NULL;

		id = ACPI_ADD_PTR(struct acpi_iort_id_mapping, node,
				  node->mapping_offset);

		for (i = 0, found = 0; i < node->mapping_count; i++, id++) {
			/*
			 * Single mapping is not translation rule,
			 * lets move on for this case
			 */
			if (id->flags & ACPI_IORT_ID_SINGLE_MAPPING) {
				if (node->type != ACPI_IORT_NODE_SMMU) {
					rid_in = id->output_base;
					found = 1;
					break;
				}

				pr_warn(FW_BUG "[node %p type %d] SINGLE MAPPING flag not allowed for SMMU node, skipping ID map\n",
					node, node->type);
				continue;
			}

			if (rid_in < id->input_base ||
			    (rid_in > id->input_base + id->id_count))
				continue;

			rid_in = id->output_base + (rid_in - id->input_base);
			found = 1;
			break;
		}

		if (!found)
			return NULL;

		/* Firmware bug! */
		if (!id->output_reference) {
			pr_err(FW_BUG "[node %p type %d] ID map has NULL parent reference\n",
			       node, node->type);
			return NULL;
		}

		node = ACPI_ADD_PTR(struct acpi_iort_node, iort_table,
				    id->output_reference);
	}

out:
	if (rid_out)
		*rid_out = rid_in;
	return node;
}

static struct acpi_iort_node *
iort_find_dev_node(struct device *dev)
{
	struct pci_bus *pbus;

	if (!dev_is_pci(dev))
		return iort_scan_node(ACPI_IORT_NODE_NAMED_COMPONENT,
				      iort_match_node_callback, dev);

	/* Find a PCI root bus */
	pbus = to_pci_dev(dev)->bus;
	while (!pci_is_root_bus(pbus))
		pbus = pbus->parent;

	return iort_scan_node(ACPI_IORT_NODE_PCI_ROOT_COMPLEX,
			      iort_match_node_callback, &pbus->dev);
}

/**
 * iort_msi_map_rid() - Map a MSI requester ID for a device
 * @dev: The device for which the mapping is to be done.
 * @req_id: The device requester ID.
 *
 * Returns: mapped MSI RID on success, input requester ID otherwise
 */
u32 iort_msi_map_rid(struct device *dev, u32 req_id)
{
	struct acpi_iort_node *node;
	u32 dev_id;

	if (!iort_table)
		return req_id;

	node = iort_find_dev_node(dev);
	if (!node) {
		dev_err(dev, "can't find related IORT node\n");
		return req_id;
	}

	if (!iort_node_map_rid(node, req_id, &dev_id,
			       ACPI_IORT_NODE_ITS_GROUP))
		return req_id;

	return dev_id;
}

/**
 * iort_dev_find_its_id() - Find the ITS identifier for a device
 * @dev: The device.
 * @idx: Index of the ITS identifier list.
 * @its_id: ITS identifier.
 *
 * Returns: 0 on success, appropriate error value otherwise
 */
static int
iort_dev_find_its_id(struct device *dev, u32 req_id, unsigned int idx,
		     int *its_id)
{
	struct acpi_iort_its_group *its;
	struct acpi_iort_node *node;

	node = iort_find_dev_node(dev);
	if (!node) {
		dev_err(dev, "can't find related IORT node\n");
		return -ENXIO;
	}

	node = iort_node_map_rid(node, req_id, NULL, ACPI_IORT_NODE_ITS_GROUP);
	if (!node) {
		dev_err(dev, "can't find related ITS node\n");
		return -ENXIO;
	}

	/* Move to ITS specific data */
	its = (struct acpi_iort_its_group *)node->node_data;
	if (idx > its->its_count) {
		dev_err(dev, "requested ITS ID index [%d] is greater than available [%d]\n",
			idx, its->its_count);
		return -ENXIO;
	}

	*its_id = its->identifiers[idx];
	return 0;
}

/**
 * iort_get_device_domain() - Find MSI domain related to a device
 * @dev: The device.
 * @req_id: Requester ID for the device.
 *
 * Returns: the MSI domain for this device, NULL otherwise
 */
struct irq_domain *
iort_get_device_domain(struct device *dev, u32 req_id)
{
	static struct fwnode_handle *handle;
	int its_id;

	if (!iort_table)
		return NULL;

	if (iort_dev_find_its_id(dev, req_id, 0, &its_id))
		return NULL;

	handle = iort_find_domain_token(its_id);
	if (!handle)
		return NULL;

	return irq_find_matching_fwnode(handle, DOMAIN_BUS_PCI_MSI);
}

static int __get_pci_rid(struct pci_dev *pdev, u16 alias, void *data)
{
	u32 *rid = data;

	*rid = alias;
	return 0;
}

/**
 * iort_iommu_configure - Set-up IOMMU configuration for a device.
 *
 * @dev: device to configure
 *
 * Returns: iommu_ops pointer on configuration success
 *          NULL on configuration failure
 */
const struct iommu_ops *iort_iommu_configure(struct device *dev)
{
	struct acpi_iort_node *node, *parent;
	const struct iort_ops_node *iort_ops;
	u32 rid = 0, devid = 0;

	if (dev_is_pci(dev)) {
		struct pci_bus *bus = to_pci_dev(dev)->bus;

		pci_for_each_dma_alias(to_pci_dev(dev), __get_pci_rid,
				       &rid);

		node = iort_scan_node(ACPI_IORT_NODE_PCI_ROOT_COMPLEX,
				      iort_match_node_callback, &bus->dev);
	} else {
		node = iort_scan_node(ACPI_IORT_NODE_NAMED_COMPONENT,
				      iort_match_node_callback, dev);
	}

	if (!node)
		return NULL;

	parent = iort_find_parent_node(node);

	if (!parent)
		return NULL;

	iort_ops = iort_smmu_get_ops_node(parent);

	if (iort_ops && iort_ops->iommu_xlate) {
		iort_node_map_rid(node, rid, &devid, parent->type);
		iort_ops->iommu_xlate(dev, devid, parent);
		return iort_ops->ops;
	}

	return NULL;
}

static int __init
add_smmu_platform_device(const struct iort_iommu_config *iort_cfg,
			 struct acpi_iort_node *node)
{
	struct platform_device *pdev;
	struct resource *r;
	enum dev_dma_attr attr;
	int ret, count;

	pdev = platform_device_alloc(iort_cfg->name, PLATFORM_DEVID_AUTO);
	if (!pdev)
		return PTR_ERR(pdev);

	count = iort_cfg->iommu_count_resources(node);

	r = kcalloc(count, sizeof(*r), GFP_KERNEL);
	if (!r) {
		ret = -ENOMEM;
		goto dev_put;
	}

	iort_cfg->iommu_init_resources(r, node);

	ret = platform_device_add_resources(pdev, r, count);
	/*
	 * Resources are duplicated in platform_device_add_resources,
	 * free their allocated memory
	 */
	kfree(r);

	if (ret)
		goto dev_put;

	/*
	 * Add a copy of IORT node pointer to platform_data to
	 * be used to retrieve IORT data information.
	 */
	ret = platform_device_add_data(pdev, &node, sizeof(node));
	if (ret)
		goto dev_put;

	pdev->dev.dma_mask = kmalloc(sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
	if (!pdev->dev.dma_mask) {
		ret = -ENOMEM;
		goto dev_put;
	}

	/*
	 * Set default dma mask value for the table walker,
	 * to be overridden on probing with correct value.
	 */
	*pdev->dev.dma_mask = DMA_BIT_MASK(32);
	pdev->dev.coherent_dma_mask = *pdev->dev.dma_mask;

	attr = iort_cfg->iommu_is_coherent(node) ?
			     DEV_DMA_COHERENT : DEV_DMA_NON_COHERENT;

	/* Configure DMA for the page table walker */
	arch_setup_dma_ops(&pdev->dev, 0, 0, NULL,
					 attr == DEV_DMA_COHERENT);

	ret = platform_device_add(pdev);
	if (ret)
		goto dma_deconfigure;

	ret = iort_cfg->iommu_init(node);
	if (ret)
		goto dma_deconfigure;

	return 0;

dma_deconfigure:
	arch_teardown_dma_ops(&pdev->dev);
	kfree(pdev->dev.dma_mask);

dev_put:
	platform_device_put(pdev);

	return ret;
}

static int __init iort_smmu_init(void)
{
	struct acpi_iort_node *iort_node, *iort_end;
	struct acpi_table_iort *iort;
	int i, ret;

	/*
	 * iort_table and iort both point to the start of IORT table, but
	 * have different struct types
	 */
	iort = (struct acpi_table_iort *)iort_table;

	/* Get the first IORT node */
	iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort,
				 iort->node_offset);
	iort_end = ACPI_ADD_PTR(struct acpi_iort_node, iort_table,
				iort_table->length);

	for (i = 0; i < iort->node_count; i++) {
		const struct iort_iommu_config *ops;

		if (iort_node >= iort_end) {
			pr_err("iort node pointer overflows, bad table\n");
			return -EINVAL;
		}

		ops = iort_get_iommu_config(iort_node);
		if (!ops)
			goto next;

		ret = add_smmu_platform_device(ops, iort_node);
		if (ret)
			return ret;
next:
		iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort_node,
					 iort_node->length);
	}

	return 0;
}

static int __init iort_table_detect(void)
{
	acpi_status status;

	if (acpi_disabled)
		return -ENODEV;

	status = acpi_get_table(ACPI_SIG_IORT, 0, &iort_table);
	if (ACPI_FAILURE(status) && status != AE_NOT_FOUND) {
		const char *msg = acpi_format_exception(status);
		pr_err("Failed to get table, %s\n", msg);
		return -EINVAL;
	}

	iort_smmu_init();

	return 0;
}
arch_initcall(iort_table_detect);
