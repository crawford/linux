/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "qiommu.h"

/*
 * Below are sample DT/ACPI configurations for a platform device QIOMMU with 2
 * masters and a PCI device qiommu for devices with segment_id==4.  Only the
 * topology relevant details are shown.  See qiommu.c for additional
 * configuration options that may be specified.
 *
 * DT:
 *	platform_device_qiommu {
 *		compatible = "qcom,qiommu";
 *		...skipping normal config details...
 *		qiommu-name = "PLATFORM_DEVICE_QIOMMU";
 *		master-names = "DMA0", "SDHC2";
 *		DMA0-addr = <...mmio_base_phys for DMA0...>;
 *		DMA0-sids = /bits/ 16 <0x1 0x2 0x3>;
 *		SDHC2-addr = <...mmio_base_phys for SDHC2...>;
 *		SDHC2-sids = /bits/ 16 <0x4>;
 *	};
 *
 *	pci4_device_qiommu {
 *		compatible = "qcom,qiommu";
 *		qiommu-name = "PCI4_QIOMMU";
 *		pci-qiommu-segment_id = /bits/ 16 <0x4>;
 *	};
 *
 * ACPI: (Just showing the _DSD section relevant to qiommu topology)
 *	ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
 *	Package()
 *	{
 *		Package(2) {"qiommu-name", "PLATFORM_DEVICE_QIOMMU"},
 *		Package(2) {"master-names", Package (2) {"DMA0", "SDHC2"}},
 *		Package(2) {"DMA0-addr", <...DMA0_base_phys...>}
 *		Package(2) {"DMA0-sids", Package (3) {0x1, 0x2, 0x3}},
 *		Package(2) {"SDHC2-addr", <...SDHC2_base_phys...>}
 *		Package(2) {"SDHC2-sids", Package (1) {0x4}}
 *	}
 *
 *	ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
 *	Package()
 *	{
 *		Package(2) {"qiommu-name", "PCI4_QIOMMU"},
 *		Package(2) {"pci-qiommu-segment_id", 0x4}
 *	}
 */

static LIST_HEAD(qiommu_devices); /* List of all qiommus */
static DEFINE_SPINLOCK(qiommu_devices_lock);

/* Use this to get the RID for a struct device on the pci bus */
#define PCI_RID(dev) \
({ \
	struct pci_dev *_pci_dev = to_pci_dev((dev)); \
	((_pci_dev->bus->number << 8) | (_pci_dev->devfn)); \
})

/* Use this to get the segment id for a struct device on the pci bus */
#define PCI_SEGMENT_ID(dev) \
({ \
	struct pci_dev *_pci_dev = to_pci_dev((dev)); \
	pci_domain_nr(_pci_dev->bus); \
})

/* RID combined with segment id uniquely identify a pci device */
#define PCI_MASTER_ID(dev) \
	(((PCI_SEGMENT_ID(dev)) << 16) | (PCI_RID(dev)))

/*
 * This macro essentially replicates BUG_ON but prints a message similar to
 * the WARN_ON macro. This should only be used for failures encountered when
 * parsing QIOMMU toplogy information from DT/ACPI. There is not really any
 * smart way to recover from such errors other than informing the user that
 * they need to revisit the QIOMMU config and fix whatever was broken.
 */
#define QIOMMU_TOP_ASSERT(cond, fmt, args...) \
do { \
	if (!(cond)) {\
		qiommu_err(qmmu, "QIOMMU_TOP_ASSERT(%s): %s():%d: " fmt, \
			   #cond, __func__, __LINE__, ##args); \
		BUG(); \
	} \
} while (0)

/* QIOMMU topology data for a master */
struct master_top_data {
	struct list_head list;
	uint64_t master_id;
	struct qiommu_master master;
};

struct qiommu_top_data {
	/* Toplogy data relevant to QIOMMUs managing a PCI root complex */
	bool is_pci_qiommu;
	uint16_t pci_segment_id;

	/* Topology data for masters physically connected to the QIOMMU */
	uint16_t num_masters;
	struct list_head masters_top_list;

	spinlock_t lock;
};

/*
 * Get IORESOURCE_MEM[0]->start for a platform device.  For qiommus or attached
 * masters on the platform bus, we depend on this being MMIO base phys.
 */
static phys_addr_t get_mmio_base_phys(struct device *dev)
{
	struct platform_device *pdev;
	struct resource *res;

	if (&platform_bus_type != dev->bus)
		return 0;
	pdev = container_of(dev, struct platform_device, dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	return (NULL == res)?(0):(res->start);
}

static struct master_top_data *
alloc_and_init_master_top_data(struct qiommu_device *qmmu)
{
	struct master_top_data *top;
	struct qiommu_master *master;

	top = devm_kzalloc(qmmu->dev, sizeof(*top), GFP_KERNEL);
	if (!top)
		return ERR_PTR(-ENOMEM);
	INIT_LIST_HEAD(&top->list);

	master = &top->master;
	master->qmmu = qmmu;

	return top;
}

/* build_prop_str("SATA0","-addr") => "SATA0-addr" */
static void build_prop_str(const char *name, const char *prop, char *dest,
			   size_t dest_size)
{
	strncpy(dest, name, dest_size - 1);
	strncat(dest, prop, dest_size - strlen(name) - 1);
}

static struct master_top_data *
build_top_data_for_plat_master(struct qiommu_device *qmmu,
			       const char *name)
{
	struct device *sdev = qmmu->dev;
	struct master_top_data *top;
	struct qiommu_master *master;
	int num_sids;
	char propname[64];
	int err;

	top = alloc_and_init_master_top_data(qmmu);
	QIOMMU_TOP_ASSERT(!IS_ERR(top), "");

	master = &top->master;
	master->name = name;

	/* Read <master_name>-addr which will become master_id */
	build_prop_str(name, "-addr", propname, sizeof(propname));
	err = device_property_read_u64(sdev, propname, &top->master_id);
	QIOMMU_TOP_ASSERT(!err, "read_u64(\"%s\") err=%d\n", propname, err);

	/* Get the number of sids used by the master */
	build_prop_str(name, "-sids", propname, sizeof(propname));
	num_sids = device_property_read_u16_array(sdev, propname, NULL, 0);
	master->num_sids = num_sids;
	QIOMMU_TOP_ASSERT(num_sids > 0 && num_sids <= ARRAY_SIZE(master->sids),
			  "invalid value, %d, for \"%s\"\n", num_sids,
			  propname);

	/* Read the array of sids used by the master */
	err = device_property_read_u16_array(sdev, propname,
					     master->sids, num_sids);
	QIOMMU_TOP_ASSERT(!err, "read_u16_array(\"%s\") err=%d\n",
			  propname, err);

	qiommu_dbg(qmmu, "loaded cfg data for platform master %s\n", name);

	return top;
}

int top_add_platform_qiommu(struct qiommu_device *qmmu)
{
	struct device *sdev = qmmu->dev;
	struct qiommu_top_data *qiommu_top = qmmu->top;
	struct master_top_data *master_top = NULL;
	int num_masters;
	const char **master_names = NULL;
	int i, err;

	/* Determine the number of named masters connected to the qiommu */
	if (!device_property_present(sdev, "master-names")) {
		qiommu_warn(qmmu, "no masters found for platform qiommu!!\n");
		return 0;
	}
	num_masters = device_property_read_string_array(sdev, "master-names",
							NULL, 0);
	if (num_masters == 0) {
		qiommu_info(qmmu, "no masters found in ACPI/DT\n");
		return 0;
	}
	QIOMMU_TOP_ASSERT(num_masters >= 0,
			  "read_string_array(master-names) err=%d\n",
			  num_masters);

	/* Read the names of the masters connected to this qiommu */
	master_names = kmalloc_array(num_masters, sizeof(char *), GFP_KERNEL);
	QIOMMU_TOP_ASSERT(master_names, "");
	err = device_property_read_string_array(sdev, "master-names",
						master_names, num_masters);
	QIOMMU_TOP_ASSERT(err >=  0, "read_string_array(master-names) err=%d\n",
			  err);

	/* Get the top info for the named masters and add to the qiommu's top */
	for (i = 0; i < num_masters; i++) {
		master_top = build_top_data_for_plat_master(qmmu,
							    master_names[i]);
		QIOMMU_TOP_ASSERT(!IS_ERR(master_top), "");

		spin_lock(&qiommu_top->lock);
		list_add_tail(&master_top->list, &qiommu_top->masters_top_list);
		qiommu_top->num_masters++;
		spin_unlock(&qiommu_top->lock);
	}

	return 0;
}

/* Fill in the toplogy information for a qiommu */
int qiommu_top_add_qiommu(struct qiommu_device *qmmu)
{
	struct qiommu_top_data *top;
	int err;

	top = devm_kzalloc(qmmu->dev, sizeof(*top), GFP_KERNEL);
	QIOMMU_TOP_ASSERT(top, "");

	INIT_LIST_HEAD(&top->masters_top_list);
	spin_lock_init(&top->lock);
	qmmu->top = top;

	/* Check if the qiommu is attached to a pci_root_complex */
	top->is_pci_qiommu = device_property_present(qmmu->dev,
						     "pci-qiommu-segment_id");
	err = device_property_read_u16(qmmu->dev, "pci-qiommu-segment_id",
				       &top->pci_segment_id);
	QIOMMU_TOP_ASSERT(!(top->is_pci_qiommu && err),
			  "read_u16(\"pci-qiommu-segment_id\") err=%d\n", err);
	if (top->is_pci_qiommu) {
		qiommu_info(qmmu, "qiommu for pci devices with segment_id == 0x%X\n",
			    top->pci_segment_id);
	} else
		err = top_add_platform_qiommu(qmmu);

	if (err)
		return err;

	/* Add qmmu to qiommu_devices */
	spin_lock(&qiommu_devices_lock);
	list_add_tail(&qmmu->list, &qiommu_devices);
	spin_unlock(&qiommu_devices_lock);

	return 0;
}

struct qiommu_master *top_alloc_pci_master(struct device *dev)
{
	struct qiommu_device *qmmu;
	struct qiommu_top_data *top = NULL;
	struct master_top_data *master_top;
	struct qiommu_master *master;
	uint16_t segment_id = PCI_SEGMENT_ID(dev);

	/* Try to find a pci qiommu that matches our segment_id */
	spin_lock(&qiommu_devices_lock);
	list_for_each_entry(qmmu, &qiommu_devices, list) {
		top = qmmu->top;
		if (top->is_pci_qiommu && segment_id == top->pci_segment_id)
			break;
		top = NULL;
	}
	spin_unlock(&qiommu_devices_lock);

	if (!top)
		return NULL;

	/* Allocate and configure a master for the pci device */
	master_top = alloc_and_init_master_top_data(qmmu);
	if (IS_ERR(master_top))
		return NULL;
	master_top->master_id = PCI_MASTER_ID(dev);

	master = &master_top->master;
	master->dev = dev;
	master->name = dev_name(dev);
	master->num_sids = 1;
	master->sids[0] = PCI_RID(dev);

	/* Add the allocated master to the top for the QIOMMU */
	spin_lock(&top->lock);
	list_add_tail(&master_top->list, &top->masters_top_list);
	top->num_masters++;
	spin_unlock(&top->lock);

	qiommu_info(qmmu, "allocated master for pci device %s\n",
		    dev_name(dev));

	return master;
}

struct qiommu_master *qiommu_top_find_master(struct device *dev)
{
	struct qiommu_device *qmmu;
	struct qiommu_top_data *top;
	struct qiommu_master *found_master = NULL;
	struct master_top_data *master_top;
	uint64_t master_id;

	if (dev_is_pci(dev))
		master_id = PCI_MASTER_ID(dev);
	else if (dev->bus == &platform_bus_type)
		master_id = get_mmio_base_phys(dev);
	else
		return NULL; /* Only pci and platform devices supported */

	spin_lock(&qiommu_devices_lock);
	list_for_each_entry(qmmu, &qiommu_devices, list) {
		top = qmmu->top;

		/* Only search QIOMMUs appropriate for the device's bus type */
		if (dev_is_pci(dev) != top->is_pci_qiommu)
			continue;

		spin_lock(&top->lock);
		list_for_each_entry(master_top, &top->masters_top_list, list) {
			if (master_id == master_top->master_id) {
				found_master = &master_top->master;
				break;
			}
		}
		spin_unlock(&top->lock);

		if (found_master)
			break;
	}
	spin_unlock(&qiommu_devices_lock);

	return found_master;
}

struct qiommu_master *qiommu_top_alloc_master(struct device *dev)
{
	struct qiommu_master *master;

	if (dev_is_pci(dev))
		return top_alloc_pci_master(dev);

	master = qiommu_top_find_master(dev);
	if (master && master->dev) {
		qiommu_info(master->qmmu, "not allocating master %s for %s because it was was already claimed by %s\n",
			    master->name, dev_name(dev), dev_name(master->dev));
		return NULL;
	} else if (master) {
		master->dev = dev;
		qiommu_info(master->qmmu, "allocated master %s for platform device %s\n",
			    master->name, dev_name(dev));
	}

	return master;
}

void qiommu_top_free_master(struct qiommu_master *master)
{
	struct qiommu_device *qmmu = master->qmmu;
	struct qiommu_top_data *top = qmmu->top;
	struct master_top_data *master_top = NULL;

	qiommu_info(qmmu, "releasing master %s\n", master->name);

	/*
	 * For platform devices just hide the master since the topology of the
	 * system is not changing, and the master may eventually get re-added.
	 */
	if (master->dev->bus == &platform_bus_type) {
		master->dev = NULL;
		return;
	}

	/*
	 * Masters can be physically removed from the pci bus so remove it from
	 * the topology for the relevant QIOMMU and cleanup any allocated
	 * resources.
	 */
	spin_lock(&top->lock);
	master_top = container_of(master, struct master_top_data, master);
	list_del(&master_top->list);
	top->num_masters--;
	spin_unlock(&top->lock);

	/* Beware this will also free master */
	devm_kfree(qmmu->dev, master_top);
}
