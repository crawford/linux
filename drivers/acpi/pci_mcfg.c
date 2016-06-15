/*
 * Copyright (C) 2016 Broadcom
 *	Author: Jayachandran C <jchandra@broadcom.com>
 * Copyright (C) 2016 Semihalf
 * 	Author: Tomasz Nowicki <tn@semihalf.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#define pr_fmt(fmt) "ACPI: " fmt

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci-acpi.h>
#include <linux/pci-ecam.h>

/* Root pointer to the mapped MCFG table */
static struct acpi_table_mcfg *mcfg_table;

/* Structure to hold entries from the MCFG table */
struct mcfg_entry {
	struct list_head	list;
	phys_addr_t		addr;
	u16			segment;
	u8			bus_start;
	u8			bus_end;
};

/* List to save MCFG entries */
static LIST_HEAD(pci_mcfg_list);

static bool pci_mcfg_fixup_match(struct pci_cfg_fixup *f,
				 struct acpi_table_header *h)
{
	int olen = min_t(u8, strlen(f->oem_id), ACPI_OEM_ID_SIZE);
	int tlen = min_t(u8, strlen(f->oem_table_id), ACPI_OEM_TABLE_ID_SIZE);

	return (!strncmp(f->oem_id, h->oem_id, olen) &&
		!strncmp(f->oem_table_id, h->oem_table_id, tlen) &&
		f->oem_revision == h->oem_revision);
}

struct pci_ecam_ops *pci_mcfg_get_ops(struct acpi_pci_root *root)
{
	int bus_num = root->secondary.start;
	int domain = root->segment;
	struct pci_cfg_fixup *f;

	if (!mcfg_table)
		return &pci_generic_ecam_ops;

	/*
	 * Match against platform specific quirks and return corresponding CAM
	 * ops.
	 *
	 * First match against PCI topology <domain:bus> then use OEM ID, OEM
	 * table ID, and OEM revision from MCFG table standard header.
	 */
	for (f = __start_acpi_mcfg_fixups; f < __end_acpi_mcfg_fixups; f++) {
		if ((f->domain == domain || f->domain == PCI_MCFG_DOMAIN_ANY) &&
		    (f->bus_num == bus_num || f->bus_num == PCI_MCFG_BUS_ANY) &&
		    pci_mcfg_fixup_match(f, &mcfg_table->header)) {
			pr_info("Handling %s %s r%d PCI MCFG quirks\n",
				f->oem_id, f->oem_table_id, f->oem_revision);
			return f->ops;
		}
	}
	/* No quirks, use ECAM */
	return &pci_generic_ecam_ops;
}

phys_addr_t pci_mcfg_lookup(u16 seg, struct resource *bus_res)
{
	struct mcfg_entry *e;

	/*
	 * We expect exact match, unless MCFG entry end bus covers more than
	 * specified by caller.
	 */
	list_for_each_entry(e, &pci_mcfg_list, list) {
		if (e->segment == seg && e->bus_start == bus_res->start &&
		    e->bus_end >= bus_res->end)
			return e->addr;
	}

	return 0;
}

static __init int pci_mcfg_parse(struct acpi_table_header *header)
{
	struct acpi_mcfg_allocation *mptr;
	struct mcfg_entry *e, *arr;
	int i, n;

	if (header->length < sizeof(struct acpi_table_mcfg))
		return -EINVAL;

	n = (header->length - sizeof(struct acpi_table_mcfg)) /
					sizeof(struct acpi_mcfg_allocation);
	mcfg_table = (struct acpi_table_mcfg *)header;
	mptr = (struct acpi_mcfg_allocation *) &mcfg_table[1];

	arr = kcalloc(n, sizeof(*arr), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	for (i = 0, e = arr; i < n; i++, mptr++, e++) {
		e->segment = mptr->pci_segment;
		e->addr =  mptr->address;
		e->bus_start = mptr->start_bus_number;
		e->bus_end = mptr->end_bus_number;
		list_add(&e->list, &pci_mcfg_list);
	}

	pr_info("MCFG table detected, %d entries\n", n);
	return 0;
}

/* Interface called by ACPI - parse and save MCFG table */
void __init pci_mmcfg_late_init(void)
{
	int err = acpi_table_parse(ACPI_SIG_MCFG, pci_mcfg_parse);
	if (err)
		pr_err("Failed to parse MCFG (%d)\n", err);
}
