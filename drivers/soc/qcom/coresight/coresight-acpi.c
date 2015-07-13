/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/types.h>
#include <linux/acpi.h>

struct fwnode_handle *
acpi_get_coresight_child_by_index(struct acpi_device *adev,
				  const char *propname,
				  int index)
{
	struct acpi_reference_args args;
	int ret;

	ret = acpi_node_get_property_reference(&adev->fwnode,
					      propname, index, &args);
	if (ret)
		return NULL;

	return &args.adev->fwnode;
}
EXPORT_SYMBOL(acpi_get_coresight_child_by_index);
