/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#ifndef __LINUX_CORESIGHT_PROPERTY_H
#define __LINUX_CORESIGHT_PROPERTY_H

#ifdef CONFIG_OF_CORESIGHT
extern struct fwnode_handle
	*of_get_coresight_child_by_index(struct device_node *node,
					 const char *propname,
					 int index);
#else
static inline struct fwnode_handle
	*of_get_coresight_child_by_index(struct device_node *node,
					 const char *propname,
					 int index)
{
	return NULL;
}
#endif

#ifdef CONFIG_CORESIGHT_ACPI
extern struct fwnode_handle
	*acpi_get_coresight_child_by_index(struct acpi_device *adev,
					   const char *propname,
					   int index);
#else
static inline struct fwnode_handle
	*acpi_get_coresight_child_by_index(struct acpi_device *adev,
					   const char *propname,
					   int index)
{
	return NULL;
}
#endif

#ifdef CONFIG_CORESIGHT_PROPERTY
extern struct coresight_platform_data *device_get_coresight_platform_data(
				struct device *dev);
#else
static inline struct coresight_platform_data
	*device_get_coresight_platform_data(struct device *dev)
{
	return NULL;
}
#endif

#endif
