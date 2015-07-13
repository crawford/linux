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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/property.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include "coresight.h"
#include "coresight-property.h"

static struct fwnode_handle *get_coresight_child_by_index(struct device *dev,
							  const char *propname,
							  int index)
{
	if (IS_ENABLED(CONFIG_OF) && dev->of_node)
		return of_get_coresight_child_by_index(dev->of_node, propname,
						       index);

	return acpi_get_coresight_child_by_index(ACPI_COMPANION(dev),
						 propname, index);
}

struct coresight_platform_data *device_get_coresight_platform_data(
							struct device *dev)
{
	int i, ret = 0;
	struct fwnode_handle *child;
	struct coresight_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = device_property_read_u32(dev, "coresight-id", &pdata->id);
	if (ret)
		return ERR_PTR(ret);

	ret = device_property_read_string(dev, "coresight-name", &pdata->name);
	if (ret)
		return ERR_PTR(ret);

	ret = device_property_read_u32(dev, "coresight-nr-inports",
				   &pdata->nr_inports);
	if (ret)
		return ERR_PTR(ret);

	pdata->nr_outports = 0;

	if (device_property_present(dev, "coresight-outports"))
		pdata->nr_outports =
			device_property_read_u32_array(dev,
						       "coresight-outports",
						       NULL, 0);
	if (pdata->nr_outports > 0) {
		pdata->outports = devm_kzalloc(dev, pdata->nr_outports *
				       sizeof(*pdata->outports),
				       GFP_KERNEL);
		if (!pdata->outports)
			return ERR_PTR(-ENOMEM);

		ret = device_property_read_u32_array(dev, "coresight-outports",
						     (u32 *)pdata->outports,
						     pdata->nr_outports);
		if (ret)
			return ERR_PTR(ret);

		pdata->child_ids = devm_kzalloc(dev, pdata->nr_outports *
						sizeof(*pdata->child_ids),
						GFP_KERNEL);
		if (!pdata->child_ids)
			return ERR_PTR(-ENOMEM);

		for (i = 0; i < pdata->nr_outports; i++) {
			child = get_coresight_child_by_index(dev,
							     "coresight-child-list",
							     i);
			if (!child)
				return ERR_PTR(-EINVAL);

			ret = fwnode_property_read_u32(child, "coresight-id",
						   (u32 *)&pdata->child_ids[i]);
			fwnode_handle_put(child);
			if (ret)
				return ERR_PTR(ret);
		}

		pdata->child_ports = devm_kzalloc(dev, pdata->nr_outports *
						  sizeof(*pdata->child_ports),
						  GFP_KERNEL);
		if (!pdata->child_ports)
			return ERR_PTR(-ENOMEM);

		ret = device_property_read_u32_array(dev, "coresight-child-ports",
						     (u32 *)pdata->child_ports,
						     pdata->nr_outports);
		if (ret)
			return ERR_PTR(ret);
	}

	pdata->default_sink = device_property_read_bool(dev,
						    "coresight-default-sink");
	return pdata;
}
EXPORT_SYMBOL(device_get_coresight_platform_data);
