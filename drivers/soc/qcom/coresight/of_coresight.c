/* Copyright (c) 2012-2013, 2015, The Linux Foundation. All rights reserved.
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
#include <linux/err.h>
#include <linux/of.h>

#include "coresight.h"
#include "coresight-cti.h"

struct fwnode_handle *of_get_coresight_child_by_index(struct device_node *node,
						      const char *propname,
						      int index)
{
	struct device_node *child_node;

	child_node = of_parse_phandle(node, propname, index);
	if (!child_node)
		return NULL;

	return &child_node->fwnode;

}
EXPORT_SYMBOL(of_get_coresight_child_by_index);

struct coresight_cti_data *of_get_coresight_cti_data(
				struct device *dev, struct device_node *node)
{
	int i, ret;
	uint32_t ctis_len;
	struct device_node *child_node;
	struct coresight_cti_data *ctidata;

	ctidata = devm_kzalloc(dev, sizeof(*ctidata), GFP_KERNEL);
	if (!ctidata)
		return ERR_PTR(-ENOMEM);

	if (of_get_property(node, "coresight-ctis", &ctis_len))
		ctidata->nr_ctis = ctis_len/sizeof(uint32_t);
	else
		return ERR_PTR(-EINVAL);

	if (ctidata->nr_ctis) {
		ctidata->names = devm_kzalloc(dev, ctidata->nr_ctis *
					      sizeof(*ctidata->names),
					      GFP_KERNEL);
		if (!ctidata->names)
			return ERR_PTR(-ENOMEM);

		for (i = 0; i < ctidata->nr_ctis; i++) {
			child_node = of_parse_phandle(node, "coresight-ctis",
						      i);
			if (!child_node)
				return ERR_PTR(-EINVAL);

			ret = of_property_read_string(child_node,
						      "coresight-name",
						      &ctidata->names[i]);
			of_node_put(child_node);
			if (ret)
				return ERR_PTR(ret);
		}
	}
	return ctidata;
}
EXPORT_SYMBOL(of_get_coresight_cti_data);
