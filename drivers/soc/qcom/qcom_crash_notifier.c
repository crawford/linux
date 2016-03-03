/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/acpi.h>

static acpi_handle crash_handle;

struct notifier_data {
	struct device *dev;
};

static int panic_notifier(struct notifier_block *self, unsigned long unused1,
			  void *unused2)
{
	acpi_status acpi_ret;

	acpi_ret = acpi_execute_simple_method(crash_handle, NULL, 0);
	if (ACPI_FAILURE(acpi_ret))
		return NOTIFY_BAD;
	return NOTIFY_DONE;
}

static struct notifier_block fw_panic_notifier = {
	.notifier_call  = panic_notifier,
	.next           = NULL,
	.priority       = INT_MAX,
};

static int notifier_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct notifier_data *drvdata;
	acpi_handle handle = ACPI_HANDLE(dev);
	acpi_handle tmp_handle;

	if (ACPI_FAILURE(acpi_get_handle(handle, "CRSH", &tmp_handle))) {
		dev_err(dev, "cannot locate CRSH method\n");
		return -ENODEV;
	}

	crash_handle = tmp_handle;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = dev;
	platform_set_drvdata(pdev, drvdata);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &fw_panic_notifier);

	dev_dbg(dev, "Panic Notifier driver initialized\n");

	return 0;
}

static int notifier_remove(struct platform_device *pdev)
{
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &fw_panic_notifier);
	return 0;
}

static const struct acpi_device_id panic_notifier_acpi_match[] = {
	{ "QCOM80F5" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, panic_notifier_acpi_match);

static struct platform_driver panic_notifier_driver = {
	.probe		= notifier_probe,
	.remove		= notifier_remove,
	.driver		= {
		.name = "qcom_panic_notifier",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(panic_notifier_acpi_match),
	},
};

module_platform_driver(panic_notifier_driver);


MODULE_DESCRIPTION("QCOM crash notifier driver");
MODULE_LICENSE("GPL v2");
