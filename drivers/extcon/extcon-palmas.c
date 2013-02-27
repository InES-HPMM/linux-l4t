/*
 * palmas-extcon.c -- Palmas VBUS detection in extcon framework.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/extcon.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/palmas.h>

#define MAX_INT_NAME	40
struct palmas_extcon {
	struct device		*dev;
	struct palmas		*palmas;
	struct extcon_dev	*edev;
	int			vbus_irq;
	int			id_irq;
	char			vbus_irq_name[MAX_INT_NAME];
	char			id_irq_name[MAX_INT_NAME];
};

const char *palmas_excon_cable[] = {
	[0] = "USB",
	[1] = "USB-Host",
	NULL,
};

static int palmas_extcon_cable_update(
		struct palmas_extcon *palma_econ)
{
	int ret;
	unsigned int status;

	ret = palmas_read(palma_econ->palmas, PALMAS_INTERRUPT_BASE,
				PALMAS_INT3_LINE_STATE,	&status);
	if (ret < 0) {
		dev_err(palma_econ->dev,
			"INT3_LINE_STATE read failed: %d\n", ret);
		return ret;
	}

	if (status & PALMAS_INT3_LINE_STATE_VBUS)
		extcon_set_cable_state(palma_econ->edev, "USB", true);
	else
		extcon_set_cable_state(palma_econ->edev, "USB", false);

	dev_info(palma_econ->dev, "VBUS %s status: 0x%02x\n",
		(status & PALMAS_INT3_LINE_STATE_VBUS) ? "Valid" : "Invalid",
		status);

	if (status & PALMAS_INT3_LINE_STATE_ID)
		extcon_set_cable_state(palma_econ->edev, "USB-Host", true);
	else
		extcon_set_cable_state(palma_econ->edev, "USB-Host", false);

	dev_info(palma_econ->dev, "ID %s status: 0x%02x\n",
		(status & PALMAS_INT3_LINE_STATE_VBUS) ? "Valid" : "Invalid",
		status);

	return 0;
}

static irqreturn_t palmas_extcon_irq(int irq, void *data)
{
	struct palmas_extcon *palma_econ = data;

	if (irq == palma_econ->vbus_irq)
		palmas_extcon_cable_update(palma_econ);
	else
		dev_err(palma_econ->dev, "Unknown interrupt %d\n", irq);

	return IRQ_HANDLED;
}

static int __devinit palmas_extcon_probe(struct platform_device *pdev)
{
	struct palmas_extcon *palma_econ;
	struct extcon_dev *edev;
	int ret;

	palma_econ = devm_kzalloc(&pdev->dev, sizeof(*palma_econ), GFP_KERNEL);
	if (!palma_econ) {
		dev_err(&pdev->dev, "Memory allocation failed for palma_econ\n");
		return -ENOMEM;
	}

	edev = devm_kzalloc(&pdev->dev, sizeof(*edev), GFP_KERNEL);
	if (!edev) {
		dev_err(&pdev->dev, "Memory allocation failed for edev\n");
		return -ENOMEM;
	}
	palma_econ->edev = edev;
	palma_econ->edev->name = dev_name(&pdev->dev);
	palma_econ->edev->supported_cable = palmas_excon_cable;

	palma_econ->dev = &pdev->dev;
	palma_econ->palmas = dev_get_drvdata(pdev->dev.parent);
	dev_set_drvdata(&pdev->dev, palma_econ);

	palma_econ->vbus_irq = platform_get_irq(pdev, 0);
	palma_econ->id_irq = platform_get_irq(pdev, 1);

	ret = extcon_dev_register(palma_econ->edev, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	/* Set initial state */
	ret = palmas_extcon_cable_update(palma_econ);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cable init failed: %d\n", ret);
		goto out;
	}

	snprintf(palma_econ->vbus_irq_name, MAX_INT_NAME,
			"vbus-%s\n", dev_name(palma_econ->dev));
	snprintf(palma_econ->id_irq_name, MAX_INT_NAME,
			"id-%s\n", dev_name(palma_econ->dev));

	ret = request_threaded_irq(palma_econ->vbus_irq, NULL,
		palmas_extcon_irq, IRQF_ONESHOT | IRQF_EARLY_RESUME,
		palma_econ->vbus_irq_name, palma_econ);
	if (ret < 0) {
		dev_err(palma_econ->dev, "request irq %d failed: %d\n",
			palma_econ->vbus_irq, ret);
		goto out;
	}

	ret = request_threaded_irq(palma_econ->id_irq, NULL,
		palmas_extcon_irq, IRQF_ONESHOT | IRQF_EARLY_RESUME,
		palma_econ->id_irq_name, palma_econ);
	if (ret < 0) {
		dev_err(palma_econ->dev, "request irq %d failed: %d\n",
			palma_econ->id_irq, ret);
		goto out_free_vbus;
	}

	device_set_wakeup_capable(&pdev->dev, 1);
	return 0;
out_free_vbus:
	free_irq(palma_econ->vbus_irq, palma_econ);
out:
	extcon_dev_unregister(palma_econ->edev);
	return ret;
}

static int __devexit palmas_extcon_remove(struct platform_device *pdev)
{
	struct palmas_extcon *palma_econ = dev_get_drvdata(&pdev->dev);

	extcon_dev_unregister(palma_econ->edev);
	free_irq(palma_econ->vbus_irq, palma_econ);
	free_irq(palma_econ->id_irq, palma_econ);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int palmas_extcon_suspend(struct device *dev)
{
	struct palmas_extcon *palma_econ = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(palma_econ->vbus_irq);
	return 0;
}

static int palmas_extcon_resume(struct device *dev)
{
	struct palmas_extcon *palma_econ = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(palma_econ->vbus_irq);
	return 0;
};
#endif

static const struct dev_pm_ops palmas_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(palmas_extcon_suspend,
				palmas_extcon_resume)
};

static struct platform_driver palmas_extcon_driver = {
	.probe = palmas_extcon_probe,
	.remove = __devexit_p(palmas_extcon_remove),
	.driver = {
		.name = "palmas-extcon",
		.owner = THIS_MODULE,
		.pm = &palmas_pm_ops,
	},
};

static int __init palmas_extcon_driver_init(void)
{
	return platform_driver_register(&palmas_extcon_driver);
}
subsys_initcall_sync(palmas_extcon_driver_init);

static void __exit palmas_extcon_driver_exit(void)
{
	platform_driver_unregister(&palmas_extcon_driver);
}
module_exit(palmas_extcon_driver_exit);

MODULE_DESCRIPTION("palmas extcon driver");
MODULE_AUTHOR("Laxman Dewangan<ldewangan@nvidia.com>");
MODULE_ALIAS("platform:palmas-extcon");
MODULE_LICENSE("GPL v2");
