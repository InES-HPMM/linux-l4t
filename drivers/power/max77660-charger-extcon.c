/*
 * max77660-charger-extcon.c -- MAXIM MAX77660 VBUS detection.
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
#include <linux/mfd/max77660/max77660-core.h>

struct max77660_chg_extcon {
	struct device		*dev;
	struct device		*parent;
	struct extcon_dev	*edev;
	struct regulator_dev	*rdev;
	int			irq;
};

const char *max77660_excon_cable[] = {
	[0] = "USB",
	NULL,
};

static int max77660_chg_extcon_cable_update(
		struct max77660_chg_extcon *chg_extcon)
{
	int ret;
	u8 status;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGSTAT, &status);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "CHSTAT read failed: %d\n", ret);
		return ret;
	}
	if (status & MAX77660_CHG_CHGINT_DC_UVP)
		extcon_set_cable_state(chg_extcon->edev, "USB", false);
	else
		extcon_set_cable_state(chg_extcon->edev, "USB", true);

	dev_info(chg_extcon->dev, "VBUS %s status: 0x%02x\n",
		(status & MAX77660_CHG_CHGINT_DC_UVP) ? "Invalid" : "Valid",
		status);

	return 0;
}

static irqreturn_t max77660_chg_extcon_irq(int irq, void *data)
{
	struct max77660_chg_extcon *chg_extcon = data;
	u8 status;
	int ret;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGINT, &status);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "CHGSTAT read failed: %d\n", ret);
		goto out;
	}

	if (status & MAX77660_CHG_CHGINT_DC_UVP)
		max77660_chg_extcon_cable_update(chg_extcon);
	else
		dev_err(chg_extcon->dev, "CHG-IRQ for unknown reason, 0x%02x\n",
			status);
out:
	return IRQ_HANDLED;
}

static int max77660_vbus_enable_time(struct regulator_dev *rdev)
{
	 return 500000;
}

static int max77660_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	int ret;
	u8 val;

	ret = max77660_reg_read(chg_extcon->parent, MAX77660_CHG_SLAVE,
					MAX77660_CHARGER_RBOOST, &val);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "RBOOST read failed: %d\n", ret);
		return ret;
	}
	return !!(val & MAX77660_RBOOST_RBOOSTEN);
}

static int max77660_vbus_enable(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	int ret;

	ret = max77660_reg_update(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST,
			MAX77660_RBOOST_RBOUT_VOUT(0x6),
			MAX77660_RBOOST_RBOUT_MASK);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "RBOOST update failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_set_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST, MAX77660_RBOOST_RBOOSTEN);
	if (ret < 0)
		dev_err(chg_extcon->dev, "RBOOST setbits failed: %d\n", ret);
	return ret;
}

static int max77660_vbus_disable(struct regulator_dev *rdev)
{
	struct max77660_chg_extcon *chg_extcon =rdev_get_drvdata(rdev);
	 int ret;

	ret = max77660_reg_clr_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_RBOOST, MAX77660_RBOOST_RBOOSTEN);
	if (ret < 0)
		dev_err(chg_extcon->dev, "RBOOST clrbits failed: %d\n", ret);
	 return ret;
}

static struct regulator_ops max77660_vbus_ops = {
	.enable		= max77660_vbus_enable,
	.disable	= max77660_vbus_disable,
	.is_enabled	= max77660_vbus_is_enabled,
	.enable_time	= max77660_vbus_enable_time,
};

static struct regulator_desc max77660_vbus_desc = {
	.name = "max77660-vbus",
	.ops = &max77660_vbus_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int __devinit max77660_chg_extcon_probe(struct platform_device *pdev)
{
	struct max77660_chg_extcon *chg_extcon;
	struct max77660_platform_data *pdata;
	struct max77660_charger_platform_data *chg_pdata;
	struct extcon_dev *edev;
	struct regulator_config config = { };
	int ret;

	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata || !pdata->charger_pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -ENODEV;
	}

	chg_pdata = pdata->charger_pdata;

	chg_extcon = devm_kzalloc(&pdev->dev, sizeof(*chg_extcon), GFP_KERNEL);
	if (!chg_extcon) {
		dev_err(&pdev->dev, "Memory allocation failed for chg_extcon\n");
		return -ENOMEM;
	}

	edev = devm_kzalloc(&pdev->dev, sizeof(*edev), GFP_KERNEL);
	if (!edev) {
		dev_err(&pdev->dev, "Memory allocation failed for edev\n");
		return -ENOMEM;
	}
	chg_extcon->edev = edev;
	chg_extcon->edev->name = dev_name(&pdev->dev);
	chg_extcon->edev->supported_cable = max77660_excon_cable;

	chg_extcon->dev = &pdev->dev;
	chg_extcon->parent = pdev->dev.parent;
	dev_set_drvdata(&pdev->dev, chg_extcon);

	chg_extcon->irq = platform_get_irq(pdev, 0);

	ret = extcon_dev_register(chg_extcon->edev, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	/* Set initial state */
	ret = max77660_chg_extcon_cable_update(chg_extcon);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cable init failed: %d\n", ret);
		goto out;
	}

	ret = request_threaded_irq(chg_extcon->irq, NULL,
		max77660_chg_extcon_irq,
		IRQF_ONESHOT | IRQF_EARLY_RESUME, dev_name(chg_extcon->dev),
		chg_extcon);
	if (ret < 0) {
		dev_err(chg_extcon->dev,
			"request irq %d failed: %dn", chg_extcon->irq, ret);
		goto out;
	}

	ret = max77660_reg_clr_bits(chg_extcon->parent, MAX77660_CHG_SLAVE,
			MAX77660_CHARGER_CHGINTM, MAX77660_CHG_CHGINT_DC_UVP);
	if (ret < 0) {
		dev_err(chg_extcon->dev, "CHGINTM update failed: %d\n", ret);
		goto out_irq_free;
	}

	config.dev = &pdev->dev;
	config.init_data = chg_pdata->vbus_reg_init_data;
	config.driver_data = chg_extcon;

	chg_extcon->rdev = regulator_register(&max77660_vbus_desc, &config);
	if (IS_ERR(chg_extcon->rdev)) {
		ret = PTR_ERR(chg_extcon->rdev);
		dev_err(&pdev->dev, "Failed to register VBUS regulator: %d\n",
					ret);
		goto out_irq_free;
	}

	device_set_wakeup_capable(&pdev->dev, 1);
	return 0;

out_irq_free:
	free_irq(chg_extcon->irq, chg_extcon);
out:
	extcon_dev_unregister(chg_extcon->edev);
	return ret;
}

static int __devexit max77660_chg_extcon_remove(struct platform_device *pdev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(&pdev->dev);

	extcon_dev_unregister(chg_extcon->edev);
	free_irq(chg_extcon->irq, chg_extcon);
	regulator_unregister(chg_extcon->rdev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77660_chg_extcon_suspend(struct device *dev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(chg_extcon->irq);
	return 0;
}

static int max77660_chg_extcon_resume(struct device *dev)
{
	struct max77660_chg_extcon *chg_extcon = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(chg_extcon->irq);
	return 0;
};
#endif

static const struct dev_pm_ops max77660_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77660_chg_extcon_suspend,
				max77660_chg_extcon_resume)
};

static struct platform_driver max77660_chg_extcon_driver = {
	.probe = max77660_chg_extcon_probe,
	.remove = __devexit_p(max77660_chg_extcon_remove),
	.driver = {
		.name = "max77660-charger-extcon",
		.owner = THIS_MODULE,
		.pm = &max77660_pm_ops,
	},
};

static int __init max77660_chg_extcon_driver_init(void)
{
	return platform_driver_register(&max77660_chg_extcon_driver);
}
subsys_initcall_sync(max77660_chg_extcon_driver_init);

static void __exit max77660_chg_extcon_driver_exit(void)
{
	platform_driver_unregister(&max77660_chg_extcon_driver);
}
module_exit(max77660_chg_extcon_driver_exit);

MODULE_DESCRIPTION("max77660 charger-extcon driver");
MODULE_AUTHOR("Laxman Dewangan<ldewangan@nvidia.com>");
MODULE_ALIAS("platform:max77660-charger-extcon");
MODULE_LICENSE("GPL v2");
