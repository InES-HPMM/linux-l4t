/*
 * bq2419x-regulator.c --  bq2419x
 *
 * Regulator driver for BQ2419X charger.
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/bq2419x.h>
#include <linux/slab.h>

struct bq2419x_regulator_info {
	struct device *dev;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct bq2419x_chip *chip;
	int gpio_otg_iusb;
	bool power_off_on_suspend;
	struct mutex mutex;
	int shutdown_complete;
};

static int bq2419x_regulator_enable_time(struct regulator_dev *rdev)
{
	return 500000;
}

static int bq2419x_dcdc_enable(struct regulator_dev *rdev)
{
	struct bq2419x_regulator_info *bq = rdev_get_drvdata(rdev);
	int ret;

	dev_info(bq->dev, "%s called\n", __func__);
	if (gpio_is_valid(bq->gpio_otg_iusb))
		gpio_set_value(bq->gpio_otg_iusb, 1);

	mutex_lock(&bq->mutex);
	if (bq && bq->shutdown_complete) {
		mutex_unlock(&bq->mutex);
		return -ENODEV;
	}

	/* Clear EN_HIZ */
	ret = regmap_update_bits(bq->chip->regmap,
			BQ2419X_INPUT_SRC_REG, BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq->dev, "error reading reg: 0x%x\n",
			BQ2419X_INPUT_SRC_REG);
		return ret;
	}

	ret = regmap_update_bits(bq->chip->regmap, BQ2419X_OTG,
			BQ2419X_OTG_ENABLE_MASK, BQ2419X_OTG_ENABLE);
	if (ret < 0) {
		dev_err(bq->dev, "register %d update failed with err %d",
			 BQ2419X_OTG, ret);
		mutex_unlock(&bq->mutex);
		return ret;
	}
	mutex_unlock(&bq->mutex);
	return ret;
}

static int bq2419x_dcdc_disable(struct regulator_dev *rdev)
{
	struct bq2419x_regulator_info *bq = rdev_get_drvdata(rdev);
	int ret = 0;

	dev_info(bq->dev, "%s called\n", __func__);
	mutex_lock(&bq->mutex);
	if (bq && bq->shutdown_complete) {
		mutex_unlock(&bq->mutex);
		return -ENODEV;
	}

	ret = regmap_update_bits(bq->chip->regmap, BQ2419X_OTG,
					BQ2419X_OTG_ENABLE_MASK, 0x10);
	if (ret < 0) {
		dev_err(bq->dev, "register %d update failed with err %d",
			BQ2419X_OTG, ret);
		mutex_unlock(&bq->mutex);
		return ret;
	}

	if (gpio_is_valid(bq->gpio_otg_iusb))
		gpio_set_value(bq->gpio_otg_iusb, 0);

	mutex_unlock(&bq->mutex);
	return ret;
}

static int bq2419x_dcdc_is_enabled(struct regulator_dev *rdev)
{
	struct bq2419x_regulator_info *bq = rdev_get_drvdata(rdev);
	int ret;
	unsigned int data;

	mutex_lock(&bq->mutex);
	if (bq && bq->shutdown_complete) {
		mutex_unlock(&bq->mutex);
		return -ENODEV;
	}
	ret = regmap_read(bq->chip->regmap, BQ2419X_OTG, &data);
	if (ret < 0) {
		dev_err(bq->dev, "register %d read failed with err %d",
			BQ2419X_OTG, ret);
		mutex_unlock(&bq->mutex);
		return ret;
	}
	mutex_unlock(&bq->mutex);
	return (data & BQ2419X_OTG_ENABLE_MASK) == BQ2419X_OTG_ENABLE;
}

static struct regulator_ops bq2419x_dcdc_ops = {
	.enable			= bq2419x_dcdc_enable,
	.disable		= bq2419x_dcdc_disable,
	.is_enabled		= bq2419x_dcdc_is_enabled,
	.enable_time		= bq2419x_regulator_enable_time,
};

static void bq2419x_regulator_shutdown(struct platform_device *pdev)
{
	struct bq2419x_regulator_info *bq = platform_get_drvdata(pdev);

	mutex_lock(&bq->mutex);
	bq->shutdown_complete = 1;
	mutex_unlock(&bq->mutex);
}

static int bq2419x_regulator_probe(struct platform_device *pdev)
{
	struct bq2419x_regulator_platform_data *pdata = NULL;
	struct bq2419x_platform_data *chip_pdata;
	struct regulator_dev *rdev;
	struct bq2419x_regulator_info *bq;
	struct regulator_config config = { };
	int ret;

	chip_pdata = dev_get_platdata(pdev->dev.parent);
	if (chip_pdata)
		pdata = chip_pdata->reg_pdata;

	if (!pdata) {
		dev_err(&pdev->dev, "No Platform data");
		return -EIO;
	}

	bq = devm_kzalloc(&pdev->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	bq->dev = &pdev->dev;
	bq->chip = dev_get_drvdata(pdev->dev.parent);

	bq->gpio_otg_iusb = pdata->gpio_otg_iusb;
	bq->power_off_on_suspend = pdata->power_off_on_suspend;
	bq->desc.name = "bq2419x-vbus";
	bq->desc.id = 0;
	bq->desc.ops = &bq2419x_dcdc_ops;
	bq->desc.type = REGULATOR_VOLTAGE;
	bq->desc.owner = THIS_MODULE;
	bq->shutdown_complete = 0;
	mutex_init(&bq->mutex);

	platform_set_drvdata(pdev, bq);

	if (gpio_is_valid(bq->gpio_otg_iusb)) {
		ret = gpio_request(bq->gpio_otg_iusb, dev_name(&pdev->dev));
		if (ret < 0) {
			dev_err(&pdev->dev,
				"gpio request failed, err = %d\n", ret);
			return ret;
		}
		gpio_direction_output(bq->gpio_otg_iusb, 0);
	}

	config.dev = &pdev->dev;
	config.init_data = pdata->reg_init_data;
	config.driver_data = bq;

	/* Register the regulators */
	rdev = regulator_register(&bq->desc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(bq->dev, "regulator register failed, err %d\n", ret);
		goto err_init;
	}

	bq->rdev = rdev;

	ret = regmap_update_bits(bq->chip->regmap, BQ2419X_OTG,
					BQ2419X_OTG_ENABLE_MASK, 0x10);
	if (ret < 0) {
		dev_err(bq->dev, "register %d update failed with err %d",
			BQ2419X_OTG, ret);
		goto err_reg_update;
	}
	return 0;

err_reg_update:
	regulator_unregister(bq->rdev);
err_init:
	if (gpio_is_valid(bq->gpio_otg_iusb))
		gpio_free(bq->gpio_otg_iusb);
	mutex_destroy(&bq->mutex);
	return ret;
}

static int bq2419x_regulator_remove(struct platform_device *pdev)
{
	struct bq2419x_regulator_info *bq = platform_get_drvdata(pdev);

	mutex_destroy(&bq->mutex);
	regulator_unregister(bq->rdev);
	if (gpio_is_valid(bq->gpio_otg_iusb))
		gpio_free(bq->gpio_otg_iusb);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bq2419x_reg_suspend(struct device *dev)
{
	struct bq2419x_regulator_info *bq = dev_get_drvdata(dev);

	if (bq->power_off_on_suspend && bq2419x_dcdc_is_enabled(bq->rdev) > 0)
		bq2419x_dcdc_disable(bq->rdev);
	return 0;
}

static int bq2419x_reg_resume(struct device *dev)
{
	struct bq2419x_regulator_info *bq = dev_get_drvdata(dev);

	/* Turn on regulator that might be turned off by bq2419x_reg_suspend()
	 * and that should be turned on according to the regulators properties.
	 */
	if (bq->power_off_on_suspend &&
		(bq->rdev->use_count > 0 || bq->rdev->constraints->always_on))
		bq2419x_dcdc_enable(bq->rdev);
	return 0;
}

static const struct dev_pm_ops bq2419x_reg_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bq2419x_reg_suspend, bq2419x_reg_resume)
};
#endif

static struct platform_driver bq2419x_regulator_driver = {
	.driver = {
		.name = "bq2419x-pmic",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &bq2419x_reg_pm_ops,
#endif
	},
	.probe = bq2419x_regulator_probe,
	.shutdown = bq2419x_regulator_shutdown,
	.remove = bq2419x_regulator_remove,
};

static int __init bq2419x_init(void)
{
	return platform_driver_register(&bq2419x_regulator_driver);
}
subsys_initcall(bq2419x_init);

static void __exit bq2419x_cleanup(void)
{
	platform_driver_unregister(&bq2419x_regulator_driver);
}
module_exit(bq2419x_cleanup);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("BQ2419X voltage regulator driver");
MODULE_LICENSE("GPL v2");
