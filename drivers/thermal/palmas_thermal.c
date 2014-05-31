/*
 * palmas_thermal.c -- TI PALMAS THERMAL.
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
 *
 * Author: Pradeep Goudagunta <pgoudagunta@nvidia.com>
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
#include <linux/pm.h>
#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>
#include <linux/of.h>
#include <linux/mfd/palmas.h>

#define PALMAS_NORMAL_OPERATING_TEMP 100000
#define PALMAS_CRITICAL_DEFUALT_TEMP 108000

struct palmas_therm_zone {
	struct device			*dev;
	struct palmas			*palmas;
	struct thermal_zone_device	*tz_device;
	int				irq;
	unsigned long			hd_threshold_temp;
	const char			*tz_name;
};

static struct thermal_trip_info palmas_tpoint = {
	.cdev_type = "emergency-balanced",
	.trip_temp = PALMAS_CRITICAL_DEFUALT_TEMP,
	.trip_type = THERMAL_TRIP_ACTIVE,
	.upper = THERMAL_NO_LIMIT,
	.lower = THERMAL_NO_LIMIT,
};

static int palmas_thermal_get_temp(struct thermal_zone_device *tz_device,
			unsigned long *temp)
{
	int ret;
	unsigned int val;
	struct palmas_therm_zone *ptherm_zone = tz_device->devdata;

	ret = palmas_read(ptherm_zone->palmas, PALMAS_INTERRUPT_BASE,
				PALMAS_INT1_LINE_STATE, &val);
	if (ret < 0) {
		dev_err(ptherm_zone->dev,
			"%s: Failed to read INT1_LINE_STATE, %d\n",
			__func__, ret);
		return -EINVAL;
	}

	if (val & PALMAS_INT1_STATUS_HOTDIE)
		*temp = palmas_tpoint.trip_temp + 1; /* + 1 to trigger cdev */
	else
		*temp = PALMAS_NORMAL_OPERATING_TEMP;

	return 0;
}

static int palmas_thermal_get_trip_type(struct thermal_zone_device *tz_device,
			int trip, enum thermal_trip_type *type)
{
	if (trip >= 1)
		return -EINVAL;

	*type = palmas_tpoint.trip_type;
	return 0;
}

static int palmas_thermal_get_trip_temp(struct thermal_zone_device *tz_device,
			int trip, unsigned long *temp)
{
	if (trip >= 1)
		return -EINVAL;

	*temp = palmas_tpoint.trip_temp;
	return 0;
}

static int palmas_thermal_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	if (!strncmp(palmas_tpoint.cdev_type, cdev->type, THERMAL_NAME_LENGTH))
		thermal_zone_bind_cooling_device(
			thz, 0, cdev, palmas_tpoint.upper, palmas_tpoint.lower);
	return 0;
}

static int palmas_thermal_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	if (!strncmp(palmas_tpoint.cdev_type, cdev->type, THERMAL_NAME_LENGTH))
		thermal_zone_unbind_cooling_device(thz, 0, cdev);
	return 0;
}

static struct thermal_zone_device_ops palmas_tz_ops = {
	.get_temp = palmas_thermal_get_temp,
	.get_trip_type = palmas_thermal_get_trip_type,
	.get_trip_temp = palmas_thermal_get_trip_temp,
	.bind = palmas_thermal_bind,
	.unbind = palmas_thermal_unbind,
};

static irqreturn_t palmas_thermal_irq(int irq, void *data)
{
	struct palmas_therm_zone *ptherm_zone = data;

	thermal_zone_device_update(ptherm_zone->tz_device);
	return IRQ_HANDLED;
}

static int palmas_thermal_probe(struct platform_device *pdev)
{
	struct palmas *palmas = dev_get_drvdata(pdev->dev.parent);
	struct palmas_platform_data *pdata;
	struct palmas_therm_zone *ptherm_zone;
	struct device_node *np = pdev->dev.of_node;
	const char *tz_name = NULL;
	u32 hd_threshold_temp = 0;
	u32 pval;
	int ret;
	u8 val;

	pdata = dev_get_platdata(pdev->dev.parent);
	if (pdata) {
		tz_name = pdata->tz_name;
		hd_threshold_temp = pdata->hd_threshold_temp;
	} else {
		if (np) {
			tz_name = of_get_property(np, "ti,tz-name", NULL);
			ret = of_property_read_u32(np,
					"ti,hot-die-threshold-temp", &pval);
			if (!ret)
				hd_threshold_temp = pval;
		}
	}
	if (!hd_threshold_temp) {
		dev_err(&pdev->dev, "Hot die temp is not provided\n");
		return -EINVAL;
	}

	if (!tz_name)
		tz_name = "palmas-die-thermal";

	ptherm_zone = devm_kzalloc(&pdev->dev, sizeof(*ptherm_zone),
			GFP_KERNEL);
	if (!ptherm_zone) {
		dev_err(&pdev->dev, "No available free memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ptherm_zone);
	ptherm_zone->dev = &pdev->dev;
	ptherm_zone->palmas = palmas;
	ptherm_zone->hd_threshold_temp = (unsigned long) hd_threshold_temp;
	ptherm_zone->tz_name = tz_name;

	ptherm_zone->tz_device = thermal_zone_device_register(tz_name,
					1, 0, ptherm_zone, &palmas_tz_ops,
					NULL, 0, 0);
	if (IS_ERR_OR_NULL(ptherm_zone->tz_device)) {
		dev_err(ptherm_zone->dev,
			"Register thermal zone device failed.\n");
		return PTR_ERR(ptherm_zone->tz_device);
	}

	palmas_tpoint.trip_temp = ptherm_zone->hd_threshold_temp;

	ptherm_zone->irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(ptherm_zone->irq, NULL,
			palmas_thermal_irq,
			IRQF_ONESHOT, dev_name(&pdev->dev),
			ptherm_zone);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"request irq %d failed: %dn", ptherm_zone->irq, ret);
		goto int_req_failed;
	}

	if (palmas_tpoint.trip_temp <= 108000UL)
		val = 0;
	else if (palmas_tpoint.trip_temp <= 112000UL)
		val = 1;
	else if (palmas_tpoint.trip_temp <= 116000UL)
		val = 2;
	else if (palmas_tpoint.trip_temp <= 120000UL)
		val = 3;
	else
		val = 3;

	val <<= PALMAS_OSC_THERM_CTRL_THERM_HD_SEL_SHIFT;
	ret = palmas_update_bits(palmas, PALMAS_PMU_CONTROL_BASE,
			PALMAS_OSC_THERM_CTRL,
			PALMAS_OSC_THERM_CTRL_THERM_HD_SEL_MASK, val);
	if (ret < 0) {
		dev_err(&pdev->dev, "osc_therm_ctrl reg update failed.\n");
		goto error;
	}

	return 0;

error:
	free_irq(ptherm_zone->irq, ptherm_zone);
int_req_failed:
	thermal_zone_device_unregister(ptherm_zone->tz_device);
	return ret;
}

static int palmas_thermal_remove(struct platform_device *pdev)
{
	struct palmas_therm_zone *ptherm_zone = platform_get_drvdata(pdev);

	free_irq(ptherm_zone->irq, ptherm_zone);
	thermal_zone_device_unregister(ptherm_zone->tz_device);
	return 0;
}

static const struct of_device_id palmas_thermal_match[] = {
	{ .compatible = "ti,palmas-thermal", },
	{ },
};
MODULE_DEVICE_TABLE(of, palmas_thermal_match);

static struct platform_driver palmas_thermal_driver = {
	.probe = palmas_thermal_probe,
	.remove = palmas_thermal_remove,
	.driver = {
		.name = "palmas-thermal",
		.owner = THIS_MODULE,
		.of_match_table = palmas_thermal_match,
	},
};

module_platform_driver(palmas_thermal_driver);

MODULE_DESCRIPTION("Palmas Thermal driver");
MODULE_AUTHOR("Pradeep Goudagunta<pgoudagunta@nvidia.com>");
MODULE_ALIAS("platform:palmas-thermal");
MODULE_LICENSE("GPL v2");
