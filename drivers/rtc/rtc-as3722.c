/*
 * rtc-as3722.c - Real Time Clock driver for ams AS3722 PMICs
 *
 * Copyright (C) 2013 ams AG
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>

/* RTC defines:
 * start year has to be a century that rtc works
 * correctly with leap years, etc.
 */
#define AS3722_RTC_START_YEAR          2000
#define AS3722_SET_ALM_RETRIES         5
#define AS3722_SET_TIME_RETRIES        5
#define AS3722_GET_TIME_RETRIES        5

/*
 * Read current time and date in RTC
 */
static int as3722_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	u8 as_time_array[6];
	int ret;

	ret = as3722_block_read(as3722, AS3722_RTC_SECOND_REG,
			6, as_time_array);
	if (ret < 0)
		return ret;

	tm->tm_sec = ((as_time_array[0] & 0xF0) >> 4) * 10
			+ (as_time_array[0] & 0x0F);
	tm->tm_min = ((as_time_array[1] & 0xF0) >> 4) * 10
			+ (as_time_array[1] & 0x0F);
	tm->tm_hour = ((as_time_array[2] & 0xF0) >> 4) * 10
			+ (as_time_array[2] & 0x0F);
	tm->tm_mday = ((as_time_array[3] & 0xF0) >> 4) * 10
			+ (as_time_array[3] & 0x0F);
	tm->tm_mon = ((as_time_array[4] & 0xF0) >> 4) * 10
			+ (as_time_array[4] & 0x0F);
	tm->tm_year = (AS3722_RTC_START_YEAR - 1900)
			+ ((as_time_array[5] & 0xF0) >> 4) * 10
			+ (as_time_array[5] & 0x0F);

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int as3722_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	u8 as_time_array[6];
	int ret;

	/* Write time to RTC */
	as_time_array[0] = ((tm->tm_sec / 10) << 4)
			+ (tm->tm_sec % 10);
	as_time_array[1] = ((tm->tm_min / 10) << 4)
			+ (tm->tm_min % 10);
	as_time_array[2] = ((tm->tm_hour / 10) << 4)
			+ (tm->tm_hour % 10);
	as_time_array[3] = ((tm->tm_mday / 10) << 4)
			+ (tm->tm_mday % 10);
	as_time_array[4] = ((tm->tm_mon / 10) << 4)
			+ (tm->tm_mon % 10);
	if (tm->tm_year >= (AS3722_RTC_START_YEAR - 1900))
		as_time_array[5] = (((tm->tm_year
				- (AS3722_RTC_START_YEAR - 1900)) / 10) << 4)
				+ ((tm->tm_year
				- (AS3722_RTC_START_YEAR - 1900)) % 10);
	else
		return -1;

	ret = as3722_block_write(as3722, AS3722_RTC_SECOND_REG, 6,
			as_time_array);

	return ret;
}

/*
 * Read alarm time and date in RTC
 */
static int as3722_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	u8 as_time_array[6];
	int ret;

	ret = as3722_block_read(as3722, AS3722_RTC_SECOND_REG,
			6, as_time_array);
	if (ret < 0)
		return ret;

	alrm->time.tm_sec = ((as_time_array[0] & 0xF0) >> 4) * 10
			+ (as_time_array[0] & 0x0F);
	alrm->time.tm_min = ((as_time_array[1] & 0xF0) >> 4) * 10
			+ (as_time_array[1] & 0x0F);
	alrm->time.tm_hour = ((as_time_array[2] & 0xF0) >> 4) * 10
			+ (as_time_array[2] & 0x0F);
	alrm->time.tm_mday = ((as_time_array[3] & 0xF0) >> 4) * 10
			+ (as_time_array[3] & 0x0F);
	alrm->time.tm_mon = ((as_time_array[4] & 0xF0) >> 4) * 10
			+ (as_time_array[4] & 0x0F);
	alrm->time.tm_year = (AS3722_RTC_START_YEAR - 1900)
			+ ((as_time_array[5] & 0xF0) >> 4) * 10
			+ (as_time_array[5] & 0x0F);

	return 0;
}

static int as3722_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	u8 as_time_array[6];
	int ret;

	/* Write time to RTC */
	as_time_array[0] = ((alrm->time.tm_sec / 10) << 4)
			+ (alrm->time.tm_sec % 10);
	as_time_array[1] = ((alrm->time.tm_min / 10) << 4)
			+ (alrm->time.tm_min % 10);
	as_time_array[2] = ((alrm->time.tm_hour / 10) << 4)
			+ (alrm->time.tm_hour % 10);
	as_time_array[3] = ((alrm->time.tm_mday / 10) << 4)
			+ (alrm->time.tm_mday % 10);
	as_time_array[4] = ((alrm->time.tm_mon / 10) << 4)
			+ (alrm->time.tm_mon % 10);
	if (alrm->time.tm_year >= (AS3722_RTC_START_YEAR - 1900))
		as_time_array[5] = (((alrm->time.tm_year
				- (AS3722_RTC_START_YEAR - 1900)) / 10) << 4)
				+ ((alrm->time.tm_year
				- (AS3722_RTC_START_YEAR - 1900)) % 10);
	else
		return -1;

	ret = as3722_block_write(as3722, AS3722_RTC_SECOND_REG, 6,
			as_time_array);

	return ret;
}

static int as3722_rtc_stop_alarm(struct as3722 *as3722)
{
	/* disable rtc alarm interrupt */
	return as3722_set_bits(as3722, AS3722_INTERRUPTMASK3_REG,
			AS3722_IRQ_MASK_RTC_ALARM, AS3722_IRQ_BIT_RTC_ALARM);
}

static int as3722_rtc_start_alarm(struct as3722 *as3722)
{
	/* enable rtc alarm interrupt */
	return as3722_set_bits(as3722, AS3722_INTERRUPTMASK3_REG,
			AS3722_IRQ_MASK_RTC_ALARM, 0);
}

static int as3722_rtc_alarm_irq_enable(struct device *dev,
		unsigned int enabled)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);

	if (enabled)
		return as3722_rtc_start_alarm(as3722);
	else
		return as3722_rtc_stop_alarm(as3722);
}

static irqreturn_t as3722_alarm_irq(int irq, void *data)
{
	struct as3722 *as3722 = data;
	struct rtc_device *rtc = as3722->rtc.rtc;

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops as3722_rtc_ops = {
	.read_time = as3722_rtc_readtime,
	.set_time = as3722_rtc_settime,
	.read_alarm = as3722_rtc_readalarm,
	.set_alarm = as3722_rtc_setalarm,
	.alarm_irq_enable = as3722_rtc_alarm_irq_enable,
};

#ifdef CONFIG_PM
static int as3722_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;
	u32 reg;

	as3722_reg_read(as3722, AS3722_INTERRUPTMASK3_REG, &reg);

	if (device_may_wakeup(dev) &&
			reg & AS3722_IRQ_MASK_RTC_ALARM) {
		ret = as3722_rtc_stop_alarm(as3722);
		if (ret != 0)
			dev_err(dev, "Failed to stop RTC alarm: %d\n",
					ret);
	}

	return ret;
}

static int as3722_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	int ret;

	if (as3722->rtc.alarm_enabled) {
		ret = as3722_rtc_start_alarm(as3722);
		if (ret != 0)
			dev_err(dev,
				"Failed to restart RTC alarm: %d\n", ret);
	}

	return 0;
}

/* Unconditionally disable the alarm */
static int as3722_rtc_freeze(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	int ret;

	ret = as3722_set_bits(as3722, AS3722_RTC_CONTROL_REG,
			AS3722_RTC_ALARM_WAKEUP_EN_MASK, 0);
	if (ret != 0)
		dev_err(&pdev->dev, "Failed to stop RTC alarm: %d\n", ret);

	return 0;
}

#define DEV_PM_OPS     (&as3722_rtc_pm_ops)
#else
#define DEV_PM_OPS     NULL
#endif

static int as3722_rtc_probe(struct platform_device *pdev)
{
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	struct as3722_rtc *rtc = &as3722->rtc;

	int alarm_irq = regmap_irq_get_virq(as3722->irq_data,
						AS3722_IRQ_RTC_ALARM);
	int ret = 0;
	u32 ctrl;

	/* enable the RTC if it's not already enabled */
	as3722_reg_read(as3722, AS3722_RTC_CONTROL_REG, &ctrl);
	if (!(ctrl &  AS3722_RTC_ON_MASK)) {
		dev_info(&pdev->dev, "Starting RTC\n");

		ret = as3722_set_bits(as3722, AS3722_RTC_CONTROL_REG,
				AS3722_RTC_ON_MASK, AS3722_RTC_ON_MASK);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to enable RTC: %d\n", ret);
			return ret;
		}
	}

	/* enable alarm wakeup */
	as3722_set_bits(as3722, AS3722_RTC_CONTROL_REG,
			AS3722_RTC_ALARM_WAKEUP_EN_MASK,
			AS3722_RTC_ALARM_WAKEUP_EN_MASK);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = rtc_device_register("as3722", &pdev->dev,
			&as3722_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err(&pdev->dev, "failed to register RTC: %d\n", ret);
		return ret;
	}

	ret = request_threaded_irq(alarm_irq, NULL, as3722_alarm_irq,
			IRQF_ONESHOT | IRQF_EARLY_RESUME, "RTC alarm",
			rtc->rtc);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request alarm IRQ %d: %d\n",
				alarm_irq, ret);
	}

	return 0;
}

static int as3722_rtc_remove(struct platform_device *pdev)
{
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	struct as3722_rtc *rtc = &as3722->rtc;
	int alarm_irq = regmap_irq_get_virq(as3722->irq_data,
						AS3722_IRQ_RTC_ALARM);

	free_irq(alarm_irq, rtc->rtc);
	rtc_device_unregister(rtc->rtc);

	return 0;
}

static const struct dev_pm_ops as3722_rtc_pm_ops = {
	.suspend        = as3722_rtc_suspend,
	.resume         = as3722_rtc_resume,
	.freeze         = as3722_rtc_freeze,
	.thaw           = as3722_rtc_resume,
	.restore        = as3722_rtc_resume,
	.poweroff       = as3722_rtc_suspend,
};

static struct platform_driver as3722_rtc_driver = {
	.probe = as3722_rtc_probe,
	.remove = as3722_rtc_remove,
	.driver = {
		.name = "as3722-rtc",
		.pm = DEV_PM_OPS,
	},
};

module_platform_driver(as3722_rtc_driver);

MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
MODULE_DESCRIPTION("RTC driver for AS3722 PMICs");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:as3722-rtc");
