/*
 * arch/arm/mach-tegra/board-vcm30_t124-sdhci.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/board_id.h>
#include <linux/i2c.h>

#include "iomap.h"
#include "gpio-names.h"
#include "board.h"
#include "board-vcm30_t124.h"
#include "devices.h"

static void (*wifi_status_cb) (int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int
vcm30_t124_wifi_status_register(void (*callback) (int, void *), void *);
static int vcm30_t124_wifi_reset(int on);
static int vcm30_t124_wifi_power(int on);
static int vcm30_t124_wifi_set_carddetect(int val);

static struct wifi_platform_data vcm30_t124_wifi_control = {
	.set_power = vcm30_t124_wifi_power,
	.set_reset = vcm30_t124_wifi_reset,
	.set_carddetect = vcm30_t124_wifi_set_carddetect,
};

static struct platform_device broadcom_wifi_device = {
	.name = "bcm4329_wlan",
	.id = 1,
	.dev = {
		.platform_data = &vcm30_t124_wifi_control,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data1 = {
	.cccr = {
		.sdio_vsn = 2,
		.multi_block = 1,
		.low_speed = 0,
		.wide_bus = 0,
		.high_power = 1,
		.high_speed = 1,
	},
	.cis = {
		.vendor = 0x02d0,
		.device = 0x4329,
	},
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data1 = {
	.mmc_data = {
		.register_status_notify = vcm30_t124_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data1,
#endif
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x5,
	.trim_delay = 0x2,
	.is_8bit = false,
	.ddr_clk_limit = 30000000,
	.max_clk_limit = 50000000,
	/*
	 * FIXME: Wifi currently working on HS mode.
	 * enable SDR104, DDR50 and SDR50 mode for wifi using
	 * bug 1457466
	 */
	.uhs_mask = MMC_UHS_MASK_SDR104 |
		MMC_UHS_MASK_DDR50 | MMC_UHS_MASK_SDR50,
	.calib_3v3_offsets = 0x7676,
	.calib_1v8_offsets = 0x7676,
	.disable_clock_gate = 1,
};
static int vcm30_t124_wifi_status_register(
			void (*callback) (int card_present, void *dev_id),
			void *dev_id)
{
	if (wifi_status_cb)
		return -EBUSY;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int vcm30_t124_wifi_set_carddetect(int val)
{

	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warn("%s: Nobody to notify\n", __func__);

	return 0;
}

static int vcm30_t124_wifi_power(int on)
{
	int ret;
	/* Deassert WiFi RST GPIO */
	ret = gpio_request(MISCIO_WF_EN_GPIO, "wifi_en");
	if (ret < 0)
		goto fail;
	gpio_direction_output(MISCIO_WF_EN_GPIO, on);
	gpio_free(MISCIO_WF_EN_GPIO);

	ret = gpio_request(MISCIO_WF_RST_GPIO, "wifi_rst");
	if (ret < 0)
		goto fail;
	gpio_direction_output(MISCIO_WF_RST_GPIO, on);
	gpio_free(MISCIO_WF_RST_GPIO);

	mdelay(100);
	return 0;
fail:
	printk(KERN_ERR "%s: gpio_request failed(%d)\r\n", __func__, ret);
	return ret;
}
static int vcm30_t124_wifi_reset(int on)
{
	/*
	 * FIXME: Implement wifi reset
	 */
	return 0;
}

int __init vcm30_t124_wifi_init(void)
{
	platform_device_register(&broadcom_wifi_device);
	return 0;
}

int __init vcm30_t124_sdhci_init(void)
{
	tegra_sdhci_device1.dev.platform_data = &tegra_sdhci_platform_data1;

	platform_device_register(&tegra_sdhci_device1);
	vcm30_t124_wifi_init();

	return 0;
}
