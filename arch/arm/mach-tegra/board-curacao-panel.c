/*
 * arch/arm/mach-tegra/board-curacao-panel.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"

#define PANEL_ENABLE		0

#if PANEL_ENABLE

#define curacao_lvds_shutdown	TEGRA_GPIO_PB2
#define curacao_bl_enb		TEGRA_GPIO_PW1

static int curacao_backlight_init(struct device *dev)
{
	int ret;

	ret = gpio_request(curacao_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(curacao_bl_enb, 1);
	if (ret < 0)
		gpio_free(curacao_bl_enb);
	else
		tegra_gpio_enable(curacao_bl_enb);

	return ret;
};

static void curacao_backlight_exit(struct device *dev)
{
	gpio_set_value(curacao_bl_enb, 0);
	gpio_free(curacao_bl_enb);
	tegra_gpio_disable(curacao_bl_enb);
}

static int curacao_backlight_notify(struct device *unused, int brightness)
{
	gpio_set_value(curacao_bl_enb, !!brightness);
	return brightness;
}

static struct platform_pwm_backlight_data curacao_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= curacao_backlight_init,
	.exit		= curacao_backlight_exit,
	.notify		= curacao_backlight_notify,
};

static struct platform_device curacao_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &curacao_backlight_data,
	},
};

static int curacao_panel_enable(void)
{
	static struct regulator *reg;

	if (reg == NULL) {
		reg = regulator_get(NULL, "avdd_lvds");
		if (WARN_ON(IS_ERR(reg)))
			pr_err("%s: couldn't get regulator avdd_lvds: %ld\n",
			       __func__, PTR_ERR(reg));
		else
			regulator_enable(reg);
	}

	gpio_set_value(curacao_lvds_shutdown, 1);
	return 0;
}

static int curacao_panel_disable(void)
{
	gpio_set_value(curacao_lvds_shutdown, 0);
	return 0;
}

static struct resource curacao_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by curacao_panel_init() */
		.end	= 0,	/* Filled in by curacao_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode curacao_panel_modes[] = {
	{
		.pclk = 18000000,
		.h_ref_to_sync = 8,
		.v_ref_to_sync = 2,
		.h_sync_width = 4,
		.v_sync_width = 1,
		.h_back_porch = 20,
		.v_back_porch = 7,
		.h_active = 480,
		.v_active = 640,
		.h_front_porch = 8,
		.v_front_porch = 8,
	},
};

static struct tegra_fb_data curacao_fb_data = {
	.win		= 0,
	.xres		= 480,
	.yres		= 640,
	.bits_per_pixel	= 16,
};

static struct tegra_dc_out curacao_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.modes		= curacao_panel_modes,
	.n_modes	= ARRAY_SIZE(curacao_panel_modes),

	.enable		= curacao_panel_enable,
	.disable	= curacao_panel_disable,
};

static struct tegra_dc_platform_data curacao_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &curacao_disp1_out,
	.fb		= &curacao_fb_data,
};

static struct nvhost_device curacao_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= curacao_disp1_resources,
	.num_resources	= ARRAY_SIZE(curacao_disp1_resources),
	.dev = {
		.platform_data = &curacao_disp1_pdata,
	},
};

static struct nvmap_platform_carveout curacao_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by curacao_panel_init() */
		.size		= 0,	/* Filled in by curacao_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data curacao_nvmap_data = {
	.carveouts	= curacao_carveouts,
	.nr_carveouts	= ARRAY_SIZE(curacao_carveouts),
};

static struct platform_device curacao_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &curacao_nvmap_data,
	},
};

static struct platform_device *curacao_gfx_devices[] __initdata = {
	&curacao_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_pwfm2_device,
	&curacao_backlight_device,
};

int __init curacao_panel_init(void)
{
	int err;
	struct resource *res;

	curacao_carveouts[1].base = tegra_carveout_start;
	curacao_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(curacao_gfx_devices,
				   ARRAY_SIZE(curacao_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&curacao_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	if (!err)
		err = nvhost_device_register(&curacao_disp1_device);
#endif

	return err;
}
#else
int __init curacao_panel_init(void)
{
	return -ENODEV;
}
#endif
