/*
 * arch/arm/mach-tegra/tegra21_init.c
 *
 * NVIDIA Tegra210 initialization support
 *
 * Copyright (C) 2013-2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/tegra-fuse.h>
#include <mach/powergate.h>

#include "reset.h"
#include "apbio.h"
#include "clock.h"
#include "dvfs.h"
#include "common.h"
#include "devices.h"

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)

/* TODO: check  the correct init values */

static __initdata struct tegra_clk_init_table tegra21x_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "clk_m",      NULL,           0,              true },
	{ "emc",        NULL,           0,              true },
	{ "cpu",        NULL,           0,              true },
	{ "kfuse",      NULL,           0,              true },
	{ "fuse",       NULL,           0,              true },
	{ "sclk",       NULL,           0,              true },
	{ "pll_p",      NULL,           0,              true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pll_p_out1", "pll_p",        0,              false,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pll_p_out3", "pll_p",        0,              true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pll_p_out2", "pll_p",        102000000,      false,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "sclk",       "pll_p_out2",   102000000,      true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pll_p_out4", "pll_p",        204000000,      true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "host1x",     "pll_p",        102000000,      false,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "cl_dvfs_ref", "pll_p",       54000000,       true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "cl_dvfs_soc", "pll_p",       54000000,       true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "hclk",       "sclk",         102000000,      true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pclk",       "hclk",         51000000,       true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "wake.sclk",  NULL,           40000000,       true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "mselect",    "pll_p",        102000000,      true,
		TEGRA_CLK_INIT_PLATFORM_SI },
	{ "pll_p_out5", "pll_p",        102000000,      true,
		TEGRA_CLK_INIT_PLATFORM_SI },
#ifdef CONFIG_TEGRA_PRE_SILICON_SUPPORT
	{ "pll_p_out2", "pll_p",        108000000,      false,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "sclk",       "pll_p_out2",   108000000,      true,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "pll_p_out4", "pll_p",        216000000,      true,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "hclk",       "sclk",         108000000,      true,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "pclk",       "hclk",         54000000,       true,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "mselect",    "pll_p",        108000000,      true,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "host1x",     "pll_p",        108000000,      false,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "cl_dvfs_ref", "clk_m",       13000000,       false,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
	{ "cl_dvfs_soc", "clk_m",       13000000,       false,
		TEGRA_CLK_INIT_PLATFORM_NON_SI },
#endif
#ifdef CONFIG_TEGRA_SLOW_CSITE
	{ "csite",      "clk_m",        1000000,        true },
#else
	{ "csite",      NULL,           0,              true },
#endif
	{ "pll_u",      NULL,           480000000,      true },
	{ "pll_re_vco", NULL,           672000000,      false },
	{ "xusb_falcon_src",    "pll_re_out",   224000000,      false},
	{ "xusb_host_src",      "pll_re_out",   112000000,      false},
	{ "xusb_ss_src",        "pll_u_480M",   120000000,      false},
	{ "xusb_hs_src",        "pll_u_60M",    60000000,       false},
	{ "xusb_fs_src",        "pll_u_48M",    48000000,       false},
	{ "sdmmc1",     "pll_p",        48000000,       false},
	{ "sdmmc3",     "pll_p",        48000000,       false},
	{ "sdmmc4",     "pll_p",        48000000,       false},
	{ "sbc1.sclk",  NULL,           40000000,       false},
	{ "sbc2.sclk",  NULL,           40000000,       false},
	{ "sbc3.sclk",  NULL,           40000000,       false},
	{ "sbc4.sclk",  NULL,           40000000,       false},
	{ "gpu",        NULL,           0,              true},
	{ "mc_capa",        "mc",           0,              true},
	{ "mc_cbpa",        "mc",           0,              true},
	{ "mc_ccpa",        "mc",           0,              true},
	{ "mc_cdpa",        "mc",           0,              true},
#ifdef CONFIG_TEGRA_PLLM_SCALED
	{ "vi",         "pll_p",        0,              false},
	{ "isp",        "pll_p",        0,              false},
#endif
#ifdef CONFIG_TEGRA_SOCTHERM
	{ "soc_therm",  "pll_p",        51000000,       false },
	{ "tsensor",    "clk_m",        500000,         false },
#endif
	{ "csite",      NULL,           0,              true },
	{ "uartb",	NULL,		0,		true },
	{ NULL,         NULL,           0,              0},

};

static __initdata struct tegra_clk_init_table tegra21x_cbus_init_table[] = {
	/* Initialize c2bus, c3bus, or cbus at the end of the list
	   * after all the clocks are moved under the proper parents.
	*/
#ifdef CONFIG_TEGRA_DUAL_CBUS
	{ "c2bus",      "pll_c2",       250000000,      false },
	{ "c3bus",      "pll_c3",       250000000,      false },
	{ "pll_c",      NULL,           792000000,      false },
#else
	{ "cbus",       "pll_c",        200000000,      false },
#endif
	{ "pll_c_out1", "pll_c",        100000000,      false },
	{ "c4bus",      "pll_c4",       200000000,      false },
	{ NULL,         NULL,           0,              0},
};
static void __init tegra_perf_init(void)
{
	u32 reg;

#ifdef CONFIG_ARM64
	asm volatile("mrs %0, PMCR_EL0" : "=r"(reg));
	reg >>= 11;
	reg = (1 << (reg & 0x1f))-1;
	reg |= 0x80000000;
	asm volatile("msr PMINTENCLR_EL1, %0" : : "r"(reg));
	reg = 1;
	asm volatile("msr PMUSERENR_EL0, %0" : : "r"(reg));
#endif
}

static void __init tegra_init_power(void)
{
   /* TODO : Do the required power initilizations here */
}

static void __init tegra_init_ahb_gizmo_settings(void)
{
   /* TODO : Set the required gizimo settings here */
}

void __init tegra21x_init_early(void)
{
#ifndef CONFIG_SMP
	/* For SMP system, initializing the reset handler here is too
	   late. For non-SMP systems, the function that calls the reset
	   handler initializer is not called, so do it here for non-SMP. */
	tegra_cpu_reset_handler_init();
#endif
	tegra_apb_io_init();
	tegra_perf_init();
	tegra_init_fuse();
	tegra21x_init_clocks();
	tegra21x_init_dvfs();
	tegra_common_init_clock();
	tegra_clk_init_from_table(tegra21x_clk_init_table);
	tegra_clk_init_cbus_plls_from_table(tegra21x_cbus_init_table);
	tegra_powergate_init();
	tegra_init_power();
	tegra_init_ahb_gizmo_settings();
	tegra_init_debug_uart_rate();
}
#endif
