/*
 * arch/arm/mach-tegra/board-vcm30_t124.c
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
 */

#include <linux/platform_device.h>
#include <linux/i2c-tegra.h>
#include <linux/i2c.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_nor.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/of_platform.h>
#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>

#include <mach/tegra_vcm30t124_pdata.h>
#include <mach/io_dpd.h>
#include <asm/mach/arch.h>
#include <mach/isomgr.h>
#include <mach/board_id.h>
#include <mach/xusb.h>

#include "iomap.h"
#include "board.h"
#include "clock.h"
#include "board-vcm30_t124.h"
#include "devices.h"
#include "board-common.h"
#include "common.h"
#include "therm-monitor.h"

#include <asm/mach-types.h>
#include "tegra-of-dev-auxdata.h"

static int is_e1860_b00;

#if defined(CONFIG_ANDROID) && defined(CONFIG_BLUEDROID_PM)
static struct resource vcm30_t124_bluedroid_pm_resources[] = {
	[0] = {
		.name	= "reset_gpio",
		.start	= MISCIO_BT_RST_GPIO,
		.end	= MISCIO_BT_RST_GPIO,
		.flags	= IORESOURCE_IO,
	},
	[1] = {
		.name	= "shutdown_gpio",
		.start	= MISCIO_BT_EN_GPIO,
		.end	= MISCIO_BT_EN_GPIO,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device vcm30_t124_bluedroid_pm_device = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(vcm30_t124_bluedroid_pm_resources),
	.resource       = vcm30_t124_bluedroid_pm_resources,
};

static noinline void __init vcm30_t124_setup_bluedroid_pm(void)
{
	platform_device_register(&vcm30_t124_bluedroid_pm_device);
}
#endif

/*
 * Set clock values as per automotive POR
 */
static __initdata struct tegra_clk_init_table vcm30_t124_clk_init_table[] = {
	/* name			parent		rate	enabled (always on)*/

	{ "automotive.sclk",	NULL,		316800000,	true},
	{ "automotive.hclk",	NULL,		316800000,	true},
	{ "automotive.pclk",	NULL,		158400000,	true},

	{ "mselect",		"pll_p",	408000000,	true},
	{ "automotive.mselect",	NULL,		408000000,	true},

	{ "se.cbus",		NULL,		432000000,	false},
	{ "msenc.cbus",		NULL,		432000000,	false},
	{ "vde.cbus",		NULL,		432000000,	false},

	{ "vic03.cbus",		NULL,		660000000,      false},
	{ "tsec.cbus",		NULL,		660000000,      false},

	{ "vi.c4bus",		NULL,		600000000,      false},
	{ "isp.c4bus",		NULL,		600000000,      false},

	{ "pll_d_out0",		"pll_d",	474000000,	true},
	{ "disp2",		"pll_d_out0",	474000000,	false},
	{ "disp1",		"pll_d_out0",	474000000,	false},

	{ "pll_d2",		NULL,		297000000,	true},
	{ "hdmi",		"pll_d2",	297000000,	false},

	{ "pll_a",		"pll_p_out1",	368640000,	true},
	{ "pll_a_out0",		"pll_a",	24576000,	true},

	{ "dam0",		"pll_p",	19900000,	false},
	{ "dam1",		"pll_p",	19900000,	false},
	{ "dam2",		"pll_p",	19900000,	false},
	{ "adx",		"pll_a_out0",	4096000,	false},
	{ "adx1",		"pll_a_out0",	4096000,	false},
	{ "amx",		"pll_a_out0",	4096000,	false},
	{ "amx1",		"pll_a_out0",	4096000,	false},

	{ "spdif_out",		"pll_a_out0",	6144000,	false},
	{ "spdif_in",		"pll_p",	48000000,	false},
	{ "hda",		"pll_p",	48000000,	false},
	{ "cilab",		"pll_p",	102000000,	false},
	{ "cilcd",		"pll_p",	102000000,	false},
	{ "cile",		"pll_p",	102000000,	false},

	{ "nor",		"pll_p",	102000000,	false},

	{ "sbc1",		"pll_p",	25000000,	false},
	{ "sbc2",		"pll_p",	25000000,	false},
	{ "sbc3",		"pll_p",	25000000,	false},
	{ "sbc4",		"pll_p",	25000000,	false},
	{ "sbc5",		"pll_p",	25000000,	false},
	{ "sbc6",		"pll_p",	25000000,	false},

	{ "uarta",		"pll_p",	408000000,	false},
	{ "uartb",		"pll_p",	408000000,	false},
	{ "uartc",		"pll_p",	408000000,	false},
	{ "uartd",		"pll_p",	408000000,	false},

	{ "vi_sensor",		"pll_p",	68000000,	false},
	{ "vi_sensor2",		"pll_p",	68000000,	false},

	{ "automotive.host1x",	NULL,		264000000,	true},

	{ "i2c1",		"pll_p",	408000000,	false},
	{ "i2c2",		"pll_p",	408000000,	false},
	{ "i2c3",		"pll_p",	408000000,	false},
	{ "i2c4",		"pll_p",	408000000,	false},
	{ "i2c5",		"pll_p",	408000000,	false},
	{ "i2c6",		"pll_p",	408000000,	false},

	{ "sdmmc2",		"pll_p",	48000000,	false},

	{ NULL,			NULL,		0,		0},
};

/*
*  Padcontrol registers which need to modified
*  based on local temperature changes.
*  Format ex:
*  {
*    .reg_addr = 0x000008ec, --  only Offset
*    .temperat = {10000, 85000, INT_MAX}, -- in Milli Deg C, Maximum 9 values
*    .value    = {0xf101a000, 0xf161d000}, -- Maximum 9 values
*  },
*
*  If local temperature is  < 10000  Milli Deg C then value "0xf101a000"
*  will be written and if between 10000 to 85000 then value "0xf161d000"
*  will be wriiten and so on.
*/
struct therm_monitor_ldep_data ltemp_reg_data[] = {
	{
		.reg_addr = INVALID_ADDR,
	},
};

/* Platform data for external thermal sensor */
struct therm_monitor_data vcm30t30_therm_monitor_data = {
	.brd_ltemp_reg_data = NULL,
	.delta_temp = 4000,
	.delta_time = 2000,
	.remote_offset = 8000,
	.alert_gpio = TEGRA_GPIO_PI6,
	.local_temp_update = false,
	.utmip_reg_update = false,/* USB registers update is not
						required for now */
	.i2c_bus_num = I2C_BUS_TMP411,
	.i2c_dev_addrs = I2C_ADDR_TMP411,
	.i2c_dev_name = "tmon-tmp411-sensor",
};

#define SET_FIXED_TARGET_RATE(clk_name, fixed_target_rate) \
	{clk_name,	NULL,	fixed_target_rate,	false}

/*
 * FIXME: Need to revisit for following clocks:
 * csi, dsi, dsilp, audio
 */

/*
 * Table of fixed target rates for automotive. parent and enable field are
 * don't care for this table.
 */

static  __initdata struct tegra_clk_init_table
				vcm30t124_fixed_target_clk_table[] = {

	/*			name,		fixed target rate*/
	SET_FIXED_TARGET_RATE("pll_m",		792000000),
	SET_FIXED_TARGET_RATE("sbus",		316800000),

#ifdef CONFIG_TEGRA_PLLCX_FIXED
#ifdef CONFIG_TEGRA_DUAL_CBUS
	SET_FIXED_TARGET_RATE("pll_c2",		432000000),
	SET_FIXED_TARGET_RATE("c2bus",		432000000),
	SET_FIXED_TARGET_RATE("pll_c3",		660000000),
	SET_FIXED_TARGET_RATE("c3bus",		660000000),
#endif
	SET_FIXED_TARGET_RATE("pll_c",		792000000),
	SET_FIXED_TARGET_RATE("pll_c4",		600000000),
	SET_FIXED_TARGET_RATE("c4bus",		600000000),
	SET_FIXED_TARGET_RATE("pll_c_out1",	316800000),
#endif
	SET_FIXED_TARGET_RATE("pll_p",		408000000),
	SET_FIXED_TARGET_RATE("gbus",		600000000),

	SET_FIXED_TARGET_RATE("gk20a.gbus",	600000000),
	SET_FIXED_TARGET_RATE("sclk",		316800000),
	SET_FIXED_TARGET_RATE("hclk",		316800000),
	SET_FIXED_TARGET_RATE("ahb.sclk",	316800000),
	SET_FIXED_TARGET_RATE("pclk",		158400000),
	SET_FIXED_TARGET_RATE("apb.sclk",	158400000),

	SET_FIXED_TARGET_RATE("cpu_lp",		1092000000),

	SET_FIXED_TARGET_RATE("vde",		432000000),
	SET_FIXED_TARGET_RATE("se",		432000000),
	SET_FIXED_TARGET_RATE("msenc",		432000000),

	SET_FIXED_TARGET_RATE("tsec",		660000000),
	SET_FIXED_TARGET_RATE("vic03",		660000000),

	SET_FIXED_TARGET_RATE("vi",		600000000),
	SET_FIXED_TARGET_RATE("isp",		600000000),

	SET_FIXED_TARGET_RATE("host1x",		264000000),
	SET_FIXED_TARGET_RATE("mselect",	408000000),

	SET_FIXED_TARGET_RATE("pll_a_out0",	24576000),
	SET_FIXED_TARGET_RATE("spdif_in",	48000000),
	SET_FIXED_TARGET_RATE("spdif_out",	6144000),
	SET_FIXED_TARGET_RATE("d_audio",	24576000),
	SET_FIXED_TARGET_RATE("adx",		4096000),
	SET_FIXED_TARGET_RATE("adx1",		4096000),
	SET_FIXED_TARGET_RATE("amx",		4096000),
	SET_FIXED_TARGET_RATE("amx1",		4096000),

	SET_FIXED_TARGET_RATE("cilab",		102000000),
	SET_FIXED_TARGET_RATE("cilcd",		102000000),
	SET_FIXED_TARGET_RATE("cile",		102000000),

	SET_FIXED_TARGET_RATE(NULL,		0),
};

static __initdata struct tegra_clk_init_table vcm30t124_a0x_i2s_clk_table[] = {
	{ "i2s4_sync",	NULL,		12288000,	false},
	{ "audio4",	"i2s4_sync",	12288000,	false},
	{ "audio4_2x",	"audio4",	12288000,	false},
	{ "i2s2_sync",	NULL,		12288000,	false},
	{ "audio2",	"i2s2_sync",	12288000,	false},
	{ "audio2_2x",	"audio2",	12288000,	false},
	{ "i2s0",	"pll_a_out0",	3072000,	false},
	{ "i2s1",	"pll_a_out0",	3072000,	false},
	{ "i2s2",	"audio2_2x",	12288000,	false},
	{ "i2s3",	"pll_a_out0",	3072000,	false},
	{ "i2s4",	"audio4_2x",	12288000,	false},
	{ "extern1",	"pll_a_out0",	12288000,	false},
	{ NULL,		NULL,		0,		0},
};

static __initdata struct tegra_clk_init_table vcm30t124_b0x_i2s_clk_table[] = {
	{ "i2s4_sync",	NULL,		12288000,	false},
	{ "audio4",	"i2s4_sync",	12288000,	false},
	{ "audio4_2x",	"audio4",	12288000,	false},
	{ "i2s2_sync",	NULL,		12288000,	false},
	{ "audio2",	"i2s2_sync",	12288000,	false},
	{ "audio2_2x",	"audio2",	12288000,	false},
	{ "i2s0",	"pll_a_out0",	12288000,	false},
	{ "i2s1",	"pll_a_out0",	3072000,	false},
	{ "i2s2",	"audio2_2x",	12288000,	false},
	{ "i2s3",	"pll_a_out0",	3072000,	false},
	{ "i2s4",	"audio4_2x",	12288000,	false},
	{ "extern1",	"pll_a_out0",	24576000,	false},
	{ NULL,		NULL,		0,		0},
};

/* Set POR value for all clocks given in the table */
static void do_set_fixed_target_rate(struct tegra_clk_init_table *table)
{
	struct clk *c;
	unsigned long flags;

	for (; table->name; table++) {
		c = tegra_get_clock_by_name(table->name);
		if (c) {
			clk_lock_save(c, &flags);
			c->fixed_target_rate = table->rate;
			clk_unlock_restore(c, &flags);
		} else {
			pr_warn("%s: Clock %s not found\n", __func__, table->name);
			BUG_ON(1);
		}
	}
}

static int __init tegra_fixed_target_rate_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct tegra_clk_init_table vcm30t124_b0x_i2s4_clk_table[] = {
		{ "i2s4", "pll_a_out0", 1024000, false},
		{ NULL,    NULL,        0,        0},
	};

	do_set_fixed_target_rate(vcm30t124_fixed_target_clk_table);
	do_set_fixed_target_rate(
			(is_e1860_b00) ?
			 vcm30t124_b0x_i2s_clk_table :
			 vcm30t124_a0x_i2s_clk_table);

	/* For voice call in b0x, set i2s4 clock in master mode */
	if (is_e1860_b00 && modem_id)
		do_set_fixed_target_rate(vcm30t124_b0x_i2s4_clk_table);

	return 0;
}


static struct tegra_nor_platform_data vcm30_t124_nor_data = {
	.flash = {
		.map_name = "cfi_probe",
		.width = 2,
	},
	.chip_parms = {
		.MuxMode = NorMuxMode_ADNonMux,
		.ReadMode = NorReadMode_Page,
		.PageLength = NorPageLength_8Word,
		.ReadyActive = NorReadyActive_WithData,
		/* FIXME: Need to use characterized value */
		.timing_default = {
			.timing0 = 0x30300273,
			.timing1 = 0x00030302,
		},
		.timing_read = {
			.timing0 = 0x30300273,
			.timing1 = 0x00030302,
		},
	},
};

static struct cs_info vcm30_t124_cs_info[] = {
	{
		.cs = CS_0,
		.num_cs_gpio = 0,
		.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
		.size = SZ_64M,
		.phys = TEGRA_NOR_FLASH_BASE,
	},
};

static void  __init vcm30_t124_nor_init(void)
{
	tegra_nor_device.resource[2].end = TEGRA_NOR_FLASH_BASE + SZ_64M - 1;

	vcm30_t124_nor_data.info.cs = kzalloc(sizeof(struct cs_info) *
					ARRAY_SIZE(vcm30_t124_cs_info),
					GFP_KERNEL);
        if (!vcm30_t124_nor_data.info.cs)
                BUG();

        vcm30_t124_nor_data.info.num_chips = ARRAY_SIZE(vcm30_t124_cs_info);

        memcpy(vcm30_t124_nor_data.info.cs, vcm30_t124_cs_info,
                                sizeof(struct cs_info) * ARRAY_SIZE(vcm30_t124_cs_info));

	tegra_nor_device.dev.platform_data = &vcm30_t124_nor_data;
	platform_device_register(&tegra_nor_device);
}

static struct i2c_board_info __initdata ak4618_board_info = {
	I2C_BOARD_INFO("ak4618", 0x10),
};

static struct i2c_board_info __initdata wm8731_board_info = {
	I2C_BOARD_INFO("wm8731", 0x1a),
};

static struct i2c_board_info __initdata ad1937_board_info = {
	I2C_BOARD_INFO("ad1937", 0x07),
};

static void __init vcm30_t124_i2c_init(void)
{
	i2c_register_board_info(0, &ak4618_board_info, 1);
	i2c_register_board_info(0, &wm8731_board_info, 1);
	i2c_register_board_info(0, &ad1937_board_info, 1);
}

static struct tegra_serial_platform_data vcm30_t124_uartc_pdata = {
	.dma_req_selector = 10,
	.modem_interrupt = false,
};

static void __init vcm30_t124_uart_init(void)
{
	int debug_port_id;

	tegra_uartc_device.dev.platform_data = &vcm30_t124_uartc_pdata;
	if (!is_tegra_debug_uartport_hs()) {
		debug_port_id = uart_console_debug_init(2);
		if (debug_port_id < 0)
			return;

		platform_device_register(uart_console_debug_device);
	} else {
		tegra_uartc_device.dev.platform_data = &vcm30_t124_uartc_pdata;
		platform_device_register(&tegra_uartc_device);
	}

}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};


#define TDM_SLOT_MAP(stream_id, nth_channel, nth_byte)	\
	((stream_id << 16) | (nth_channel << 8) | nth_byte)

static unsigned int tegra_vcm30t124_amx_slot_map[] = {
	/* jack 0 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* jack 1 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* jack 2 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* jack 3 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
};

static unsigned int tegra_vcm30t124_adx_slot_map[] = {
	/* jack 0 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* jack 1 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* jack 2 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* jack 3 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
};

struct snd_soc_pcm_stream tegra_vcm30t124_amx_input_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[1] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[2] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[3] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
};

struct snd_soc_pcm_stream tegra_vcm30t124_adx_input_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[1] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[2] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[3] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
};

static const struct snd_soc_dapm_route tegra_e1860_a0x_audio_map[] = {
	{"Headphone-y",	NULL,	"y DAC1OUT"},
	{"Headphone-y", NULL,	"y DAC2OUT"},
	{"Headphone-y",	NULL,	"y DAC3OUT"},
	{"Headphone-y", NULL,	"y DAC4OUT"},
	{"y ADC1IN",	NULL,	"LineIn-y"},
	{"y ADC2IN",	NULL,	"LineIn-y"},
	{"Headphone-x",	NULL,	"x ROUT"},
	{"Headphone-x",	NULL,	"x LOUT"},
	{"x LLINEIN",	NULL,	"LineIn-x"},
	{"x RLINEIN",	NULL,	"LineIn-x"},
	{"Spdif-out",	NULL,	"z OUT"},
	{"z IN",	NULL,	"Spdif-in"},
};

static const struct snd_soc_dapm_route tegra_e1860_b00_audio_map[] = {
	{"Headphone-x",	NULL,	"x DACOUT1"},
	{"Headphone-x",	NULL,	"x DACOUT2"},
	{"Headphone-x",	NULL,	"x DACOUT3"},
	{"Headphone-x",	NULL,	"x DACOUT4"},
	{"Headphone-x",	NULL,	"x DACOUT5"},
	{"Headphone-x",	NULL,	"x DACOUT6"},
	{"Mic-x", NULL, "x MICBIAS"},
	{"x IN1",	NULL,	"Mic-x"},
	{"x IN2",	NULL,	"Mic-x"},
	{"x IN3",	NULL,	"Mic-x"},
	{"x IN4",	NULL,	"Mic-x"},
	{"x IN5",	NULL,	"Mic-x"},
	{"x IN6",	NULL,	"Mic-x"},
	{"Headphone-y",	NULL,	"y DAC1OUT"},
	{"Headphone-y", NULL,	"y DAC2OUT"},
	{"Headphone-y",	NULL,	"y DAC3OUT"},
	{"Headphone-y", NULL,	"y DAC4OUT"},
	{"y ADC1IN",	NULL,	"LineIn-y"},
	{"y ADC2IN",	NULL,	"LineIn-y"},
	{"Spdif-out",	NULL,	"z OUT"},
	{"z IN",	NULL,	"Spdif-in"},
};

static const struct snd_soc_dapm_route tegra_voice_call_audio_map[] = {
	{"Headphone-x",	NULL,	"x DACOUT1"},
	{"Headphone-x",	NULL,	"x DACOUT2"},
	{"Headphone-x",	NULL,	"x DACOUT3"},
	{"Headphone-x",	NULL,	"x DACOUT4"},
	{"Headphone-x",	NULL,	"x DACOUT5"},
	{"Headphone-x",	NULL,	"x DACOUT6"},
	{"Mic-x", NULL, "x MICBIAS"},
	{"x IN1",	NULL,	"Mic-x"},
	{"x IN2",	NULL,	"Mic-x"},
	{"x IN3",	NULL,	"Mic-x"},
	{"x IN4",	NULL,	"Mic-x"},
	{"x IN5",	NULL,	"Mic-x"},
	{"x IN6",	NULL,	"Mic-x"},
	{"Spdif-out",	NULL,	"y OUT"},
	{"y IN",	NULL,	"Spdif-in"},
};

static struct tegra_vcm30t124_platform_data tegra_e1860_a0x_pdata = {
	/* initialize DAI link config */
	.dai_config[0] = {
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "wm8731.0-001a",
		.codec_dai_name = "wm8731-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dai_config[1] = {
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "ad193x.0-0007",
		.codec_dai_name = "ad193x-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[2] = {
		.cpu_name = "tegra30-spdif",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "SPDIF",
		.codec_prefix = "z",
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dev_num = 3,
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_vcm30t124_amx_slot_map,
		.params = tegra_vcm30t124_amx_input_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_vcm30t124_amx_slot_map,
		.params = tegra_vcm30t124_amx_input_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_vcm30t124_adx_slot_map,
		.params = tegra_vcm30t124_adx_input_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_vcm30t124_adx_slot_map,
		.params = tegra_vcm30t124_adx_input_params,
	},
	.num_adx = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_e1860_a0x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_e1860_a0x_audio_map),
	/* sound card: tegra-wm8731-ad1937 */
	.card_name = "tegra-wm-ad",
};

static struct tegra_vcm30t124_platform_data tegra_e1860_b00_pdata = {
	/* intialize DAI link config */
	.dai_config[0] = {
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "ak4618.0-0010",
		.codec_dai_name = "ak4618-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[1] = {
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "ad193x.0-0007",
		.codec_dai_name = "ad193x-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[2] = {
		.cpu_name = "tegra30-spdif",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "SPDIF",
		.codec_prefix = "z",
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dev_num = 3,
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_vcm30t124_amx_slot_map,
		.params = tegra_vcm30t124_amx_input_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_vcm30t124_amx_slot_map,
		.params = tegra_vcm30t124_amx_input_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_vcm30t124_adx_slot_map,
		.params = tegra_vcm30t124_adx_input_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_vcm30t124_adx_slot_map,
		.params = tegra_vcm30t124_adx_input_params,
	},
	.num_adx = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_e1860_b00_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_e1860_b00_audio_map),
	/* sound card: tegra-ak4618-ad1937 */
	.card_name = "tegra-ak-ad",
};

/* voice call related structures
 * I2S0<-DAM0<-I2S4 (DAM0 in_rate = 16khz)
 * I2S0->DAM1->I2S4 (DAM1 in_rate = 48khz) */
static unsigned int tegra_voice_call_in_srate[2] = {
	16000, /* DAM0 in_srate */
	48000, /* DAM1 in_srate */
};

static struct tegra_vcm30t124_platform_data tegra_voice_call_pdata = {
	/* initialize DAI link config */
	.dai_config[0] = {
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "ak4618.0-0010",
		.codec_dai_name = "ak4618-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[1] = {
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 16000,
		.params.rate_max = 16000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dev_num = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_voice_call_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_voice_call_audio_map),
	/* initialize DAM input sampling rate */
	.dam_in_srate = tegra_voice_call_in_srate,
	.num_dam = 2,
	/* sound card: tegra-ak4618-voicecall */
	.card_name = "tegra-ak-vc",
};

static struct platform_device tegra_snd_vcm30t124 = {
	.name = "tegra-snd-vcm30t124",
	.id = 0,
};

static struct platform_device tegra_spdif_dit = {
	.name = "spdif-dit",
	.id = 0,
};

static void __init vcm30_t124_audio_init(void)
{
	int modem_id = tegra_get_modem_id();
	int is_e1892 = 0;

	/* check the version of embedded breakout board */
	is_e1892 = tegra_is_board(NULL, "61892", NULL, NULL, NULL);

	/* set max9485 addr as priv data for a0x and b00 */
	tegra_e1860_a0x_pdata.priv_data =
	tegra_e1860_b00_pdata.priv_data =
	tegra_voice_call_pdata.priv_data =
		(void *)(is_e1892 ? 0x70 : 0x60);

	/* initialize the platform data structure */
	tegra_snd_vcm30t124.dev.platform_data = is_e1860_b00 ?
		(modem_id ? &tegra_voice_call_pdata :
			&tegra_e1860_b00_pdata) : &tegra_e1860_a0x_pdata;

	/* register the platform device and dummy codec if any */
	platform_device_register(&tegra_snd_vcm30t124);
	platform_device_register(&tegra_spdif_dit);
}

static struct platform_device *vcm30_t124_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE) && !defined(CONFIG_USE_OF)
	&tegra12_se_device,
#endif
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 63,
		.xcvr_setup_offset = 6,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_use_lsb = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
	.id_det_type = TEGRA_USB_VIRTUAL_ID,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
	},
};


static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
	.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void __init vcm30_t124_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;

#if defined(CONFIG_USB_TEGRA_OTG)
		/* register OTG device */
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);

		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
		platform_device_register(&tegra_udc_device);
#else
		/* register host mode */
		tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;
		platform_device_register(&tegra_ehci1_device);
#endif
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
		platform_device_register(&tegra_ehci2_device);
	}

	if (!(usb_port_owner_info & UTMI3_PORT_OWNER_XUSB)) {
		tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
}

#ifdef CONFIG_USE_OF
struct of_dev_auxdata vcm30_t124_auxdata_lookup[] __initdata = {
	T124_UART_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE, "gk20a.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahub", 0x70300000,
				"tegra30-ahub-apbif", NULL),
	T124_I2C_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci", NULL),

	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra114-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
				NULL),
#ifdef CONFIG_SATA_AHCI_TEGRA
	OF_DEV_AUXDATA("nvidia,tegra114-ahci-sata", TEGRA_SATA_BAR5_BASE, "tegra-sata", NULL),
#endif

#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	OF_DEV_AUXDATA("nvidia,tegra124-se", TEGRA_SE_BASE, "tegra12-se", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC1_BASE, "sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC2_BASE, "sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC3_BASE, "sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC4_BASE, "sdhci-tegra.3", NULL),
	{}
};
#endif

static void __init tegra_vcm30_t124_early_init(void)
{
	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);
	tegra_clk_init_from_table(vcm30_t124_clk_init_table);
	tegra_clk_init_from_table(
			(is_e1860_b00) ?
			vcm30t124_b0x_i2s_clk_table :
			vcm30t124_a0x_i2s_clk_table);

	tegra_clk_verify_parents();
	tegra_fixed_target_rate_init();
	tegra_soc_device_init("vcm30t124");
}

static void __init tegra_vcm30_t124_late_init(void)
{
	tegra_init_board_info();
	vcm30_t124_usb_init();
	vcm30_t124_nor_init();
	vcm30_t124_i2c_init();
	vcm30_t124_uart_init();
	vcm30_t124_pca953x_init();
	vcm30_t124_audio_init();
	platform_add_devices(vcm30_t124_devices,
			ARRAY_SIZE(vcm30_t124_devices));
#ifdef CONFIG_TEGRA_WIFI_ENABLE
	vcm30_t124_sdhci_init();
#endif
	vcm30_t124_regulator_init();
	vcm30_t124_suspend_init();
	isomgr_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	vcm30_t124_panel_init();

#ifdef CONFIG_SENSORS_TMON_TMP411
	register_therm_monitor(&vcm30t30_therm_monitor_data);
#endif

#if defined(CONFIG_ANDROID) && defined(CONFIG_BLUEDROID_PM)
	vcm30_t124_setup_bluedroid_pm();
#endif
}

static void __init tegra_vcm30_t124_dt_init(void)
{
	tegra_vcm30_t124_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, vcm30_t124_auxdata_lookup,
		&platform_bus);
#endif

	tegra_vcm30_t124_late_init();
}

static void __init tegra_vcm30_t124_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve4(0, SZ_16M + SZ_2M, SZ_16M + SZ_2M, 64 * SZ_1M);
#else
	tegra_reserve4(SZ_128M, SZ_16M + SZ_2M, SZ_16M + SZ_2M, 64 * SZ_1M);
#endif
}

static const char * const vcm30_t124_dt_board_compat[] = {
	"nvidia,vcm30t124",
	NULL
};

DT_MACHINE_START(VCM30_T124, "vcm30t124")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
        .init_time      = clocksource_of_init,
	.init_machine	= tegra_vcm30_t124_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= vcm30_t124_dt_board_compat,
        .init_late      = tegra_init_late
MACHINE_END
