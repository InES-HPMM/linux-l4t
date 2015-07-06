/*
 * arch/arm/mach-tegra/vcm30_t124.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This file is intended to hold all code related to vcm30_t124 MCM
 *
 * Broadly, it contains Clock POR values, internal flash and temparature
 * sensor etc
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/platform_data/tegra_nor.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/tegra-pmc.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra_soctherm.h>
#include <linux/irqchip/tegra.h>
#include <asm/io.h>
#include "board.h"
#include "board-common.h"
#include <linux/platform/tegra/clock.h>
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "tegra-of-dev-auxdata.h"
#include "vcm30_t124.h"
#include <asm/mach/arch.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/platform/tegra/common.h>
#include <linux/thermal.h>
#include <linux/platform/tegra/tegra12_emc.h>
#include <linux/platform_data/thermal_sensors.h>
#include <linux/platform_data/tmon_tmp411.h>
#include <linux/i2c.h>

#define MAX_NUM_TEMPERAT 10
#define INVALID_ADDR 0xffffffff
#define NUM_OF_TRIPS (sizeof(trip) / sizeof(struct thermal_trip_info))

struct therm_monitor_ldep_data {
	unsigned int reg_addr;
	unsigned int value[MAX_NUM_TEMPERAT - 1];
};

struct therm_monitor_data {
	unsigned int remote_offset;
	signed int alert_gpio;
	unsigned char i2c_bus_num;
	unsigned int i2c_dev_addrs;
	unsigned char *i2c_dev_name;
	unsigned char i2c_board_size;
	struct thermal_trip_info *trips;
	unsigned int num_trips;
	struct thermal_zone_params *tzp;
	int polling_delay;
	int passive_delay;
};

struct dram_data {
	unsigned long cur_state;
	unsigned long max_state;
};

struct dram_data data;

static struct tmon_plat_data tmon_pdata;
struct i2c_board_info __initdata tgr_i2c_board_info[1];
/*
 * Set clock values as per automotive POR for VCM30T124
 *
 * If the clock POR values are different for a board, eg for i2s, those
 * would be hanled in the board files
 *
 * Use this table to initialize set of clocks during boot up with specified
 * rate. If enabled, they will remain ON throughout.
 *
 * Add clocks here if they've attributes different from the ones in common
 * clock table (mach-tegra/common.c). Parent=NULL implies the parent will not
 * be changed by this table; its previous value will be retained.
 * "rate" specifies only the initial rate for this clock.
 *
 * System busses should have clock always on, while individual controllers
 * clocks can be enabled/disabled by the controller drivers.
 */
static __initdata struct tegra_clk_init_table vcm30_t124_clk_init_table[] = {
	/* name			parent		rate	enabled (always on)*/

	{ "automotive.sclk",	NULL,		282000000,	true},
	{ "automotive.hclk",	NULL,		282000000,	true},
	{ "automotive.pclk",	NULL,		282000000,	true},

	{ "pll_p_out1",		"pll_p",	9600000,	true},
	{ "pll_p_out2",		"pll_p",	48000000,	true},
	{ "pll_p_out3",		"pll_p",	102000000,	true},
	{ "pll_p_out4",		"pll_p",	204000000,	true},
	{ "pll_p_out5",		"pll_p",	204000000,	true},

	{ "mselect",		"pll_p",	204000000,	true},
	{ "automotive.mselect",	NULL,		204000000,	true},

	{ "se.cbus",		NULL,		372000000,	false},
	{ "msenc.cbus",		NULL,		372000000,	false},
	{ "vde",		"pll_c3",	450000000,	false},

	{ "vic03.cbus",		NULL,		564000000,      false},
	{ "tsec.cbus",		NULL,		564000000,      false},
	{ "tsec",		"pll_c",	564000000,	false},
	{ "vic03",		"pll_c",	564000000,	false},

	{ "vi.c4bus",		NULL,		600000000,      false},
	{ "isp.c4bus",		NULL,		600000000,      false},
	{ "vi",			"pll_c4",	600000000,	false},
	{ "isp",		"pll_c4",	600000000,	false},

	{ "pll_d_out0",		"pll_d",	474000000,	true},
	{ "disp2",		"pll_d_out0",	474000000,	false},
	{ "disp1",		"pll_d_out0",	474000000,	false},

	{ "pll_d2",		NULL,		297000000,	true},
	{ "hdmi",		"pll_d2",	297000000,	false},

	{ "pll_a",		"pll_p_out1",	368640000,	true},
	{ "pll_a_out0",		"pll_a",	24576000,	true},

	{ "dam0",		"pll_p",	40000000,	false},
	{ "dam1",		"pll_p",	40000000,	false},
	{ "dam2",		"pll_p",	40000000,	false},
	{ "adx",		"pll_p",	24000000,	false},
	{ "adx1",		"pll_p",	24000000,	false},
	{ "amx",		"pll_p",	24000000,	false},
	{ "amx1",		"pll_p",	24000000,	false},
	{ "d_audio",		"pll_p",	48000000,	false},

	{ "spdif_out",		"pll_a_out0",	6144000,	false},
	{ "spdif_in",		"pll_p",	48000000,	false},
	{ "hda",		"pll_p",	108000000,	false},
	{ "hda2codec_2x",	"pll_p",	48000000,	false},
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

	{ "automotive.host1x",	NULL,		282000000,	true},

	{ "sata_oob",		"pll_p",	204000000,	false},
	{ "sata",		"pll_p",	204000000,	false},

	{ "i2c1",		"pll_p",	408000000,	false},
	{ "i2c2",		"pll_p",	408000000,	false},
	{ "i2c3",		"pll_p",	408000000,	false},
	{ "i2c4",		"pll_p",	408000000,	false},
	{ "i2c5",		"pll_p",	102000000,	false},
	{ "i2c6",		"pll_p",	408000000,	false},

	{ "sdmmc2",		"pll_p",	48000000,	false},
	{ "gk20a.gbus",		NULL,		600000000,	false},

	{ NULL,			NULL,		0,		0},
};

#define SET_FIXED_TARGET_RATE(clk_name, fixed_target_rate) \
	{clk_name,	NULL,	fixed_target_rate,	false}

/*
 * FIXME: Need to revisit for following clocks:
 * csi, dsi, dsilp, audio
 */

/*
 * Table of clocks that have fixed, characterized rates. Entries in this table
 * are orthagonal to the above table, but the "rate" set in the above table
 * will be un-used if a clock is made FIXED via this table.
 *
 * Parent and enable fields cannot be set thru' this table.
 */

static  __initdata struct tegra_clk_init_table
				vcm30t124_fixed_target_clk_table[] = {

	/*			name,		fixed target rate*/
	SET_FIXED_TARGET_RATE("pll_m",		792000000),
	SET_FIXED_TARGET_RATE("sbus",		282000000),

#ifdef CONFIG_TEGRA_PLLCX_FIXED
#ifdef CONFIG_TEGRA_DUAL_CBUS
	SET_FIXED_TARGET_RATE("pll_c2",		372000000),
	SET_FIXED_TARGET_RATE("c2bus",		372000000),
	SET_FIXED_TARGET_RATE("pll_c3",		450000000),
	SET_FIXED_TARGET_RATE("c3bus",		450000000),
#endif
	SET_FIXED_TARGET_RATE("pll_c",		564000000),
	SET_FIXED_TARGET_RATE("pll_c4",		600000000),
	SET_FIXED_TARGET_RATE("c4bus",		600000000),
	SET_FIXED_TARGET_RATE("pll_c_out1",	282000000),
#endif
	SET_FIXED_TARGET_RATE("pll_p",		408000000),

	SET_FIXED_TARGET_RATE("sata_oob",	204000000),
	SET_FIXED_TARGET_RATE("sata",		204000000),

	SET_FIXED_TARGET_RATE("sclk",		282000000),
	SET_FIXED_TARGET_RATE("hclk",		282000000),
	SET_FIXED_TARGET_RATE("ahb.sclk",	282000000),
	SET_FIXED_TARGET_RATE("pclk",		282000000),
	SET_FIXED_TARGET_RATE("apb.sclk",	282000000),
	SET_FIXED_TARGET_RATE("cpu_lp",		948000000),

	SET_FIXED_TARGET_RATE("vde",		450000000),
	SET_FIXED_TARGET_RATE("se",		372000000),
	SET_FIXED_TARGET_RATE("msenc",		372000000),

	SET_FIXED_TARGET_RATE("tsec",		564000000),
	SET_FIXED_TARGET_RATE("vic03",		564000000),

	SET_FIXED_TARGET_RATE("vi",		600000000),
	SET_FIXED_TARGET_RATE("isp",		600000000),

	SET_FIXED_TARGET_RATE("host1x",		282000000),
	SET_FIXED_TARGET_RATE("mselect",	204000000),

	SET_FIXED_TARGET_RATE("pll_a_out0",	24576000),
	SET_FIXED_TARGET_RATE("spdif_in",	48000000),
	SET_FIXED_TARGET_RATE("spdif_out",	6144000),
	SET_FIXED_TARGET_RATE("d_audio",	48000000),
	SET_FIXED_TARGET_RATE("adx",		24000000),
	SET_FIXED_TARGET_RATE("adx1",		24000000),
	SET_FIXED_TARGET_RATE("amx",		24000000),
	SET_FIXED_TARGET_RATE("amx1",		24000000),

	SET_FIXED_TARGET_RATE("cilab",		102000000),
	SET_FIXED_TARGET_RATE("cilcd",		102000000),
	SET_FIXED_TARGET_RATE("cile",		102000000),

	SET_FIXED_TARGET_RATE(NULL,		0),
};

/* Therm Monitor */

/*
 *  registers which need to modified
 *  based on local temperature changes.
 *  Format ex:
 *  {
 *    .reg_addr = 0x000008ec,
 *    .value    = {0xf101a000, 0xf161d000}, -- Maximum 9 values
 *  },
 *
 */
struct therm_monitor_ldep_data dram_cdev_reg_data[] = {
	{
		.reg_addr = 0x7001b070,
		.value = {0xbd1, 0x000005d9},
	},
	{
		.reg_addr = 0x7001b3e0,
		.value = {0x8000188b, 0x80000cc7},
	},
	{
		.reg_addr = 0x7001b0a8,
		.value = {0x00000c11, 0x00000609},
	},
	{
		.reg_addr = 0x7001b3dc,
		.value = {0x000002f4, 0x00000176},
	},
	{
		.reg_addr = 0x7001b028,
		.value = {0x1, 0x1},
	},
	{
		.reg_addr = INVALID_ADDR,
	},
};

/*
 * trip type active is used because there is a need to change
 * DRAM registers setting based on trip temperature and hyst.
 *
 * IF THERMAL_NO_LIMIT is chosen then thermal framework sets
 * lower state to 0 and upper to max state of cdev, DRAM have
 * lower state 0 and upper 1 so it is ok to use THERMAL_NO_LIMIT
 * If lower and upper limits are different for any trip point
 * then use those limits instead of THERMAL_NO_LIMIT
 *
 * trip_temp, trip_type, upper state, lower state, hyst,
 * tripped, mask, bound, cooling device name
 *
 * mask makes trip point writable through debugfs entries if set to 1
 */
struct thermal_trip_info trip[] = {
	{90000, THERMAL_TRIP_ACTIVE, THERMAL_NO_LIMIT, THERMAL_NO_LIMIT, 10000,
		false, 0x0, false, "dram_cdev"},
};

static int get_dram_cdev_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	struct dram_data *data = cdev->devdata;
	*max_state = data->max_state;

	return 0;
}

static int get_dram_cdev_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	struct dram_data *data = cdev->devdata;
	*cur_state = data->cur_state;

	return 0;
}

static int set_dram_cdev_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	struct dram_data *data = cdev->devdata;
	static int call_at_boot;

	if (data->cur_state != cur_state || call_at_boot == 0) {
		int i;
		call_at_boot = 1;
		for (i = 0; dram_cdev_reg_data[i].reg_addr
						!= INVALID_ADDR; i++) {
			void *reg_addr = ioremap(dram_cdev_reg_data[i].reg_addr,
									4);
			writel(dram_cdev_reg_data[i].value[cur_state],
				reg_addr);
			iounmap(reg_addr);
		}

		data->cur_state = cur_state;
	}

	return 0;
}

struct thermal_cooling_device_ops dram_cdev_ops = {
	.get_max_state = get_dram_cdev_max_state,
	.get_cur_state = get_dram_cdev_cur_state,
	.set_cur_state = set_dram_cdev_state,
};

/*
 * DRAM have only one trip point so two states (two settings of
 * registers values ,default and another when temperature crosses
 * trip point)
 * since in kernel thermal framework min state can be 0 so in
 * case of DRAM max state is 1 ,since it has only two states.
 */
int __init dram_cdev_init(void)
{
	static struct thermal_cooling_device *cdev;
	char name[] = "dram_cdev";
	data.max_state = 1;
	cdev = thermal_cooling_device_register(name,
				&data, &dram_cdev_ops);
	if (IS_ERR_OR_NULL(cdev)) {
		pr_err("Cooling device: %s not registered\n",
			name);
		return -EFAULT;
	}

	return 0;
}

late_initcall(dram_cdev_init);

/*
 * step_wise governor is used for DRAM cooling device.
 *
 * governor_params is applicable in case of pid_thermal_gov
 *
 * not enabling thermal to hwmon sysfs interface if want to
 * enable set no_hwmon = false
 *
 * tbp is applicable in case of fairshair governor therefore
 * it is null
 */
struct thermal_zone_params tzp = {
	.governor_name = "step_wise",
	.governor_params = NULL,
	.no_hwmon = true,
	.tbp = NULL,
};

#ifdef CONFIG_SENSORS_TMON_TMP411
static struct therm_monitor_data vcm30t124_therm_monitor_data = {
	.remote_offset = 8000,
	.alert_gpio = TEGRA_GPIO_PI6,

	.i2c_bus_num = I2C_BUS_TMP411,
	.i2c_dev_addrs = I2C_ADDR_TMP411,
	.i2c_dev_name = "tmon-tmp411-sensor",
	.trips = trip,
	.num_trips = NUM_OF_TRIPS,
	.tzp = &tzp,
	.passive_delay = 2000,
	.polling_delay = 2000,
};
#endif

void register_therm_monitor(struct therm_monitor_data *brd_therm_monitor_data)
{

	/* Thermal monitor operational parameters */
	tmon_pdata.remote_offset = brd_therm_monitor_data->remote_offset;
	tmon_pdata.alert_gpio = brd_therm_monitor_data->alert_gpio;

	tmon_pdata.trips = brd_therm_monitor_data->trips;
	tmon_pdata.num_trips = brd_therm_monitor_data->num_trips;
	tmon_pdata.tzp = brd_therm_monitor_data->tzp;
	tmon_pdata.polling_delay = brd_therm_monitor_data->polling_delay;
	tmon_pdata.passive_delay = brd_therm_monitor_data->passive_delay;

	/* Fill the i2c board info */
	strcpy(tgr_i2c_board_info[0].type,
		brd_therm_monitor_data->i2c_dev_name);
	tgr_i2c_board_info[0].addr = brd_therm_monitor_data->i2c_dev_addrs;
	tgr_i2c_board_info[0].platform_data = &tmon_pdata;

	i2c_register_board_info(brd_therm_monitor_data->i2c_bus_num,
			tgr_i2c_board_info, 1);
}

void __init tegra_vcm30_t124_therm_mon_init(void)
{
#ifdef CONFIG_SENSORS_TMON_TMP411
	register_therm_monitor(&vcm30t124_therm_monitor_data);
#endif
}

/* wakeup IDs corresponding to XHCI usage*/
#define USB3_UTMIP_WAKEUP 41
#define XUSB_PADCTL_WAKEUP 58
void __init tegra_vcm30_t124_usb_init(void)
{
	/*
	 * lp0 has auto-resume issues when XHCI wakeup source is allowed in
	 * the platform.  Undeclare it to avoid autoresume.
	 */
	tegra_disable_wake_source(USB3_UTMIP_WAKEUP);
	tegra_disable_wake_source(XUSB_PADCTL_WAKEUP);
}

#ifdef CONFIG_USE_OF
/*
 * This table is used to populate the device name which is necessary to
 * make the clock framework to work.
 * eg the dt device name "nvidia,tegra124-vi" is modified to "vi.0" for
 * clk_get and clk_enable to work properly
 */
static struct of_dev_auxdata tegra_vcm30_t124_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#if defined(CONFIG_USB_TEGRA_OTG)
	OF_DEV_AUXDATA("nvidia,tegra124-udc", TEGRA_USB_BASE, "tegra-udc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-otg", TEGRA_USB_BASE, "tegra-otg", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-ehci", TEGRA_USB_BASE, "tegra-ehci.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ehci", TEGRA_USB2_BASE, "tegra-ehci.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ehci", TEGRA_USB3_BASE, "tegra-ehci.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci", NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-nvavp", 0x60001000, "nvavp", NULL),
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	OF_DEV_AUXDATA("nvidia,tegra124-se", TEGRA_SE_BASE, "tegra12-se", NULL),
#endif

	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
			"gk20a.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahub", 0x70300000, "tegra30-ahub-apbif",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-alt-pcm", 0,
			"tegra124-virt-alt-pcm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-xbar-control", 0,
			"tegra124-virt-xbar-control", NULL),
	OF_DEV_AUXDATA("nvidia,tegra-audio-vcm30t124", 0x70300000,
				"tegra-snd-vcm30t124", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
			NULL),
#ifdef CONFIG_SATA_AHCI_TEGRA
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", TEGRA_SATA_BAR5_BASE,
			"tegra-sata", NULL),
#endif

	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC1_BASE,
			"sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC2_BASE,
			"sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC3_BASE,
			"sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC4_BASE,
			"sdhci-tegra.3", NULL),

	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nor", TEGRA_SNOR_BASE, "tegra-nor",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
		 NULL),
	{}
};

void __init tegra_vcm30_t124_populate_auxdata(void)
{
	of_platform_populate(NULL, of_default_bus_match_table,
			tegra_vcm30_t124_auxdata_lookup, &platform_bus);
}
#endif

/* Soc Therm */
static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,
	.gain_p = 1000,
	.gain_d = 0,
	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct soctherm_platform_data vcm30_t124_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
};

static const struct soctherm_therm vcm30t124_therm[] = {
};

int __init tegra_vcm30_t124_soctherm_init(void)
{
	struct soctherm_therm *therm;

	therm = &vcm30_t124_soctherm_data.therm[THERM_CPU];
	tegra_add_cpu_clk_switch_trips(therm->trips, &therm->num_trips);

	therm = &vcm30_t124_soctherm_data.therm[THERM_GPU];
	tegra_add_tgpu_trips(therm->trips, &therm->num_trips);

	return tegra_soctherm_init(&vcm30_t124_soctherm_data);
}

/* Set POR value for all clocks given in the table */
void __init tegra_vcm30_t124_set_fixed_rate(struct tegra_clk_init_table *table)
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

int __init tegra_vcm30_t124_early_init(void)
{
	tegra_clk_init_from_table(vcm30_t124_clk_init_table);
	tegra_vcm30_t124_set_fixed_rate(vcm30t124_fixed_target_clk_table);

	return 0;
}

void __init tegra_vcm30_t124_reserve(void)
{
	/* 2560*1600*4*2 = 32768000 = 31.250*1M = 32M bytes */
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve4(0, 32 * SZ_1M, 32 * SZ_1M, 64 * SZ_1M);
#else
	tegra_reserve4(SZ_256M, 32 * SZ_1M, 32 * SZ_1M, 64 * SZ_1M);
#endif
}

/* VCM30T124 suspend init */
static struct tegra_suspend_platform_data vcm30_t124_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 2000,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0xfefe,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
};

int __init tegra_vcm30_t124_suspend_init(void)
{
	tegra_init_suspend(&vcm30_t124_suspend_data);
	return 0;
}

static void __init tegra_vcm30t124_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();
	tegra_vcm30_t124_suspend_init();
}

static void __init tegra_vcm30t124_dt_init(void)
{
	tegra_vcm30_t124_early_init();
#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_vcm30t124_late_init();
}

static const char * const vcm30t124_dt_compat[] = {
	"nvidia,vcm30t124",
	NULL
};

/*
 * vcm30t124 is VCM and not a board in itself.
 * A machine struct has been defined for vcm30t124
 * only to provide mechanism to initialize whatever
 * minimal is required for boards with this vcm to
 * boot.
 */
DT_MACHINE_START(VCM30T124, "vcm30t124")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= NULL,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_vcm30t124_dt_init,
	.dt_compat	= vcm30t124_dt_compat,
	.init_late	= tegra_init_late
MACHINE_END
