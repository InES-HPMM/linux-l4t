/*
 * arch/arm/mach-tegra/board-pluto-sensors.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/max77665-charger.h>
#include <linux/mfd/max77665.h>
#include <linux/input/max77665-haptic.h>
#include <linux/power/max17042_battery.h>
#include <linux/nct1008.h>
#include <linux/interrupt.h>
#include <mach/edp.h>
#include <linux/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/max77665-flash.h>
#ifndef CONFIG_OF
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ad5816.h>
#endif
#include <asm/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-pluto.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

#define NTC_10K_TGAIN   0xE6A2
#define NTC_10K_TOFF    0x2694
#define MAX77665_CHARGER_INT	TEGRA_GPIO_PJ2

#ifndef CONFIG_OF
static struct nvc_gpio_pdata imx091_gpio_pdata[] = {
	{IMX091_GPIO_RESET, CAM_RSTN, true, false},
	{IMX091_GPIO_PWDN, CAM1_POWER_DWN_GPIO, true, false},
	{IMX091_GPIO_GP1, CAM_GPIO1, true, false}
};
#endif

static struct board_info board_info;
static struct max17042_config_data conf_data = {
	.valrt_thresh = 0xff00,
	.talrt_thresh = 0xff00,
	.soc_alrt_thresh = 0xff00,
	.shdntimer = 0xe000,
	.design_cap = 0x07d0,
	.at_rate = 0x0000,
	.tgain = NTC_10K_TGAIN,
	.toff = NTC_10K_TOFF,
	.vempty = 0xACDA,
	.qrtbl00 = 0x5C80,
	.qrtbl10 = 0x438C,
	.qrtbl20 = 0x1198,
	.qrtbl30 = 0x0E19,
	.full_soc_thresh = 0x5A00,
	.rcomp0 = 0x0077,
	.tcompc0 = 0x1F2A,
	.ichgt_term = 0x0320,
	.temp_nom = 0x1400,
	.temp_lim = 0x2305,
	.filter_cfg = 0x87A4,
	.config = 0x2210,
	.learn_cfg = 0x2606,
	.misc_cfg = 0x0810,
	.fullcap =  0x07d0,
	.fullcapnom = 0x07d0,
	.lavg_empty = 0x1000,
	.dqacc = 0x01f4,
	.dpacc = 0x3200,
	.fctc = 0x05e0,
	.kempty0 = 0x0600,
	.cell_technology = POWER_SUPPLY_TECHNOLOGY_LION,
	.cell_char_tbl = {
		/* Data to be written from 0x80h */
		0x9180, 0xA4C0, 0xB6A0, 0xB760, 0xB980, 0xBB30,
		0xBBC0, 0xBC50, 0xBD50, 0xBE50, 0xBF80, 0xC290,
		0xC470, 0xC7D0, 0xCC40, 0xCFB0,
		/* Data to be written from 0x90h */
		0x00C0, 0x0200, 0x1C10, 0x0B00, 0x0900, 0x1F00,
		0x1F00, 0x23C0, 0x1990, 0x19F0, 0x09A0, 0x0CE0,
		0x0BE0, 0x07D0, 0x0990, 0x0990,
		/* Data to be written from 0xA0h */
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100,
	},
};

static unsigned int bat_depl_states[] = {
	5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000,
	3800, 3600, 3400, 3200, 3000, 2800, 2600, 2400,
	2200, 2000, 1900, 1800, 1700, 1600, 1500, 1400,
	1300, 1200, 1100, 1000,  900,  800,  700,  600,
	 500,  400,  300,  200,  100,    0
};

static struct edp_client bat_depl_client = {
	.states = bat_depl_states,
	.num_states = ARRAY_SIZE(bat_depl_states),
	.e0_index = 0,
	.priority = EDP_MAX_PRIO
};

static struct max17042_platform_data max17042_pdata = {
	.config_data = &conf_data,
	.init_data  = NULL,
	.num_init_data = 0,
	.enable_por_init = 1, /* Use POR init from Maxim appnote */
	.enable_current_sense = 1,
	.r_sns = 0,
	.edp_client = &bat_depl_client
};

static struct i2c_board_info max17042_device[] = {
	{
		I2C_BOARD_INFO("max17042", 0x36),
		.platform_data = &max17042_pdata,
	},
};

static struct nvc_torch_lumi_level_v1 pluto_max77665_lumi_tbl[] = {
	{0, 100000},
	{1, 201690},
	{2, 298080},
	{3, 387700},
	{4, 479050},
	{5, 562000},
	{6, 652560},
	{7, 732150},
	{8, 816050},
	{9, 896710},
	{10, 976890},
	{11, 1070160},
	{12, 1151000},
	{13, 1227790},
	{14, 1287690},
	{15, 1375060},
};

static unsigned max77665_f_estates[] = { 3500, 2375, 560, 456, 0 };

static struct max77665_f_platform_data pluto_max77665_flash_pdata = {
	.config		= {
		.led_mask		= 3,
		/* set to true only when using the torch strobe input
		 * to trigger the flash.
		 */
		.flash_on_torch         = false,
		/* use ONE-SHOOT flash mode - flash triggered at the
		 * raising edge of strobe or strobe signal.
		 */
		.flash_mode		= 1,
		/* .flash_on_torch         = true, */
		.max_total_current_mA	= 1000,
		.max_peak_current_mA	= 600,
		.max_flash_threshold_mV	= 3400,
		.max_flash_hysteresis_mV = 200,
		.max_flash_lbdly_f_uS	= 256,
		.max_flash_lbdly_r_uS	= 256,
		.led_config[0] = {
			.flash_torch_ratio = 18100,
			.granularity = 1000,
			.flash_levels = ARRAY_SIZE(pluto_max77665_lumi_tbl),
			.lumi_levels = pluto_max77665_lumi_tbl,
			},
		.led_config[1] = {
			.flash_torch_ratio = 18100,
			.granularity = 1000,
			.flash_levels = ARRAY_SIZE(pluto_max77665_lumi_tbl),
			.lumi_levels = pluto_max77665_lumi_tbl,
			},
		},
	.pinstate	= {
		.mask	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		.values	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		},
	.dev_name	= "torch",
	.gpio_strobe	= CAM_FLASH_STROBE,
	.edpc_config	= {
		.states = max77665_f_estates,
		.num_states = ARRAY_SIZE(max77665_f_estates),
		.e0_index = ARRAY_SIZE(max77665_f_estates) - 1,
		.priority = EDP_MAX_PRIO + 2,
		},
};

static struct max77665_haptic_platform_data max77665_haptic_pdata = {
	.pwm_channel_id = 2,
	.pwm_period = 50,
	.type = MAX77665_HAPTIC_LRA,
	.mode = MAX77665_INTERNAL_MODE,
	.internal_mode_pattern = 0,
	.pattern_cycle = 10,
	.pattern_signal_period = 0xD0,
	.pwm_divisor = MAX77665_PWM_DIVISOR_128,
	.feedback_duty_cycle = 12,
	.invert = MAX77665_INVERT_OFF,
	.cont_mode = MAX77665_CONT_MODE,
	.motor_startup_val = 0,
	.scf_val = 2,
	.edp_states = { 360, 0 },
};

static struct max77665_charger_cable maxim_cable[] = {
	{
		.name           = "USB",
	},
	{
		.name           = "USB-Host",
	},
	{
		.name           = "TA",
	},
	{
		.name           = "Fast-charger",
	},
	{
		.name           = "Slow-charger",
	},
	{
		.name           = "Charge-downstream",
	},
};

static struct max77665_charger_plat_data max77665_charger = {
	.fast_chg_cc = 1500, /* fast charger current*/
	.term_volt = 3700, /* charger termination voltage */
	.curr_lim = 1500, /* input current limit */
	.num_cables = MAX_CABLES,
	.cables = maxim_cable,
	.irq_base = MAX77665_TEGRA_IRQ_BASE,
	.update_status = max17042_update_status,
};

static struct max77665_muic_platform_data max77665_muic = {
	.irq_base = 0,
};

static struct max77665_platform_data pluto_max77665_pdata = {
	.irq_base = MAX77665_TEGRA_IRQ_BASE,
	.irq_flag = IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
	.muic_platform_data = {
		.pdata = &max77665_muic,
		.size =	sizeof(max77665_muic),
		},
	.charger_platform_data = {
		.pdata = &max77665_charger,
		.size =	sizeof(max77665_charger),
		},
	.flash_platform_data = {
		.pdata = &pluto_max77665_flash_pdata,
		.size =	sizeof(pluto_max77665_flash_pdata),
		},
	.haptic_platform_data = {
		.pdata = &max77665_haptic_pdata,
		.size = sizeof(max77665_haptic_pdata),
		},
};

static struct i2c_board_info pluto_i2c_board_info_max77665[] = {
	{
		I2C_BOARD_INFO("max77665", 0x66),
		.platform_data = &pluto_max77665_pdata,
	},
};

/* isl29029 support is provided by isl29028*/
static struct i2c_board_info pluto_i2c1_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init pluto_throttle_init(void)
{
	if (machine_is_tegra_pluto())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(pluto_throttle_init);

static struct nct1008_platform_data pluto_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 0,
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 120, /* C */

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

static struct i2c_board_info pluto_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &pluto_nct1008_pdata,
		.irq = -1,
	}
};

#ifndef CONFIG_OF
#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
}

static int pluto_focuser_power_on(struct ad5816_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto ad5816_vdd_i2c_fail;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto ad5816_vdd_fail;

	return 0;

ad5816_vdd_fail:
	regulator_disable(pw->vdd_i2c);

ad5816_vdd_i2c_fail:
	pr_err("%s FAILED\n", __func__);

	return -ENODEV;
}

static int pluto_focuser_power_off(struct ad5816_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 0;
}

static struct tegra_pingroup_config mclk_disable =
	VI_PINMUX(CAM_MCLK, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_disable =
	VI_PINMUX(GPIO_PBB0, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

/*
 * more regulators need to be allocated to activate the sensor devices.
 * pluto_vcmvdd: this is a workaround due to the focuser device(AD5816) will
 *               hook up the i2c bus if it is not powered up.
 * pluto_i2cvdd: by default, the power supply on the i2c bus is OFF. So it
 *               should be turned on every time any sensor device is activated.
*/
static struct regulator *pluto_vcmvdd;
static struct regulator *pluto_i2cvdd;

static int pluto_get_extra_regulators(void)
{
	if (!pluto_vcmvdd) {
		pluto_vcmvdd = regulator_get(NULL, "vdd_af_cam1");
		if (WARN_ON(IS_ERR(pluto_vcmvdd))) {
			pr_err("%s: can't get regulator vdd_af_cam1: %ld\n",
				__func__, PTR_ERR(pluto_vcmvdd));
			pluto_vcmvdd = NULL;
			return -ENODEV;
		}
	}

	if (!pluto_i2cvdd) {
		pluto_i2cvdd = regulator_get(NULL, "vddio_cam_mb");
		if (unlikely(WARN_ON(IS_ERR(pluto_i2cvdd)))) {
			pr_err("%s: can't get regulator vddio_cam_mb: %ld\n",
				__func__, PTR_ERR(pluto_i2cvdd));
			pluto_i2cvdd = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

static int pluto_imx091_power_on(struct nvc_regulator *vreg)
{
	int err;

	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	if (pluto_get_extra_regulators())
		goto imx091_poweron_fail;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(vreg[IMX091_VREG_AVDD].vreg);
	if (unlikely(err))
		goto imx091_avdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_DVDD].vreg);
	if (unlikely(err))
		goto imx091_dvdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_IOVDD].vreg);
	if (unlikely(err))
		goto imx091_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	tegra_pinmux_config_table(&mclk_enable, 1);
	err = regulator_enable(pluto_i2cvdd);
	if (unlikely(err))
		goto imx091_i2c_fail;

	err = regulator_enable(pluto_vcmvdd);
	if (unlikely(err))
		goto imx091_vcm_fail;
	usleep_range(300, 310);

	return 1;

imx091_vcm_fail:
	regulator_disable(pluto_i2cvdd);

imx091_i2c_fail:
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	regulator_disable(vreg[IMX091_VREG_IOVDD].vreg);

imx091_iovdd_fail:
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);

imx091_dvdd_fail:
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);

imx091_avdd_fail:
imx091_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int pluto_imx091_power_off(struct nvc_regulator *vreg)
{
	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	usleep_range(1, 2);
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(vreg[IMX091_VREG_IOVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);
	regulator_disable(pluto_i2cvdd);
	regulator_disable(pluto_vcmvdd);

	return 0;
}

static struct nvc_imager_cap imx091_cap = {
	.identifier		= "IMX091",
	.sensor_nvc_interface	= 3,
	.pixel_types[0]		= 0x100,
	.orientation		= 0,
	.direction		= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 850000, /* value / 1,000,000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge		= 0,
	.v_sync_edge		= 0,
	.mclk_on_vgp0		= 0,
	.csi_port		= 0,
	.data_lanes		= 4,
	.virtual_channel_id	= 0,
	.discontinuous_clk_mode	= 1,
	.cil_threshold_settle	= 0x0,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid		= NVC_FOCUS_GUID(0),
	.torch_guid		= NVC_TORCH_GUID(0),
	.cap_version		= NVC_IMAGER_CAPABILITIES_VERSION2,
};

static unsigned imx091_estates[] = { 876, 656, 220, 0 };

static struct imx091_platform_data imx091_pdata = {
	.num			= 0,
	.sync			= 0,
	.dev_name		= "camera",
	.gpio_count		= ARRAY_SIZE(imx091_gpio_pdata),
	.gpio			= imx091_gpio_pdata,
	.flash_cap		= {
		.sdo_trigger_enabled = 1,
		.adjustable_flash_timing = 1,
	},
	.cap			= &imx091_cap,
	.edpc_config	= {
		.states = imx091_estates,
		.num_states = ARRAY_SIZE(imx091_estates),
		.e0_index = 0,
		.priority = EDP_MAX_PRIO + 1,
		},
	.power_on		= pluto_imx091_power_on,
	.power_off		= pluto_imx091_power_off,
};

static int pluto_imx132_power_on(struct imx132_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	if (pluto_get_extra_regulators())
		goto pluto_imx132_poweron_fail;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	tegra_pinmux_config_table(&pbb0_enable, 1);

	err = regulator_enable(pluto_i2cvdd);
	if (unlikely(err))
		goto imx132_i2c_fail;

	err = regulator_enable(pluto_vcmvdd);
	if (unlikely(err))
		goto imx132_vcm_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx132_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx132_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx132_iovdd_fail;

	usleep_range(1, 2);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);

	return 0;

imx132_iovdd_fail:
	regulator_disable(pw->dvdd);

imx132_dvdd_fail:
	regulator_disable(pw->avdd);

imx132_avdd_fail:
	regulator_disable(pluto_vcmvdd);

imx132_vcm_fail:
	regulator_disable(pluto_i2cvdd);

imx132_i2c_fail:
	tegra_pinmux_config_table(&pbb0_disable, 1);

pluto_imx132_poweron_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int pluto_imx132_power_off(struct imx132_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd ||
			!pluto_i2cvdd || !pluto_vcmvdd)))
		return -EFAULT;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	tegra_pinmux_config_table(&pbb0_disable, 1);

	regulator_disable(pluto_vcmvdd);
	regulator_disable(pluto_i2cvdd);

	return 0;
}

struct imx132_platform_data imx132_pdata = {
	.power_on = pluto_imx132_power_on,
	.power_off = pluto_imx132_power_off,
};

static struct ad5816_platform_data pluto_ad5816_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "focuser",
	.power_on	= pluto_focuser_power_on,
	.power_off	= pluto_focuser_power_off,
};

static struct i2c_board_info pluto_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x10),
		.platform_data = &imx091_pdata,
	},
	{
		I2C_BOARD_INFO("imx132", 0x36),
		.platform_data = &imx132_pdata,
	},
	{
		I2C_BOARD_INFO("ad5816", 0x0E),
		.platform_data = &pluto_ad5816_pdata,
	},
};

static int pluto_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	tegra_pinmux_config_table(&mclk_disable, 1);
	tegra_pinmux_config_table(&pbb0_disable, 1);
	i2c_register_board_info(2, pluto_i2c_board_info_e1625,
		ARRAY_SIZE(pluto_i2c_board_info_e1625));

	return 0;
}
#endif

/* MPU board file definition */
static struct mpu_platform_data mpu_gyro_data = {
	.int_config	= 0x00,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id	= COMPASS_ID_AK8963,
	.secondary_i2c_addr	= MPU_COMPASS_ADDR,
	.secondary_orientation	= MPU_COMPASS_ORIENTATION,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct i2c_board_info __initdata inv_mpu_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	int i = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

	/* MPU-IRQ assignment */
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu_i2c0_board_info[i++].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
#if MPU_COMPASS_IRQ_GPIO
	inv_mpu_i2c0_board_info[i++].irq = gpio_to_irq(MPU_COMPASS_IRQ_GPIO);
#endif
	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c0_board_info,
		ARRAY_SIZE(inv_mpu_i2c0_board_info));
}

static int pluto_nct1008_init(void)
{
	int nct1008_port;
	int ret = 0;

	if (board_info.board_id == BOARD_E1580 ||
	    board_info.board_id == BOARD_E1575 ||
	    board_info.board_id == BOARD_E1577) {
		nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert port assumed TEGRA_GPIO_PX6 for unknown pluto board id E%d\n",
			board_info.board_id);
	}

	tegra_add_cdev_trips(pluto_nct1008_pdata.trips,
				&pluto_nct1008_pdata.num_trips);

	pluto_i2c4_nct1008_board_info[0].irq =
		gpio_to_irq(nct1008_port);
	pr_info("%s: pluto nct1008 irq %d",
			__func__, pluto_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* pluto has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, pluto_i2c4_nct1008_board_info,
		ARRAY_SIZE(pluto_i2c4_nct1008_board_info));

	return ret;
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int tegra_skin_match(struct thermal_zone_device *thz, void *data)
{
	return strcmp((char *)data, thz->type) == 0;
}

static int tegra_skin_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, tegra_skin_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct therm_est_data skin_data = {
	.cdev_type = "skin-balanced",
	.toffset = 9793,
	.polling_period = 1100,
	.ndevs = 2,
	.tc1 = 10,
	.tc2 = 1,
	.devs = {
			{
				.dev_data = "nct_ext",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					2, 1, 1, 1,
					1, 1, 1, 1,
					1, 1, 1, 0,
					1, 1, 0, 0,
					0, 0, -1, -7
				},
			},
			{
				.dev_data = "nct_int",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					-11, -7, -5, -3,
					-3, -2, -1, 0,
					0, 0, 1, 1,
					1, 2, 2, 3,
					4, 6, 11, 18
				},
			},
	},
	.trip_temp = 45000,
	.passive_delay = 15000,
};

static struct throttle_table skin_throttle_table[] = {
		/* CPU_THROT_LOW cannot be used by other than CPU */
		/* NO_CAP cannot be used by CPU */
		/*    CPU,   C2BUS,   C3BUS,    SCLK,     EMC */
		{ { 1530000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1530000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1326000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1122000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ { 1020000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  918000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  816000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  714000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  NO_CAP,  NO_CAP,  NO_CAP,  NO_CAP } },
		{ {  612000,  564000,  564000,  NO_CAP,  NO_CAP } },
		{ {  612000,  564000,  564000,  NO_CAP,  NO_CAP } },
		{ {  612000,  528000,  528000,  NO_CAP,  NO_CAP } },
		{ {  612000,  528000,  528000,  NO_CAP,  NO_CAP } },
		{ {  612000,  492000,  492000,  NO_CAP,  NO_CAP } },
		{ {  612000,  492000,  492000,  NO_CAP,  NO_CAP } },
		{ {  612000,  420000,  420000,  NO_CAP,  NO_CAP } },
		{ {  612000,  420000,  420000,  NO_CAP,  NO_CAP } },
		{ {  612000,  408000,  408000,  NO_CAP,  NO_CAP } },
		{ {  612000,  408000,  408000,  NO_CAP,  NO_CAP } },
		{ {  612000,  360000,  360000,  NO_CAP,  NO_CAP } },
		{ {  612000,  360000,  360000,  NO_CAP,  NO_CAP } },
		{ {  510000,  360000,  360000,  312000,  NO_CAP } },
		{ {  510000,  360000,  360000,  312000,  NO_CAP } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  510000,  360000,  360000,  312000,  480000 } },
		{ {  468000,  360000,  360000,  312000,  480000 } },
		{ {  468000,  360000,  360000,  312000,  480000 } },
		{ {  468000,  276000,  276000,  208000,  480000 } },
		{ {  468000,  276000,  276000,  208000,  480000 } },
		{ {  372000,  276000,  276000,  208000,  204000 } },
		{ {  372000,  276000,  276000,  208000,  204000 } },
		{ {  288000,  276000,  276000,  208000,  204000 } },
		{ {  288000,  276000,  276000,  208000,  204000 } },
		{ {  252000,  276000,  228000,  208000,  102000 } },
		{ {  252000,  276000,  228000,  208000,  102000 } },
		{ {  204000,  276000,  228000,  208000,  102000 } },
		{ {  204000,  276000,  228000,  208000,  102000 } },
		{ {  102000,  276000,  228000,  208000,  102000 } },
	  { { CPU_THROT_LOW,  276000,  228000,  208000,  102000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init pluto_skin_init(void)
{
	if (machine_is_tegra_pluto()) {
		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(pluto_skin_init);
#endif

void __init max77665_init(void)
{
	int err;

	err = gpio_request(MAX77665_CHARGER_INT, "CHARGER_INT");
	if (err < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, err);
		goto fail_init_irq;
	}

	err = gpio_direction_input(MAX77665_CHARGER_INT);
	if (err < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, err);
		goto fail_init_irq;
	}

	pluto_i2c_board_info_max77665[0].irq =
				gpio_to_irq(MAX77665_CHARGER_INT);
fail_init_irq:
	err = i2c_register_board_info(4, pluto_i2c_board_info_max77665,
		ARRAY_SIZE(pluto_i2c_board_info_max77665));
	if (err)
		pr_err("%s: max77665 device register failed.\n", __func__);

	return;
}

int __init pluto_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	pr_debug("%s: ++\n", __func__);

#ifndef CONFIG_OF
	pluto_camera_init();
#endif
	err = pluto_nct1008_init();
	if (err)
		return err;

	err = i2c_register_board_info(0, pluto_i2c1_isl_board_info,
				ARRAY_SIZE(pluto_i2c1_isl_board_info));
	if (err)
		pr_err("%s: isl board register failed.\n", __func__);

	mpuirq_init();
	max77665_init();
	pluto_i2c_board_info_max77665[0].irq = gpio_to_irq(TEGRA_GPIO_PJ0);

	err = i2c_register_board_info(0, max17042_device,
				ARRAY_SIZE(max17042_device));
	if (err)
		pr_err("%s: max17042 device register failed.\n", __func__);


	return 0;
}
