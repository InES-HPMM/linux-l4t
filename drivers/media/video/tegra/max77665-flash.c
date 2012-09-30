/*
 * MAX77665_F.c - MAX77665_F flash/torch kernel driver
 *
 * Copyright (C) 2012 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

/* Implementation
 * --------------
 * The board level details about the device need to be provided in the board
 * file with the max77665_platform_data structure.
 * Standard among NVC kernel drivers in this structure is:
 * .cfg = Use the NVC_CFG_ defines that are in nvc_torch.h.
 *        Descriptions of the configuration options are with the defines.
 *        This value is typically 0.
 * .num = The number of the instance of the device.  This should start at 1 and
 *        and increment for each device on the board.  This number will be
 *        appended to the MISC driver name, Example: /dev/torch.1
 * .sync = If there is a need to synchronize two devices, then this value is
 *         the number of the device instance this device is allowed to sync to.
 *         This is typically used for stereo applications.
 * .dev_name = The MISC driver name the device registers as.  If not used,
 *             then the part number of the device is used for the driver name.
 *             If using the NVC user driver then use the name found in this
 *             driver under _default_pdata.
 *
 * The following is specific to NVC kernel flash/torch drivers:
 * .pinstate = a pointer to the nvc_torch_pin_state structure.  This
 *             structure gives the details of which VI GPIO to use to trigger
 *             the flash.  The mask tells which pin and the values is the
 *             level.  For example, if VI GPIO pin 6 is used, then
 *             .mask = 0x0040
 *             .values = 0x0040
 *             If VI GPIO pin 0 is used, then
 *             .mask = 0x0001
 *             .values = 0x0001
 *             This is typically just one pin but there is some legacy
 *             here that insinuates more than one pin can be used.
 *             When the flash level is set, then the driver will return the
 *             value in values.  When the flash level is off, the driver will
 *             return 0 for the values to deassert the signal.
 *             If a VI GPIO is not used, then the mask and values must be set
 *             to 0.  The flash may then be triggered via I2C instead.
 *             However, a VI GPIO is strongly encouraged since it allows
 *             tighter timing with the picture taken as well as reduced power
 *             by asserting the trigger signal for only when needed.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <media/nvc.h>
#include <linux/mfd/max77665.h>
#include <media/max77665-flash.h>

#define MAX77665_F_RW_FLASH_FLED1CURR		0x00
#define MAX77665_F_RW_FLASH_FLED2CURR		0x01
#define MAX77665_F_RW_TORCH_FLEDCURR		0x02
#define MAX77665_F_RW_TORCH_TIMER		0x03
#define MAX77665_F_RW_FLASH_TIMER		0x04
#define MAX77665_F_RW_FLED_ENABLE		0x05
#define MAX77665_F_RW_MAXFLASH_VOLT		0x06
#define MAX77665_F_RW_MAXFLASH_TIMER		0x07
#define MAX77665_F_RO_MAXFLASH_FLED1STAT	0x08
#define MAX77665_F_RO_MAXFLASH_FLED2STAT	0x09
#define MAX77665_F_RW_BOOST_MODE		0x0A
#define MAX77665_F_RW_BOOST_VOLT		0x0B
#define MAX77665_F_RO_BOOST_VOLTRD		0x0C
#define MAX77665_F_RC_FLASH_INTSTAT		0x0E
#define MAX77665_F_RW_FLASH_INTMASK		0x0F
#define MAX77665_F_RO_FLASH_STATUS		0x10

#define FIELD(x, y)			((x) << (y))
#define FMASK(x)			FIELD(0x03, (x))

#define TORCH_TIMER_SAFETY_DIS		0x1

#define TIMER_ONESHOT			0x0
#define TIMER_MAX			0x1

#define BOOST_FLASH_MODE_OFF		0x0
#define BOOST_FLASH_MODE_LED1		0x1
#define BOOST_FLASH_MODE_LED2		0x2
#define BOOST_FLASH_MODE_BOTH		0x3
#define BOOST_FLASH_MODE_FIXED		0x4

#define BOOST_MODE_ONELED		0x0
#define BOOST_MODE_TWOLED		0x1

#define LED2_TORCH_MODE_SHIFT		0
#define LED1_TORCH_MODE_SHIFT		2
#define LED2_FLASH_MODE_SHIFT		4
#define LED1_FLASH_MODE_SHIFT		6

#define LED1_TORCH_TRIG_MASK		FMASK(LED1_TORCH_MODE_SHIFT)
#define LED2_TORCH_TRIG_MASK		FMASK(LED2_TORCH_MODE_SHIFT)
#define LED1_FLASH_TRIG_MASK		FMASK(LED1_FLASH_MODE_SHIFT)
#define LED2_FLASH_TRIG_MASK		FMASK(LED2_FLASH_MODE_SHIFT)

/* TO DO: Need to confirm with maxim these trigger settings */
#define TRIG_MODE_OFF			0x00
#define TRIG_MODE_I2C			0x01
#define TRIG_MODE_FLASHEN		0x02
#define TRIG_MODE_TORCHEN		0x03

#define TORCH_TRIG_BY_FLASHEN	\
			(FIELD(TRIG_MODE_FLASHEN, LED2_TORCH_MODE_SHIFT) | \
			FIELD(TRIG_MODE_FLASHEN, LED1_TORCH_MODE_SHIFT))

#define TORCH_TRIG_BY_TORCHEN	\
			(FIELD(TRIG_MODE_TORCHEN, LED2_TORCH_MODE_SHIFT) | \
			FIELD(TRIG_MODE_TORCHEN, LED1_TORCH_MODE_SHIFT))

#define FLASH_TRIG_BY_FLASHEN	\
			(FIELD(TRIG_MODE_FLASHEN, LED2_FLASH_MODE_SHIFT) | \
			FIELD(TRIG_MODE_FLASHEN, LED1_FLASH_MODE_SHIFT))

#define FLASH_TRIG_BY_TORCHEN	\
			(FIELD(TRIG_MODE_TORCHEN, LED2_FLASH_MODE_SHIFT) | \
			FIELD(TRIG_MODE_TORCHEN, LED1_FLASH_MODE_SHIFT))

#define MAXFLASH_DISABLE		0
#define MAXFLASH_ENABLE			1

#define MAXFLASH_VOLT_HYS_FLOOR		100 /* mV */
#define MAXFLASH_VOLT_HYS_CEILING	300 /* mV */
#define MAXFLASH_VOLT_HYS_STEP		100 /* mV */

#define MAXFLASH_V_TH_FLOOR		2400 /* mV */
#define MAXFLASH_V_TH_CEILING		3400 /* mV */
#define MAXFLASH_V_TH_STEP		33 /* mV */

#define MAXFLASH_TIMER_STEP		256 /* uS */

#define BOOST_VOLT_FLOOR		3300 /* mV */
#define BOOST_VOLT_CEILING		5500 /* mV */
#define BOOST_VOLT_STEP			25 /* mV */

#define MAX77665_F_MAX_FLASH_LEVEL	(1 << 6)
#define MAX77665_F_MAX_TORCH_LEVEL	(1 << 4)

#define MAX77665_F_MAX_FLASH_CURRENT(x)    \
	DIV_ROUND_UP(((x) * MAX77665_F_MAX_FLASH_LEVEL), 1000)
#define MAX77665_F_MAX_TORCH_CURRENT(x) \
	DIV_ROUND_UP(((x) * MAX77665_F_MAX_TORCH_LEVEL), 1000)

#define SUSTAINTIME_DEF				558

/* minimium debounce time 600uS */
#define RECHARGEFACTOR_DEF			600

#define max77665_f_max_flash_cap_size	(sizeof(u32) \
				+ (sizeof(struct nvc_torch_level_info) \
				* (MAX77665_F_MAX_FLASH_LEVEL)))
#define max77665_f_max_torch_cap_size	(sizeof(u32) \
				+ (sizeof(s32) * (MAX77665_F_MAX_TORCH_LEVEL)))

struct max77665_f_caps_struct {
	u32 curr_step_uA;
	u32 max_peak_curr_mA;
	u32 max_torch_curr_mA;
	u32 max_total_current_mA;
};

struct max77665_f_reg_cache {
	bool regs_stale;
	u8 led1_curr;
	u8 led2_curr;
	u8 led_tcurr;
	u8 leds_en;
	u8 t_timer;
	u8 f_timer;
	u8 m_flash;
	u8 m_timing;
	u8 boost_control;
	u8 boost_vout_flash;
};

struct max77665_f_state_regs {
	u8 fled1_status;
	u8 fled2_status;
	u8 boost_volt;
	u8 flash_state;
	u8 progress_state;
};

struct max77665_f_info {
	struct device *dev;
	struct miscdevice miscdev;
	struct dentry *d_max77665_f;
	struct list_head list;
	struct max77665_f_info *s_info;
	struct mutex mutex;
	struct max77665_f_power_rail pwr_rail;
	struct max77665_f_platform_data *pdata;
	struct nvc_torch_flash_capabilities *flash_cap;
	struct nvc_torch_torch_capabilities *torch_cap;
	struct max77665_f_config config;
	struct max77665_f_reg_cache regs;
	struct max77665_f_state_regs states;
	struct regmap *regmap;
	atomic_t in_use;
	int flash_cap_size;
	int torch_cap_size;
	int pwr_api;
	int pwr_dev;
	int sustainTime;
	u8 fled_settings;
	bool flash_en;
	u8 s_mode;
	u8 new_ftimer;
	u8 new_ttimer;
};

static const struct max77665_f_caps_struct max77665_f_caps = {
	15625,
	MAX77665_F_MAX_FLASH_CURRENT(15625),
	MAX77665_F_MAX_TORCH_CURRENT(15625),
	625 * 2
};

static struct max77665_f_platform_data max77665_f_default_pdata = {
	.config		= {
			.led_mask = 3, /* both LEDs enabled */
			.torch_on_flash = false,
			.flash_on_torch = false,
			.flash_mode = 2,
			.torch_mode = 1,
			.adaptive_mode = 2,
			.max_peak_current_mA = 1000,
			.max_torch_current_mA = 250,
			.max_peak_duration_ms = 0,
			.max_flash_threshold_mV = 0,
			},
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "torch",
	.pinstate	= {0x0000, 0x0000},
};

static LIST_HEAD(max77665_f_info_list);
static DEFINE_SPINLOCK(max77665_f_spinlock);

static int max77665_f_dev_power_set(struct max77665_f_info *info, int pwr);

static inline int max77665_f_reg_wr(struct max77665_f_info *info,
		u8 reg, u8 val, bool stale_chk, bool refresh)
{
	if (likely(info->regs.regs_stale || refresh))
		return regmap_write(info->regmap, reg, val);
	return 0;
}

static int max77665_f_set_leds(struct max77665_f_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	int err = 0;
	u8 fled_en = 0;
	u8 t_curr = 0;
	u8 val = 0;

	if (mask & 1) {
		fled_en |= (info->fled_settings & LED1_TORCH_TRIG_MASK);

		if (info->flash_en) {
			if (curr1 >= info->flash_cap->numberoflevels)
				curr1 = info->flash_cap->numberoflevels - 1;

			err = max77665_f_reg_wr(info,
				MAX77665_F_RW_FLASH_FLED1CURR, curr1,
				info->regs.regs_stale,
				info->regs.led1_curr != curr1);
			if (!err)
				info->regs.led1_curr = curr1;
			else
				goto set_led_end;

			fled_en |= (info->fled_settings & LED1_FLASH_TRIG_MASK);
		} else {
			if (curr1 >= info->torch_cap->numberoflevels)
				curr1 = info->torch_cap->numberoflevels - 1;

			t_curr = curr1;
		}
	}

	if (mask & 2) {
		fled_en |= (info->fled_settings & LED2_TORCH_TRIG_MASK);

		if (info->flash_en) {
			if (curr2 >= info->flash_cap->numberoflevels)
				curr2 = info->flash_cap->numberoflevels - 1;

			err = max77665_f_reg_wr(info,
				MAX77665_F_RW_FLASH_FLED2CURR, curr2,
				info->regs.regs_stale,
				info->regs.led2_curr != curr2);
			if (!err)
				info->regs.led2_curr = curr2;
			else
				goto set_led_end;

			fled_en |= (info->fled_settings & LED2_FLASH_TRIG_MASK);
		} else {
			if (curr2 >= info->torch_cap->numberoflevels)
				curr2 = info->torch_cap->numberoflevels - 1;

			t_curr |= curr2 << 4;
		}
	}

	/* if any led is set as flash, update the flash timer register */
	if (fled_en & (LED1_FLASH_TRIG_MASK | LED2_FLASH_TRIG_MASK)) {
		val = (info->regs.f_timer & FIELD(TIMER_MAX, 7)) |
			info->new_ftimer;
		err = max77665_f_reg_wr(info,
			MAX77665_F_RW_FLASH_TIMER, val, info->regs.regs_stale,
			(info->regs.f_timer & 0x0f) != info->new_ftimer);
		if (err)
			goto set_led_end;
		info->regs.f_timer = val;
	}

	/* if any led is set as torch, update the torch timer register */
	if (fled_en & (LED1_TORCH_TRIG_MASK | LED2_TORCH_TRIG_MASK)) {
		err = max77665_f_reg_wr(info,
				MAX77665_F_RW_TORCH_FLEDCURR, t_curr,
				info->regs.regs_stale,
				info->regs.led_tcurr != val);
		if (!err)
			info->regs.led_tcurr = val;
		else
			goto set_led_end;

		val = (info->regs.t_timer & FIELD(TIMER_MAX, 7)) |
			(info->new_ttimer & 0x0f);
		err = max77665_f_reg_wr(info, MAX77665_F_RW_TORCH_TIMER, val,
				info->regs.regs_stale,
				(info->regs.t_timer & 0x0f) !=
				info->new_ttimer);
		if (err)
			goto set_led_end;
		info->regs.t_timer = val;
	}

	err = max77665_f_reg_wr(info,
		MAX77665_F_RW_FLED_ENABLE, fled_en,
				info->regs.regs_stale,
				info->regs.leds_en != fled_en);
	if (err)
		goto set_led_end;
	info->regs.leds_en = fled_en;

	dev_dbg(info->dev, "%s %x %x %x en = %x\n",
		__func__, mask, curr1, curr2, fled_en);

set_led_end:
	return err;
}

static inline int max77665_f_get_boost_volt(u16 mV)
{
	if (mV <= BOOST_VOLT_FLOOR)
		return 0;
	if (mV >= BOOST_VOLT_CEILING)
		return 0x64;

	return (mV - BOOST_VOLT_FLOOR) / BOOST_VOLT_STEP + 0x0C;
}

static void max77665_f_update_config(struct max77665_f_info *info)
{
	struct max77665_f_config *pcfg = &info->config;
	struct max77665_f_config *pcfg_cust;

	memcpy(pcfg, &max77665_f_default_pdata.config, sizeof(*pcfg));
	if (!info->pdata) {
		info->pdata = &max77665_f_default_pdata;
		dev_dbg(info->dev, "%s No platform data.  Using defaults.\n",
			__func__);
		goto update_end;
	}
	pcfg_cust = &info->pdata->config;

	pcfg->torch_on_flash = pcfg_cust->torch_on_flash;
	pcfg->flash_on_torch = pcfg_cust->flash_on_torch;

	if (pcfg_cust->led_mask)
		pcfg->led_mask = pcfg_cust->led_mask;

	if (pcfg_cust->flash_mode)
		pcfg->flash_mode = pcfg_cust->flash_mode;

	if (pcfg_cust->torch_mode)
		pcfg->torch_mode = pcfg_cust->torch_mode;

	if (pcfg_cust->adaptive_mode)
		pcfg->adaptive_mode = pcfg_cust->adaptive_mode;

	if (pcfg_cust->boost_vout_flash_mV)
		pcfg->boost_vout_flash_mV = pcfg_cust->boost_vout_flash_mV;

	if (pcfg_cust->max_total_current_mA)
		pcfg->max_total_current_mA = pcfg_cust->max_total_current_mA;

	if (pcfg_cust->max_peak_current_mA)
		pcfg->max_peak_current_mA = pcfg_cust->max_peak_current_mA;

	if (pcfg_cust->max_peak_duration_ms)
		pcfg->max_peak_duration_ms = pcfg_cust->max_peak_duration_ms;

	if (pcfg_cust->max_torch_current_mA)
		pcfg->max_torch_current_mA = pcfg_cust->max_torch_current_mA;

	if (pcfg_cust->max_flash_threshold_mV)
		pcfg->max_flash_threshold_mV =
				pcfg_cust->max_flash_threshold_mV;

	if (pcfg_cust->max_flash_hysteresis_mV)
		pcfg->max_flash_hysteresis_mV =
				pcfg_cust->max_flash_hysteresis_mV;

	if (pcfg_cust->max_flash_lbdly_f_uS)
		pcfg->max_flash_lbdly_f_uS =
				pcfg_cust->max_flash_lbdly_f_uS;

	if (pcfg_cust->max_flash_lbdly_r_uS)
		pcfg->max_flash_lbdly_r_uS =
				pcfg_cust->max_flash_lbdly_r_uS;

update_end:
	/* FLED enable settings */
	/* How TORCH is triggered */
	if (pcfg->torch_on_flash) /* triggered by FLASHEN */
		info->fled_settings = TORCH_TRIG_BY_FLASHEN;
	else /* triggered by TORCHEN */
		info->fled_settings = TORCH_TRIG_BY_TORCHEN;

	/* How FLASH is triggered */
	if (pcfg->flash_on_torch) /* triggered by TORCHEN */
		info->fled_settings |= FLASH_TRIG_BY_TORCHEN;
	else /* triggered by FLASHEN */
		info->fled_settings |= FLASH_TRIG_BY_FLASHEN;

	if (1 == pcfg->adaptive_mode)
		info->regs.boost_control = BOOST_FLASH_MODE_FIXED;
	else {
		if (pcfg->led_mask > 3) {
			dev_dbg(info->dev, "%s invalid led mask = %d\n",
				__func__, pcfg->led_mask);
			info->regs.boost_control = BOOST_FLASH_MODE_BOTH;
		} else
			info->regs.boost_control = pcfg->led_mask;
	}

	/* both FLED1/FLED2 are enabled */
	if (info->regs.boost_control == FIELD(BOOST_FLASH_MODE_BOTH, 0))
		info->regs.boost_control |= FIELD(BOOST_MODE_TWOLED, 7);

	info->regs.boost_vout_flash =
		max77665_f_get_boost_volt(pcfg->boost_vout_flash_mV);

	info->regs.f_timer = (pcfg->flash_mode == 1) ?
			FIELD(TIMER_ONESHOT, 7) : FIELD(TIMER_MAX, 7);

	switch (pcfg->torch_mode) {
	case 1:
		info->regs.t_timer = FIELD(TIMER_ONESHOT, 7) |
					FIELD(TORCH_TIMER_SAFETY_DIS, 6);
		break;
	case 2:
		info->regs.t_timer = FIELD(TIMER_ONESHOT, 7);
		break;
	case 3:
	default:
		info->regs.t_timer = FIELD(TIMER_MAX, 7);
		break;
	}

	if (pcfg->max_flash_threshold_mV) {
		if (pcfg->max_flash_threshold_mV < MAXFLASH_V_TH_FLOOR)
			pcfg->max_flash_threshold_mV = MAXFLASH_V_TH_FLOOR;
		else if (pcfg->max_flash_threshold_mV > MAXFLASH_V_TH_CEILING)
			pcfg->max_flash_threshold_mV = MAXFLASH_V_TH_CEILING;

		/* 0 - hysteresis disabled */
		if (pcfg->max_flash_hysteresis_mV) {
			if (pcfg->max_flash_hysteresis_mV <
				MAXFLASH_VOLT_HYS_FLOOR)
				pcfg->max_flash_hysteresis_mV =
					MAXFLASH_VOLT_HYS_FLOOR;
			else if (pcfg->max_flash_hysteresis_mV >
				 MAXFLASH_VOLT_HYS_CEILING)
				pcfg->max_flash_hysteresis_mV =
					MAXFLASH_VOLT_HYS_CEILING;
		}

		info->regs.m_flash = FIELD(MAXFLASH_ENABLE, 7) |
		((pcfg->max_flash_threshold_mV - MAXFLASH_V_TH_FLOOR) /
				MAXFLASH_V_TH_STEP);
		info->regs.m_flash |= (pcfg->max_flash_hysteresis_mV +
		MAXFLASH_VOLT_HYS_STEP / 2) / MAXFLASH_VOLT_HYS_STEP;
	}

	if (pcfg->max_flash_lbdly_f_uS)
		info->regs.m_timing =
		FIELD(pcfg->max_flash_lbdly_f_uS / MAXFLASH_TIMER_STEP, 0);

	if (pcfg->max_flash_lbdly_r_uS)
		info->regs.m_timing |=
		FIELD(pcfg->max_flash_lbdly_r_uS / MAXFLASH_TIMER_STEP, 3);
}

static int max77665_f_update_settings(struct max77665_f_info *info)
{
	int err = 0;

	info->regs.regs_stale = true;
	err |= max77665_f_reg_wr(info, MAX77665_F_RW_BOOST_MODE,
				info->regs.boost_control, true, false);

	err |= max77665_f_reg_wr(info, MAX77665_F_RW_BOOST_VOLT,
				info->regs.boost_vout_flash, true, false);

	err |= max77665_f_reg_wr(info, MAX77665_F_RW_MAXFLASH_VOLT,
				info->regs.m_flash, true, false);

	err |= max77665_f_reg_wr(info, MAX77665_F_RW_MAXFLASH_TIMER,
				info->regs.m_timing, true, false);

	err |= max77665_f_set_leds(info, info->config.led_mask,
				info->regs.led1_curr, info->regs.led2_curr);

	info->regs.regs_stale = false;
	return err;
}

static int max77665_f_configure(struct max77665_f_info *info, bool update)
{
	struct max77665_f_config *pcfg = &info->config;
	struct nvc_torch_flash_capabilities *pfcap = info->flash_cap;
	struct nvc_torch_torch_capabilities *ptcap = info->torch_cap;
	int val;
	int i;

	if (pcfg->max_peak_current_mA > max77665_f_caps.max_peak_curr_mA ||
		!pcfg->max_peak_current_mA) {
		dev_warn(info->dev,
				"max_peak_current_mA of %d invalid, "
				"changed to %d\n",
				pcfg->max_peak_current_mA,
				max77665_f_caps.max_peak_curr_mA);
		pcfg->max_peak_current_mA = max77665_f_caps.max_peak_curr_mA;
	}

	i = 1;
	if ((info->config.led_mask & 3) == 3)
		i = 2;
	val = pcfg->max_peak_current_mA * i;
	if (val > max77665_f_caps.max_total_current_mA)
		val = max77665_f_caps.max_total_current_mA;

	if (!pcfg->max_total_current_mA || pcfg->max_total_current_mA > val)
		pcfg->max_total_current_mA = val;
	pcfg->max_peak_current_mA =
		pcfg->max_total_current_mA / i;

	if (pcfg->max_torch_current_mA > max77665_f_caps.max_torch_curr_mA ||
		!pcfg->max_torch_current_mA) {
		dev_warn(info->dev,
				"max_torch_current_mA of %d invalid"
				"changing to %d\n",
				pcfg->max_torch_current_mA,
				max77665_f_caps.max_torch_curr_mA);
		pcfg->max_torch_current_mA =
			max77665_f_caps.max_torch_curr_mA;
	}

	pfcap->levels[0].guidenum = 0;
	pfcap->levels[0].sustaintime = 0xFFFFFFFF;
	pfcap->levels[0].rechargefactor = 0;
	val = max77665_f_caps.curr_step_uA;
	for (i = 1; i < MAX77665_F_MAX_FLASH_LEVEL; i++) {
		pfcap->levels[i].guidenum = val * i / 1000; /* mA */
		if (pfcap->levels[i].guidenum >
			pcfg->max_peak_current_mA) {
			pfcap->levels[i].guidenum = 0;
			break;
		}
		pfcap->levels[i].sustaintime = info->sustainTime;
		pfcap->levels[i].rechargefactor = RECHARGEFACTOR_DEF;
	}
	info->flash_cap_size = (sizeof(u32) +
			(sizeof(struct nvc_torch_level_info) * i));
	pfcap->numberoflevels = i;

	for (i = 0; i < MAX77665_F_MAX_TORCH_LEVEL; i++) {
		ptcap->guidenum[i] = pfcap->levels[i].guidenum;
		if (ptcap->guidenum[i] > pcfg->max_torch_current_mA) {
			ptcap->guidenum[i] = 0;
			break;
		}
	}
	info->torch_cap_size = (sizeof(u32) + (sizeof(s32) * i));
	ptcap->numberoflevels = i;

	if (update && (info->pwr_dev == NVC_PWR_COMM ||
			info->pwr_dev == NVC_PWR_ON))
		return max77665_f_update_settings(info);

	return 0;
}

static int max77665_f_strobe(struct max77665_f_info *info, int t_on)
{
	u32 gpio = info->pdata->gpio_strobe & 0xffff;
	u32 lact = (info->pdata->gpio_strobe & 0xffff0000) ? 1 : 0;
	return gpio_direction_output(gpio, lact ^ (t_on & 1));
}

#ifdef CONFIG_PM
static int max77665_f_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct max77665_f_info *info = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Suspending\n");
	info->regs.regs_stale = true;

	return 0;
}

static int max77665_f_resume(struct platform_device *pdev)
{
	struct max77665_f_info *info = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Resuming\n");
	info->regs.regs_stale = true;

	return 0;
}

static void max77665_f_shutdown(struct platform_device *pdev)
{
	struct max77665_f_info *info = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Shutting down\n");

	mutex_lock(&info->mutex);
	max77665_f_set_leds(info, 3, 0, 0);
	mutex_unlock(&info->mutex);
	info->regs.regs_stale = true;
}
#endif

static int max77665_f_power_on(struct max77665_f_info *info)
{
	struct max77665_f_power_rail *pw = &info->pwr_rail;
	int err = 0;

	if (pw->vio) {
		err = regulator_enable(pw->vio);
		if (err) {
			dev_err(info->dev, "%s host err\n", __func__);
			return err;
		}
	}

	if (pw->vbus) {
		err = regulator_enable(pw->vbus);
		if (err) {
			if (pw->vio)
				regulator_disable(pw->vio);

			dev_err(info->dev, "%s in err\n", __func__);
			return err;
		}
	}

	if (pw->i2c) {
		err = regulator_enable(pw->i2c);
		if (err) {
			if (pw->vbus)
				regulator_disable(pw->vbus);
			if (pw->vio)
				regulator_disable(pw->vio);

			dev_err(info->dev, "%s i2c err\n", __func__);
			return err;
		}
	}

	if (info->pdata && info->pdata->poweron_callback)
		err = info->pdata->poweron_callback(pw);

	if (!err)
		err = max77665_f_update_settings(info);

	return err;
}

static int max77665_f_power_off(struct max77665_f_info *info)
{
	struct max77665_f_power_rail *pw = &info->pwr_rail;
	int err = 0;

	if (info->pdata && info->pdata->poweroff_callback)
		err = info->pdata->poweroff_callback(pw);

	if (!err)
		return err;

	if (pw->i2c)
		regulator_disable(pw->i2c);

	if (pw->vbus)
		regulator_disable(pw->vbus);

	if (pw->vio)
		regulator_disable(pw->vio);

	return err;
}

static int max77665_f_power(struct max77665_f_info *info, int pwr)
{
	int err = 0;

	if (pwr == info->pwr_dev)
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF:
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = max77665_f_power_off(info);
		break;
	case NVC_PWR_STDBY_OFF:
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = max77665_f_power_on(info);
		break;
	case NVC_PWR_STDBY:
		err = max77665_f_power_on(info);
		err |= max77665_f_set_leds(info, 3, 0, 0);
		break;

	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = max77665_f_power_on(info);
		break;

	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(info->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_dev = pwr;
	if (err > 0)
		return 0;

	return err;
}

static int max77665_f_power_sync(struct max77665_f_info *info, int pwr)
{
	int err1 = 0;
	int err2 = 0;

	if ((info->s_mode == NVC_SYNC_OFF) ||
			(info->s_mode == NVC_SYNC_MASTER) ||
			(info->s_mode == NVC_SYNC_STEREO))
		err1 = max77665_f_power(info, pwr);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
			(info->s_mode == NVC_SYNC_STEREO))
		err2 = max77665_f_power(info->s_info, pwr);
	return err1 | err2;
}

static int max77665_f_power_user_set(struct max77665_f_info *info, int pwr)
{
	int err = 0;

	if (!pwr || (pwr > NVC_PWR_ON))
		return 0;

	if (pwr > info->pwr_dev)
		err = max77665_f_power_sync(info, pwr);
	if (!err)
		info->pwr_api = pwr;
	else
		info->pwr_api = NVC_PWR_ERR;
	if (info->pdata->cfg & NVC_CFG_NOERR)
		return 0;

	return err;
}

static int max77665_f_dev_power_set(struct max77665_f_info *info, int pwr)
{
	if (pwr < info->pwr_api)
		pwr = info->pwr_api;
	return max77665_f_power(info, pwr);
}

static int max77665_f_get_param(struct max77665_f_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_pin_state pinstate;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	u8 reg;

	if (copy_from_user(&params,
			(const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (info->s_mode == NVC_SYNC_SLAVE)
		info = info->s_info;
	switch (params.param) {
	case NVC_PARAM_FLASH_CAPS:
		dev_dbg(info->dev, "%s FLASH_CAPS\n", __func__);
		data_ptr = info->flash_cap;
		data_size = info->flash_cap_size;
		break;

	case NVC_PARAM_FLASH_LEVEL:
		reg = info->regs.led1_curr;
		data_ptr = &info->flash_cap->levels[reg].guidenum;
		data_size = sizeof(info->flash_cap->levels[reg].guidenum);
		break;

	case NVC_PARAM_TORCH_CAPS:
		dev_dbg(info->dev, "%s TORCH_CAPS\n", __func__);
		data_ptr = info->torch_cap;
		data_size = info->torch_cap_size;
		break;

	case NVC_PARAM_TORCH_LEVEL:
		reg = info->regs.led1_curr;
		data_ptr = &info->torch_cap->guidenum[reg];
		data_size = sizeof(info->torch_cap->guidenum[reg]);
		break;

	case NVC_PARAM_FLASH_PIN_STATE:
		pinstate = info->pdata->pinstate;
		if (!info->flash_en)
			pinstate.values ^= 0xffff;

		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %x&%x\n",
				__func__, pinstate.mask, pinstate.values);
		data_ptr = &pinstate;
		data_size = sizeof(pinstate);
		break;

	case NVC_PARAM_STEREO:
		dev_dbg(info->dev, "%s STEREO: %d\n", __func__, info->s_mode);
		data_ptr = &info->s_mode;
		data_size = sizeof(info->s_mode);
		break;

	default:
		dev_err(info->dev,
				"%s unsupported parameter: %d\n",
				__func__, params.param);
		return -EINVAL;
	}

	if (params.sizeofvalue < data_size) {
		dev_err(info->dev,
				"%s data size mismatch %d != %d\n",
				__func__, params.sizeofvalue, data_size);
		return -EINVAL;
	}

	if (copy_to_user((void __user *)params.p_value,
			 data_ptr,
			 data_size)) {
		dev_err(info->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

static int max77665_f_param_update(struct max77665_f_info *info,
			       struct nvc_param *params,
			       u8 val)
{
	int err;

	switch (params->param) {
	case NVC_PARAM_FLASH_LEVEL:
		dev_dbg(info->dev, "%s FLASH_LEVEL: %d\n", __func__, val);
		max77665_f_dev_power_set(info, NVC_PWR_ON);
		info->new_ftimer =
			info->flash_cap->levels[val].sustaintime & 0X0F;
		info->flash_en = true;

		err = max77665_f_set_leds(info,
			info->config.led_mask, val, val);
		if (!val) /*turn pwr off if no flash && no pwr_api*/
			max77665_f_dev_power_set(info, NVC_PWR_OFF);
		return err;

	case NVC_PARAM_TORCH_LEVEL:
		dev_dbg(info->dev, "%s TORCH_LEVEL: %d\n", __func__, val);
		max77665_f_dev_power_set(info, NVC_PWR_ON);
		info->flash_en = false;

		err = max77665_f_set_leds(info,
			info->config.led_mask, val, val);
		if (!val) /*turn pwr off if no flash && no pwr_api*/
			max77665_f_dev_power_set(info, NVC_PWR_OFF);
		return err;

	case NVC_PARAM_FLASH_PIN_STATE:
		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %d\n",
				__func__, val);
		return max77665_f_strobe(info, val);

	default:
		dev_err(info->dev,
				"%s unsupported parameter: %d\n",
				__func__, params->param);
		return -EINVAL;
	}
}

static int max77665_f_set_param(struct max77665_f_info *info, long arg)
{
	struct nvc_param params;
	u8 val;
	int err = 0;

	if (copy_from_user(&params,
				(const void __user *)arg,
				sizeof(struct nvc_param))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (copy_from_user(&val, (const void __user *)params.p_value,
			   sizeof(val))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	/* parameters independent of sync mode */
	switch (params.param) {
	case NVC_PARAM_STEREO:
		dev_dbg(info->dev, "%s STEREO: %d\n", __func__, (int)val);
		if (val == info->s_mode)
			return 0;

		switch (val) {
		case NVC_SYNC_OFF:
			info->s_mode = val;
			if (info->s_info != NULL) {
				info->s_info->s_mode = val;
				max77665_f_power(info->s_info, NVC_PWR_OFF);
			}
			break;

		case NVC_SYNC_MASTER:
			info->s_mode = val;
			if (info->s_info != NULL)
				info->s_info->s_mode = val;
			break;

		case NVC_SYNC_SLAVE:
		case NVC_SYNC_STEREO:
			if (info->s_info != NULL) {
				/* sync power */
				info->s_info->pwr_api = info->pwr_api;
				err = max77665_f_power(info->s_info,
						     info->pwr_dev);
				if (!err) {
					info->s_mode = val;
					info->s_info->s_mode = val;
				} else {
					max77665_f_power(info->s_info,
						       NVC_PWR_OFF);
					err = -EIO;
				}
			} else {
				err = -EINVAL;
			}
			break;

		default:
			err = -EINVAL;
		}
		if (info->pdata->cfg & NVC_CFG_NOERR)
			return 0;

		return err;

	default:
	/* parameters dependent on sync mode */
		switch (info->s_mode) {
		case NVC_SYNC_OFF:
		case NVC_SYNC_MASTER:
			return max77665_f_param_update(info, &params, val);

		case NVC_SYNC_SLAVE:
			return max77665_f_param_update(info->s_info,
						 &params,
						 val);

		case NVC_SYNC_STEREO:
			err = max77665_f_param_update(info, &params, val);
			if (!(info->pdata->cfg & NVC_CFG_SYNC_I2C_MUX))
				err |= max77665_f_param_update(info->s_info,
							 &params,
							 val);
			return err;

		default:
			dev_err(info->dev, "%s %d internal err\n",
					__func__, __LINE__);
			return -EINVAL;
		}
	}
}

static long max77665_f_ioctl(struct file *file,
			   unsigned int cmd,
			   unsigned long arg)
{
	struct max77665_f_info *info = file->private_data;
	int pwr;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		return max77665_f_set_param(info, arg);

	case NVC_IOCTL_PARAM_RD:
		return max77665_f_get_param(info, arg);

	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(info->dev, "%s PWR_WR: %d\n", __func__, pwr);
		return max77665_f_power_user_set(info, pwr);

	case NVC_IOCTL_PWR_RD:
		if (info->s_mode == NVC_SYNC_SLAVE)
			pwr = info->s_info->pwr_api / 2;
		else
			pwr = info->pwr_api / 2;
		dev_dbg(info->dev, "%s PWR_RD: %d\n", __func__, pwr);
		if (copy_to_user((void __user *)arg, (const void *)&pwr,
				 sizeof(pwr))) {
			dev_err(info->dev, "%s copy_to_user err line %d\n",
					__func__, __LINE__);
			return -EFAULT;
		}

		return 0;

	default:
		dev_err(info->dev, "%s unsupported ioctl: %x\n",
				__func__, cmd);
		return -EINVAL;
	}
}

static int max77665_f_sync_enable(int dev1, int dev2)
{
	struct max77665_f_info *sync1 = NULL;
	struct max77665_f_info *sync2 = NULL;
	struct max77665_f_info *pos = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &max77665_f_info_list, list) {
		if (pos->pdata->num == dev1) {
			sync1 = pos;
			break;
		}
	}
	pos = NULL;
	list_for_each_entry_rcu(pos, &max77665_f_info_list, list) {
		if (pos->pdata->num == dev2) {
			sync2 = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (sync1 != NULL)
		sync1->s_info = NULL;
	if (sync2 != NULL)
		sync2->s_info = NULL;
	if (!dev1 && !dev2)
		return 0; /* no err if default instance 0's used */

	if (dev1 == dev2)
		return -EINVAL; /* err if sync instance is itself */

	if ((sync1 != NULL) && (sync2 != NULL)) {
		sync1->s_info = sync2;
		sync2->s_info = sync1;
	}

	return 0;
}

static int max77665_f_sync_disable(struct max77665_f_info *info)
{
	if (info->s_info != NULL) {
		info->s_info->s_mode = 0;
		info->s_info->s_info = NULL;
		info->s_mode = 0;
		info->s_info = NULL;
		return 0;
	}

	return -EINVAL;
}

static int max77665_f_open(struct inode *inode, struct file *file)
{
	struct max77665_f_info *info = NULL;
	struct max77665_f_info *pos = NULL;
	int err;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &max77665_f_info_list, list) {
		if (pos->miscdev.minor == iminor(inode)) {
			info = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (!info)
		return -ENODEV;

	err = max77665_f_sync_enable(info->pdata->num, info->pdata->sync);
	if (err == -EINVAL)
		dev_err(info->dev,
			 "%s err: invalid num (%u) and sync (%u) instance\n",
			 __func__, info->pdata->num, info->pdata->sync);
	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	if (info->s_info != NULL) {
		if (atomic_xchg(&info->s_info->in_use, 1))
			return -EBUSY;
	}

	file->private_data = info;
	dev_dbg(info->dev, "%s\n", __func__);
	return 0;
}

static int max77665_f_release(struct inode *inode, struct file *file)
{
	struct max77665_f_info *info = file->private_data;

	dev_dbg(info->dev, "%s\n", __func__);
	max77665_f_power_sync(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	if (info->s_info != NULL)
		WARN_ON(!atomic_xchg(&info->s_info->in_use, 0));
	max77665_f_sync_disable(info);
	return 0;
}

static const struct file_operations max77665_f_fileops = {
	.owner = THIS_MODULE,
	.open = max77665_f_open,
	.unlocked_ioctl = max77665_f_ioctl,
	.release = max77665_f_release,
};

static void max77665_f_del(struct max77665_f_info *info)
{
	max77665_f_power_sync(info, NVC_PWR_OFF);

	if (info->pdata && info->pdata->power_put)
		info->pdata->power_put(&info->pwr_rail);

	max77665_f_sync_disable(info);
	spin_lock(&max77665_f_spinlock);
	list_del_rcu(&info->list);
	spin_unlock(&max77665_f_spinlock);
	synchronize_rcu();
}

static int max77665_f_remove(struct platform_device *pdev)
{
	struct max77665_f_info *info = dev_get_drvdata(&pdev->dev);

	dev_dbg(info->dev, "%s\n", __func__);
	misc_deregister(&info->miscdev);
	max77665_f_del(info);
	if (info->d_max77665_f)
		debugfs_remove_recursive(info->d_max77665_f);

	return 0;
}

static int max77665_f_debugfs_init(struct max77665_f_info *info);

static int max77665_f_probe(struct platform_device *pdev)
{
	struct max77665_f_info *info;
	struct max77665 *maxim;
	char dname[16];

	dev_info(&pdev->dev, "%s\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(*info) +
			max77665_f_max_flash_cap_size +
			max77665_f_max_torch_cap_size,
			GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->dev = &pdev->dev;
	maxim = dev_get_drvdata(info->dev->parent);
	info->regmap = maxim->regmap[MAX77665_I2C_SLAVE_PMIC];
	if (unlikely(!info->regmap)) {
		dev_err(&pdev->dev, "%s: max77665 platform error\n", __func__);
		return -EFAULT;
	}

	if (pdev->dev.platform_data) {
		info->pdata = pdev->dev.platform_data;
		dev_dbg(&pdev->dev, "pdata: %s\n", info->pdata->dev_name);
	} else
		dev_warn(&pdev->dev, "%s NO platform data\n", __func__);

	info->flash_cap = (void *)info + sizeof(*info);
	info->torch_cap = (void *)info->flash_cap +
				max77665_f_max_flash_cap_size;
	info->sustainTime = SUSTAINTIME_DEF;

	max77665_f_update_config(info);

	/* flash mode */
	info->flash_en = true;

	max77665_f_configure(info, false);

	dev_set_drvdata(info->dev, info);
	mutex_init(&info->mutex);
	INIT_LIST_HEAD(&info->list);
	spin_lock(&max77665_f_spinlock);
	list_add_rcu(&info->list, &max77665_f_info_list);
	spin_unlock(&max77665_f_spinlock);

	if (info->pdata && info->pdata->power_get)
		info->pdata->power_get(&info->pwr_rail);

	if (info->pdata->dev_name != NULL)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "max77665_f");
	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
			 dname, info->pdata->num);
	info->miscdev.name = dname;
	info->miscdev.fops = &max77665_f_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&pdev->dev, "%s unable to register misc device %s\n",
				__func__, dname);
		max77665_f_del(info);
		return -ENODEV;
	}

	max77665_f_debugfs_init(info);
	return 0;
}

static int max77665_f_status_show(struct seq_file *s, void *data)
{
	struct max77665_f_info *info = s->private;

	dev_info(info->dev, "%s\n", __func__);

	seq_printf(s, "max77665_f status:\n"
		"    Led Mask         = %01x\n"
		"    Led1 Current     = 0x%02x\n"
		"    Led2 Current     = 0x%02x\n"
		"    Output Mode      = %s\n"
		"    Led Settings     = 0x%02x\n"
		"    Flash TimeOut    = 0x%02x\n"
		"    PinState Mask    = 0x%04x\n"
		"    PinState Values  = 0x%04x\n"
		"    Max_Peak_Current = %dmA\n"
		,
		info->config.led_mask,
		info->regs.led1_curr,
		info->regs.led2_curr,
		info->flash_en ? "FLASH" : "TORCH",
		info->fled_settings,
		info->regs.f_timer,
		info->pdata->pinstate.mask,
		info->pdata->pinstate.values,
		info->config.max_peak_current_mA
		);

	return 0;
}

static ssize_t max77665_f_attr_set(struct file *s,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct max77665_f_info *info =
		((struct seq_file *)s->private_data)->private;
	char buf[24];
	int buf_size;
	u32 val = 0;

	dev_info(info->dev, "%s\n", __func__);

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf + 1, "0x%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "0X%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%d", &val) == 1)
		goto set_attr;

	dev_err(info->dev, "SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	dev_info(info->dev, "new data = %x\n", val);
	switch (buf[0]) {
	case 'c': /* change led 1/2 current settings */
		max77665_f_set_leds(info, info->config.led_mask,
			val & 0xff, (val >> 8) & 0xff);
		break;
	case 'l': /* enable/disable led 1/2 */
		info->config.led_mask = val;
		max77665_f_configure(info, false);
		break;
	case 'm': /* change pinstate setting */
		info->pdata->pinstate.mask = (val >> 16) & 0xffff;
		info->pdata->pinstate.values = val & 0xffff;
		break;
	case 'f': /* modify flash timeout reg */
		info->new_ftimer = val & 0X0F;
		max77665_f_set_leds(info, info->config.led_mask,
			info->regs.led1_curr,
			info->regs.led2_curr);
		break;
	case 'p':
		if (val)
			max77665_f_power(info, NVC_PWR_ON);
		else
			max77665_f_power(info, NVC_PWR_OFF);
		break;
	case 'k':
		if (val & 0xffff)
			info->config.max_peak_current_mA =
				val & 0xffff;
		max77665_f_configure(info, true);
		break;
	case 'x':
		info->flash_en = (val & 0xff00) ? true : false;
		if (val & 0xff)
			info->fled_settings = val & 0xff;
		max77665_f_configure(info, true);
		break;
	case 'g':
		info->pdata->gpio_strobe = val;
		max77665_f_strobe(info, 1);
		break;
	}

	return count;
}

static int max77665_f_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, max77665_f_status_show, inode->i_private);
}

static const struct file_operations max77665_f_debugfs_fops = {
	.open = max77665_f_debugfs_open,
	.read = seq_read,
	.write = max77665_f_attr_set,
	.llseek = seq_lseek,
	.release = single_release,
};

static int max77665_f_debugfs_init(struct max77665_f_info *info)
{
	struct dentry *d;

	info->d_max77665_f = debugfs_create_dir(
		info->miscdev.this_device->kobj.name, NULL);
	if (info->d_max77665_f == NULL) {
		dev_err(info->dev, "%s: debugfs mk dir failed\n", __func__);
		return -ENOMEM;
	}

	d = debugfs_create_file("d", S_IRUGO|S_IWUSR, info->d_max77665_f,
		(void *)info, &max77665_f_debugfs_fops);
	if (!d) {
		dev_err(info->dev, "%s: debugfs mk file failed\n", __func__);
		debugfs_remove_recursive(info->d_max77665_f);
		info->d_max77665_f = NULL;
	}

	return -EFAULT;
}

static const struct platform_device_id max77665_flash_id[] = {
	{ "max77665-flash", 0 },
	{ },
};

static struct platform_driver max77665_flash_drv = {
	.driver = {
		.name = "max77665-flash",
		.owner = THIS_MODULE,
	},
	.id_table = max77665_flash_id,
	.probe = max77665_f_probe,
	.remove = max77665_f_remove,
#ifdef CONFIG_PM
	.shutdown = max77665_f_shutdown,
	.suspend  = max77665_f_suspend,
	.resume   = max77665_f_resume,
#endif
};

module_platform_driver(max77665_flash_drv);
MODULE_DESCRIPTION("MAXIM MAX77665_F flash/torch driver");
MODULE_AUTHOR("Charlie Huang <chahuang@nvidia.com>");
MODULE_LICENSE("GPL");
