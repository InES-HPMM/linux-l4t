/*
 * AS364X.c - AS364X flash/torch kernel driver
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.

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

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <media/nvc.h>
#include <media/as364x.h>

#define AS364X_REG_CHIPID		0x00
#define AS364X_REG_LED1_SET_CURR	0x01
#define AS364X_REG_LED2_SET_CURR	0x02
#define AS364X_REG_TXMASK		0x03
#define AS364X_REG_LOWVOLTAGE		0x04
#define AS364X_REG_FLASHTIMER		0x05
#define AS364X_REG_CONTROL		0x06
#define AS364X_REG_STROBE		0x07
#define AS364X_REG_FAULT		0x08
#define AS364X_REG_PWM_INDICATOR	0x09
#define AS364X_REG_LED_CURR_MIN		0x0E
#define AS364X_REG_LED_CURR_ACT		0x0F
#define AS364X_REG_PASSWORD		0x80
#define AS364X_REG_CURR_BOOST		0x81

#define AS364X_REG_CONTROL_MODE_EXT_TORCH	0x00
#define AS364X_REG_CONTROL_MODE_INDICATOR	0x01
#define AS364X_REG_CONTROL_MODE_ASSIST		0x02
#define AS364X_REG_CONTROL_MODE_FLASH		0x03

#define AS364X_MAX_ASSIST_CURRENT(x)    \
	DIV_ROUND_UP(((x) * 0xff * 0x7f / 0xff), 1000)
#define AS364X_MAX_INDICATOR_CURRENT(x) \
	DIV_ROUND_UP(((x) * 0xff * 0x3f / 0xff), 1000)

#define AS364X_MAX_FLASH_LEVEL	256
#define AS364X_MAX_TORCH_LEVEL	128

#define SUSTAINTIME_DEF		558
#define DEFAULT_FLASHTIME	((SUSTAINTIME_DEF > 256) ? \
				((SUSTAINTIME_DEF - 249) / 8 + 128) : \
				((SUSTAINTIME_DEF - 1) / 2))
#define RECHARGEFACTOR_DEF	197

#define as364x_max_flash_cap_size	(sizeof(u32) \
				+ (sizeof(struct nvc_torch_level_info) \
				* (AS364X_MAX_FLASH_LEVEL)))
#define as364x_max_torch_cap_size	(sizeof(u32) \
				+ (sizeof(s32) * (AS364X_MAX_TORCH_LEVEL)))

struct as364x_caps_struct {
	char *name;
	u32 curr_step_uA;
	u32 curr_step_boost_uA;
	u32 txmask_step_uA;
	u32 txmask_step_boost_uA;
	u32 num_regs;
	u32 max_peak_curr_mA;
	u32 min_ilimit_mA;
	u32 max_assist_curr_mA;
	u32 max_indicator_curr_mA;
	bool led2_support;
};

struct as364x_reg_cache {
	u8 dev_id;
	u8 led1_curr;
	u8 led2_curr;
	u8 txmask;
	u8 strobe;
	u8 ftime;
	u8 vlow;
	u8 pwm_ind;
};

static const struct as364x_caps_struct as364x_caps[] = {
	{"as3643", 5098, 0, 81600, 0, 11, 1300, 1000,
		AS364X_MAX_ASSIST_CURRENT(5098),
		AS364X_MAX_INDICATOR_CURRENT(5098), false},
	{"as3647", 6274, 0, 100400, 0, 11, 1600, 2000,
		AS364X_MAX_ASSIST_CURRENT(6274),
		AS364X_MAX_INDICATOR_CURRENT(6274), false},
	{"as3648", 3529, 3921, 56467, 62747, 14, 1000, 2000,
		AS364X_MAX_ASSIST_CURRENT(3529),
		AS364X_MAX_INDICATOR_CURRENT(3529), true},
};

/* translated from the default register values after power up */
const struct as364x_config default_cfg = {
	.use_tx_mask = 0,
	.I_limit_mA = 3000,
	.txmasked_current_mA = 339,
	.vin_low_v_run_mV = 3220,
	.vin_low_v_mV = 3300,
	.strobe_type = 2,
	.freq_switch_on = 0,
	.led_off_when_vin_low = 0,
	.max_peak_current_mA = 900,
	.max_sustained_current_mA = 0,
	.max_peak_duration_ms = 0,
	.min_current_mA = 0,
};

struct as364x_info {
	struct i2c_client *i2c_client;
	struct miscdevice miscdev;
	struct dentry *d_as364x;
	struct list_head list;
	struct as364x_info *s_info;
	struct mutex mutex;
	struct regulator *v_in;
	struct as364x_power_rail power;
	struct as364x_platform_data *pdata;
	struct nvc_torch_flash_capabilities *flash_cap;
	struct nvc_torch_torch_capabilities *torch_cap;
	struct as364x_caps_struct caps;
	struct as364x_config config;
	struct as364x_reg_cache regs;
	atomic_t in_use;
	int flash_cap_size;
	int torch_cap_size;
	int pwr_state;
	u8 s_mode;
	u8 flash_mode;
	u8 led_num;
	u8 led_mask;
};

static struct as364x_platform_data as364x_default_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "torch",
	.pinstate	= {0x0000, 0x0000},
	.led_mask	= 3,
};

static const struct i2c_device_id as364x_id[] = {
	{ "as364x", 0 },
	{ "as3643", 0 },
	{ "as3647", 0 },
	{ "as3648", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, as364x_id);

static LIST_HEAD(as364x_info_list);
static DEFINE_SPINLOCK(as364x_spinlock);

static const u16 v_in_low[] = {0, 3000, 3070, 3140, 3220, 3300, 3338, 3470};

static int as364x_debugfs_init(struct as364x_info *info);

static int as364x_reg_rd(struct as364x_info *info, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];

	*val = 0;
	msg[0].addr = info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = val;
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2)
		return -EIO;

	return 0;
}

static int as364x_reg_raw_wr(struct as364x_info *info, u8 *buf, u8 num)
{
	struct i2c_msg msg;

	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = num;
	msg.buf = buf;
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1)
		return -EIO;

	dev_dbg(&info->i2c_client->dev, "%s %x %x\n", __func__, buf[0], buf[1]);
	return 0;
}

static int as364x_reg_wr(struct as364x_info *info, u8 reg, u8 val)
{
	u8 buf[2];

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	buf[0] = reg;
	buf[1] = val;
	return as364x_reg_raw_wr(info, buf, sizeof(buf));
}

static int as364x_set_leds(struct as364x_info *info,
			u8 mask, u8 curr1, u8 curr2)
{
	int err = 0;
	u8 regs[7];

	if (mask & 1) {
		if (info->flash_mode == AS364X_REG_CONTROL_MODE_FLASH) {
			if (curr1 >= info->flash_cap->numberoflevels)
				curr1 = info->flash_cap->numberoflevels - 1;
		} else {
			if (curr1 >= info->torch_cap->numberoflevels)
				curr1 = info->torch_cap->numberoflevels - 1;
		}
	} else
		curr1 = 0;

	if (mask & 2 && info->caps.led2_support) {
		if (info->flash_mode == AS364X_REG_CONTROL_MODE_FLASH) {
			if (curr2 >= info->flash_cap->numberoflevels)
				curr2 = info->flash_cap->numberoflevels - 1;
		} else {
			if (curr2 >= info->torch_cap->numberoflevels)
				curr2 = info->torch_cap->numberoflevels - 1;
		}
	} else
		curr2 = 0;

	regs[0] = AS364X_REG_LED1_SET_CURR;
	regs[1] = curr1;
	regs[2] = curr2;
	regs[3] = info->regs.txmask;
	regs[4] = info->regs.vlow;
	regs[5] = info->regs.ftime;
	if (mask == 0 || (curr1 == 0 && curr2 == 0))
		regs[6] = info->flash_mode & (~0x08);
	else
		regs[6] = info->flash_mode | 0x08;
	err = as364x_reg_raw_wr(info, regs, sizeof(regs));
	if (!err) {
		info->regs.led1_curr = curr1;
		info->regs.led2_curr = curr2;
	}

	dev_dbg(&info->i2c_client->dev, "%s %x %x %x %x control = %x\n",
			__func__, mask, curr1, curr2,
			info->regs.ftime, regs[6]);
	return err;
}

static int as364x_set_txmask(struct as364x_info *info)
{
	struct as364x_caps_struct *p_cap = &info->caps;
	struct as364x_config *p_cfg = &info->config;
	int err;
	u8 tm;
	u32 limit = 0, txmask;

	tm = p_cfg->use_tx_mask ? 1 : 0;

	if (p_cfg->I_limit_mA > p_cap->min_ilimit_mA)
		limit = (p_cfg->I_limit_mA - p_cap->min_ilimit_mA) / 500;

	if (limit > 3)
		limit = 3;
	tm |= limit<<2;

	txmask = p_cfg->txmasked_current_mA * 1000;

	if (p_cfg->boost_mode)
		txmask /= p_cap->txmask_step_boost_uA;
	else
		txmask /= p_cap->txmask_step_uA;

	if (txmask > 0xf)
		txmask = 0xf;

	tm |= txmask<<4;

	err = as364x_reg_wr(info, AS364X_REG_TXMASK, tm);
	if (!err)
		info->regs.txmask = tm;

	return err;
}

static int as364x_get_vin_index(u16 mV)
{
	int vin;

	for (vin = ARRAY_SIZE(v_in_low) - 1; vin >= 0; vin--) {
		if (mV >= v_in_low[vin])
			break;
	}

	return vin;
}

static void as364x_config_init(struct as364x_info *info)
{
	struct as364x_config *pcfg = &info->config;
	struct as364x_config *pcfg_cust = &info->pdata->config;

	memcpy(pcfg, &default_cfg, sizeof(info->config));

	pcfg->use_tx_mask = pcfg_cust->use_tx_mask;
	pcfg->freq_switch_on = pcfg_cust->freq_switch_on;
	pcfg->inct_pwm = pcfg_cust->inct_pwm;
	pcfg->load_balance_on = pcfg_cust->load_balance_on;
	pcfg->led_off_when_vin_low = pcfg_cust->led_off_when_vin_low;
	pcfg->boost_mode = pcfg_cust->boost_mode;

	if (pcfg_cust->strobe_type)
		pcfg->strobe_type = pcfg_cust->strobe_type;

	if (pcfg_cust->vin_low_v_run_mV) {
		if (pcfg_cust->vin_low_v_run_mV == 0xffff)
			pcfg->vin_low_v_run_mV = 0;
		else
			pcfg->vin_low_v_run_mV = pcfg_cust->vin_low_v_run_mV;
	}

	if (pcfg_cust->vin_low_v_mV) {
		if (pcfg_cust->vin_low_v_mV == 0xffff)
			pcfg->vin_low_v_mV = 0;
		else
			pcfg->vin_low_v_mV = pcfg_cust->vin_low_v_mV;
	}

	if (pcfg_cust->I_limit_mA)
		pcfg->I_limit_mA = pcfg_cust->I_limit_mA;

	if (pcfg_cust->txmasked_current_mA)
		pcfg->txmasked_current_mA = pcfg_cust->txmasked_current_mA;

	if (pcfg_cust->max_total_current_mA)
		pcfg->max_total_current_mA = pcfg_cust->max_total_current_mA;

	if (pcfg_cust->max_peak_current_mA)
		pcfg->max_peak_current_mA = pcfg_cust->max_peak_current_mA;

	if (pcfg_cust->max_peak_duration_ms)
		pcfg->max_peak_duration_ms = pcfg_cust->max_peak_duration_ms;

	if (pcfg_cust->max_sustained_current_mA)
		pcfg->max_sustained_current_mA =
			pcfg_cust->max_sustained_current_mA;

	if (pcfg_cust->min_current_mA)
		pcfg->min_current_mA = pcfg_cust->min_current_mA;

}

static int as364x_update_settings(struct as364x_info *info)
{
	int err;

	err = as364x_set_txmask(info);

	err |= as364x_reg_wr(info, AS364X_REG_LOWVOLTAGE, info->regs.vlow);

	err |= as364x_reg_wr(info, AS364X_REG_PWM_INDICATOR,
			info->regs.pwm_ind);

	err |= as364x_reg_wr(info, AS364X_REG_STROBE, info->regs.strobe);

	if (info->caps.led2_support) {
		err |= as364x_reg_wr(info, AS364X_REG_PASSWORD, 0xa1);
		if (info->config.boost_mode)
			err |= as364x_reg_wr(info, AS364X_REG_CURR_BOOST, 1);
		else
			err |= as364x_reg_wr(info, AS364X_REG_CURR_BOOST, 0);
	}

	err |= as364x_set_leds(info,
		info->led_mask, info->regs.led1_curr, info->regs.led2_curr);

	dev_dbg(&info->i2c_client->dev, "UP: strobe: %x pwm_ind: %x vlow: %x\n",
		info->regs.strobe, info->regs.pwm_ind, info->regs.vlow);
	return err;
}

static int as364x_configure(struct as364x_info *info, bool update)
{
	struct as364x_config *pcfg = &info->config;
	struct as364x_caps_struct *pcap = &info->caps;
	struct nvc_torch_flash_capabilities *pfcap = info->flash_cap;
	struct nvc_torch_torch_capabilities *ptcap = info->torch_cap;
	int val;
	int i;

	if (!pcap->led2_support)
		pcfg->boost_mode = false;

	val = as364x_get_vin_index(pcfg->vin_low_v_run_mV);
	info->regs.vlow = val<<0;

	val = as364x_get_vin_index(pcfg->vin_low_v_mV);
	info->regs.vlow |= val<<3;

	if (pcfg->led_off_when_vin_low)
		info->regs.vlow |= 0x40;

	info->regs.pwm_ind = pcfg->inct_pwm & 0x03;
	if (pcfg->freq_switch_on)
		info->regs.pwm_ind |= 0x04;
	if (pcfg->load_balance_on)
		info->regs.pwm_ind |= 0x20;

	info->regs.strobe = pcfg->strobe_type == 2 ? 0xc0 : 0x80;
	info->led_mask = info->pdata->led_mask;

	info->regs.ftime = DEFAULT_FLASHTIME;

	if (pcfg->max_peak_current_mA > pcap->max_peak_curr_mA ||
		!pcfg->max_peak_current_mA) {
		dev_warn(&info->i2c_client->dev,
				"max_peak_current_mA of %d invalid"
				"changing to %d\n",
				pcfg->max_peak_current_mA,
				pcap->max_peak_curr_mA);
		pcfg->max_peak_current_mA = pcap->max_peak_curr_mA;
	}

	info->led_num = 1;
	if (pcap->led2_support && (info->led_mask & 3) == 3)
		info->led_num = 2;
	val = pcfg->max_peak_current_mA * info->led_num;

	if (!pcfg->max_total_current_mA || pcfg->max_total_current_mA > val)
		pcfg->max_total_current_mA = val;
	pcfg->max_peak_current_mA =
		info->config.max_total_current_mA / info->led_num;

	if (pcfg->max_sustained_current_mA > pcap->max_assist_curr_mA ||
		!pcfg->max_sustained_current_mA) {
		dev_warn(&info->i2c_client->dev,
				"max_sustained_current_mA of %d invalid"
				"changing to %d\n",
				pcfg->max_sustained_current_mA,
				pcap->max_assist_curr_mA);
		pcfg->max_sustained_current_mA =
			pcap->max_assist_curr_mA;
	}
	if ((1000 * pcfg->min_current_mA) < pcap->curr_step_uA) {
		pcfg->min_current_mA = pcap->curr_step_uA / 1000;
		dev_warn(&info->i2c_client->dev,
				"min_current_mA lower than possible, icreasing"
				" to %d\n",
				pcfg->min_current_mA);
	}
	if (pcfg->min_current_mA > pcap->max_indicator_curr_mA) {
		dev_warn(&info->i2c_client->dev,
				"min_current_mA of %d higher than possible,"
				" reducing to %d",
				pcfg->min_current_mA,
				pcap->max_indicator_curr_mA);
		pcfg->min_current_mA =
			pcap->max_indicator_curr_mA;
	}

	if (pcfg->boost_mode)
		val = pcap->curr_step_uA;
	else
		val = pcap->curr_step_boost_uA;
	for (i = 0; i < AS364X_MAX_FLASH_LEVEL; i++) {
		pfcap->levels[i].guidenum = val * i / 1000;
		if (pfcap->levels[i].guidenum >
			pcfg->max_peak_current_mA) {
			pfcap->levels[i].guidenum = 0;
			break;
		}
		pfcap->levels[i].sustaintime = SUSTAINTIME_DEF;
		pfcap->levels[i].rechargefactor = RECHARGEFACTOR_DEF;
	}
	info->flash_cap_size = (sizeof(u32) +
			(sizeof(struct nvc_torch_level_info) * i));
	pfcap->numberoflevels = i;

	for (i = 0; i < AS364X_MAX_TORCH_LEVEL; i++) {
		ptcap->guidenum[i] = pfcap->levels[i].guidenum;
		if (ptcap->guidenum[i] > pcfg->max_peak_current_mA) {
			ptcap->guidenum[i] = 0;
			break;
		}
	}
	info->torch_cap_size = (sizeof(u32) + (sizeof(s32) * i));
	ptcap->numberoflevels = i;

	if (update && (info->pwr_state == NVC_PWR_COMM ||
			info->pwr_state == NVC_PWR_ON))
		return as364x_update_settings(info);

	return 0;
}

static int as364x_strobe(struct as364x_info *info, int t_on)
{
	u32 gpio = info->pdata->gpio_strobe & 0xffff;
	u32 lact = (info->pdata->gpio_strobe & 0xffff0000) ? 1 : 0;
	return gpio_direction_output(gpio, lact ^ (t_on & 1));
}

#ifdef CONFIG_PM
static int as364x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct as364x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Suspending %s\n", info->caps.name);

	return 0;
}

static int as364x_resume(struct i2c_client *client)
{
	struct as364x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Resuming %s\n", info->caps.name);

	return 0;
}

static void as364x_shutdown(struct i2c_client *client)
{
	struct as364x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Shutting down %s\n", info->caps.name);

	mutex_lock(&info->mutex);
	as364x_set_leds(info, 3, 0, 0);
	mutex_unlock(&info->mutex);
}
#endif

static int as364x_power_on(struct as364x_info *info)
{
	struct as364x_power_rail *power = &info->power;
	int err = 0;

	if (power->v_in) {
		err = regulator_enable(power->v_in);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s v_in err\n",
				__func__);
			return err;
		}
	}

	if (power->v_i2c) {
		err = regulator_enable(power->v_i2c);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s v_i2c err\n",
				__func__);
			regulator_disable(power->v_in);
			return err;
		}
	}

	if (info->pdata && info->pdata->power_on_callback)
		err = info->pdata->power_on_callback(&info->power);

	if (!err) {
		usleep_range(100, 120);
		err = as364x_update_settings(info);
	}
	return err;
}

static int as364x_power_off(struct as364x_info *info)
{
	struct as364x_power_rail *power = &info->power;
	int err = 0;

	if (info->pdata && info->pdata->power_off_callback)
		err = info->pdata->power_off_callback(&info->power);
	if (IS_ERR_VALUE(err))
		return err;

	if (power->v_in) {
		err = regulator_disable(power->v_in);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s vi_in err\n",
				__func__);
			return err;
		}
	}

	if (power->v_i2c) {
		err = regulator_disable(power->v_i2c);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s vi_i2c err\n",
				__func__);
			return err;
		}
	}

	return 0;
}

static int as364x_power(struct as364x_info *info, int pwr)
{
	int err = 0;

	dev_dbg(&info->i2c_client->dev, "%s %d %d\n",
		__func__, pwr, info->pwr_state);
	if (pwr == info->pwr_state) /* power state no change */
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF:
		err = as364x_set_leds(info, 3, 0, 0);
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err |= as364x_power_off(info);
		break;
	case NVC_PWR_STDBY_OFF:
		err = as364x_set_leds(info, 3, 0, 0);
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			     (info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err |= as364x_power_on(info);
		break;
	case NVC_PWR_STDBY:
		err = as364x_power_on(info);
		err |= as364x_set_leds(info, 3, 0, 0);
		break;
	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = as364x_power_on(info);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(&info->i2c_client->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_state = pwr;
	if (err > 0)
		return 0;

	return err;
}

static int as364x_power_sync(struct as364x_info *info, int pwr)
{
	int err1 = 0;
	int err2 = 0;

	if ((info->s_mode == NVC_SYNC_OFF) ||
		(info->s_mode == NVC_SYNC_MASTER) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err1 = as364x_power(info, pwr);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err2 = as364x_power(info->s_info, pwr);
	return err1 | err2;
}

static int as364x_get_dev_id(struct as364x_info *info)
{
	int err;

	/* ChipID[7:3] is a fixed identification B0 */
	if ((info->regs.dev_id & 0xb0) == 0xb0)
		return 0;

	if (NVC_PWR_OFF == info->pwr_state ||
		NVC_PWR_OFF_FORCE == info->pwr_state)
		as364x_power_on(info);
	err = as364x_reg_rd(info, AS364X_REG_CHIPID, &info->regs.dev_id);
	if (err)
		goto read_devid_exit;

	if ((info->regs.dev_id & 0xb0) != 0xb0)
		err = -ENODEV;

read_devid_exit:
	if (NVC_PWR_OFF == info->pwr_state)
		as364x_power_off(info);

	return err;
}

static int as364x_user_get_param(struct as364x_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_pin_state pinstate;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	u8 reg;

	if (copy_from_user(&params,
			(const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (info->s_mode == NVC_SYNC_SLAVE)
		info = info->s_info;
	switch (params.param) {
	case NVC_PARAM_FLASH_CAPS:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_CAPS\n", __func__);
		data_ptr = info->flash_cap;
		data_size = info->flash_cap_size;
		break;
	case NVC_PARAM_FLASH_LEVEL:
		reg = info->regs.led1_curr;
		data_ptr = &info->flash_cap->levels[reg].guidenum;
		data_size = sizeof(info->flash_cap->levels[reg].guidenum);
		break;
	case NVC_PARAM_TORCH_CAPS:
		dev_dbg(&info->i2c_client->dev, "%s TORCH_CAPS\n", __func__);
		data_ptr = info->torch_cap;
		data_size = info->torch_cap_size;
		break;
	case NVC_PARAM_TORCH_LEVEL:
		reg = info->regs.led1_curr;
		data_ptr = &info->torch_cap->guidenum[reg];
		data_size = sizeof(info->torch_cap->guidenum[reg]);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		/* By default use Active Pin State Setting */
		pinstate = info->pdata->pinstate;
		if ((info->flash_mode != AS364X_REG_CONTROL_MODE_FLASH) ||
		    (!info->regs.led1_curr && !info->regs.led2_curr))
			pinstate.values ^= 0xffff; /* Inactive Pin Setting */

		dev_dbg(&info->i2c_client->dev, "%s FLASH_PIN_STATE: %x&%x\n",
				__func__, pinstate.mask, pinstate.values);
		data_ptr = &pinstate;
		data_size = sizeof(pinstate);
		break;
	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
				__func__, info->s_mode);
		data_ptr = &info->s_mode;
		data_size = sizeof(info->s_mode);
		break;
	default:
		dev_err(&info->i2c_client->dev,
				"%s unsupported parameter: %d\n",
				__func__, params.param);
		return -EINVAL;
	}

	if (params.sizeofvalue < data_size) {
		dev_err(&info->i2c_client->dev,
				"%s data size mismatch %d != %d\n",
				__func__, params.sizeofvalue, data_size);
		return -EINVAL;
	}

	if (copy_to_user((void __user *)params.p_value,
			 data_ptr,
			 data_size)) {
		dev_err(&info->i2c_client->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

static int as364x_set_param(struct as364x_info *info,
			       struct nvc_param *params,
			       u8 val)
{
	int err;

	switch (params->param) {
	case NVC_PARAM_FLASH_LEVEL:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_LEVEL: %d\n",
				__func__, val);

		info->flash_mode = AS364X_REG_CONTROL_MODE_FLASH;
		err = as364x_set_leds(info, info->led_mask, val, val);
		if (!val)
			info->flash_mode = AS364X_REG_CONTROL_MODE_ASSIST;
		return err;
	case NVC_PARAM_TORCH_LEVEL:
		dev_dbg(&info->i2c_client->dev, "%s TORCH_LEVEL: %d\n",
				__func__, val);
		info->flash_mode = AS364X_REG_CONTROL_MODE_ASSIST;
		err = as364x_set_leds(info, info->led_mask, val, val);
		return err;
	case NVC_PARAM_FLASH_PIN_STATE:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_PIN_STATE: %d\n",
				__func__, val);
		return as364x_strobe(info, val);
	default:
		dev_err(&info->i2c_client->dev,
				"%s unsupported parameter: %d\n",
				__func__, params->param);
		return -EINVAL;
	}
}

static int as364x_user_set_param(struct as364x_info *info, long arg)
{
	struct nvc_param params;
	u8 val;
	int err = 0;

	if (copy_from_user(&params,
				(const void __user *)arg,
				sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (copy_from_user(&val, (const void __user *)params.p_value,
			   sizeof(val))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	/* parameters independent of sync mode */
	switch (params.param) {
	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
				__func__, (int)val);
		if (val == info->s_mode)
			return 0;

		switch (val) {
		case NVC_SYNC_OFF:
			info->s_mode = val;
			if (info->s_info != NULL) {
				info->s_info->s_mode = val;
				as364x_power(info->s_info, NVC_PWR_OFF);
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
				info->s_info->pwr_state = info->pwr_state;
				err = as364x_power(info->s_info,
						     info->pwr_state);
				if (!err) {
					info->s_mode = val;
					info->s_info->s_mode = val;
				} else {
					as364x_power(info->s_info,
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
			return as364x_set_param(info, &params, val);
		case NVC_SYNC_SLAVE:
			return as364x_set_param(info->s_info, &params, val);
		case NVC_SYNC_STEREO:
			err = as364x_set_param(info, &params, val);
			if (!(info->pdata->cfg & NVC_CFG_SYNC_I2C_MUX))
				err |= as364x_set_param(info->s_info,
						&params, val);
			return err;
		default:
			dev_err(&info->i2c_client->dev, "%s %d internal err\n",
					__func__, __LINE__);
			return -EINVAL;
		}
	}
}

static long as364x_ioctl(struct file *file,
			   unsigned int cmd,
			   unsigned long arg)
{
	struct as364x_info *info = file->private_data;
	int pwr;
	int err;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		return as364x_user_set_param(info, arg);
	case NVC_IOCTL_PARAM_RD:
		return as364x_user_get_param(info, arg);
	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_WR: %d\n",
			__func__, pwr);
		if (!pwr || (pwr > NVC_PWR_ON)) /* Invalid Power State */
			return 0;

		err = as364x_power_sync(info, pwr);

		if (info->pdata->cfg & NVC_CFG_NOERR)
			return 0;
		return err;
	case NVC_IOCTL_PWR_RD:
		if (info->s_mode == NVC_SYNC_SLAVE)
			pwr = info->s_info->pwr_state / 2;
		else
			pwr = info->pwr_state / 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_RD: %d\n",
				__func__, pwr);
		if (copy_to_user((void __user *)arg, (const void *)&pwr,
				 sizeof(pwr))) {
			dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
			return -EFAULT;
		}

		return 0;
	default:
		dev_err(&info->i2c_client->dev, "%s unsupported ioctl: %x\n",
				__func__, cmd);
		return -EINVAL;
	}
}

static int as364x_sync_en(int dev1, int dev2)
{
	struct as364x_info *sync1 = NULL;
	struct as364x_info *sync2 = NULL;
	struct as364x_info *pos = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &as364x_info_list, list) {
		if (pos->pdata->num == dev1) {
			sync1 = pos;
			break;
		}
	}
	pos = NULL;
	list_for_each_entry_rcu(pos, &as364x_info_list, list) {
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

static int as364x_sync_dis(struct as364x_info *info)
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

static int as364x_open(struct inode *inode, struct file *file)
{
	struct as364x_info *info = NULL;
	struct as364x_info *pos = NULL;
	int err;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &as364x_info_list, list) {
		if (pos->miscdev.minor == iminor(inode)) {
			info = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (!info)
		return -ENODEV;

	err = as364x_sync_en(info->pdata->num, info->pdata->sync);
	if (err == -EINVAL)
		dev_err(&info->i2c_client->dev,
			 "%s err: invalid num (%u) and sync (%u) instance\n",
			 __func__, info->pdata->num, info->pdata->sync);
	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	if (info->s_info != NULL) {
		if (atomic_xchg(&info->s_info->in_use, 1))
			return -EBUSY;
	}

	file->private_data = info;
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	return 0;
}

static int as364x_release(struct inode *inode, struct file *file)
{
	struct as364x_info *info = file->private_data;

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	as364x_power_sync(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	if (info->s_info != NULL)
		WARN_ON(!atomic_xchg(&info->s_info->in_use, 0));
	as364x_sync_dis(info);
	return 0;
}

static int as364x_power_put(struct as364x_power_rail *pw)
{
	if (likely(pw->v_in))
		regulator_put(pw->v_in);

	if (likely(pw->v_i2c))
		regulator_put(pw->v_i2c);

	pw->v_in = NULL;
	pw->v_i2c = NULL;

	return 0;
}

static int as364x_regulator_get(struct as364x_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR_OR_NULL(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int as364x_power_get(struct as364x_info *info)
{
	struct as364x_power_rail *pw = &info->power;

	as364x_regulator_get(info, &pw->v_in, "vin"); /* 3.7v */
	as364x_regulator_get(info, &pw->v_i2c, "vi2c"); /* 1.8v */
	info->pwr_state = NVC_PWR_OFF;

	return 0;
}

static const struct file_operations as364x_fileops = {
	.owner = THIS_MODULE,
	.open = as364x_open,
	.unlocked_ioctl = as364x_ioctl,
	.release = as364x_release,
};

static void as364x_del(struct as364x_info *info)
{
	as364x_power_sync(info, NVC_PWR_OFF);
	as364x_power_put(&info->power);

	as364x_sync_dis(info);
	spin_lock(&as364x_spinlock);
	list_del_rcu(&info->list);
	spin_unlock(&as364x_spinlock);
	synchronize_rcu();
}

static int as364x_remove(struct i2c_client *client)
{
	struct as364x_info *info = i2c_get_clientdata(client);

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	misc_deregister(&info->miscdev);
	as364x_del(info);
	return 0;
}

static int as364x_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct as364x_info *info;
	char dname[16];
	int err;

	dev_dbg(&client->dev, "%s\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(*info) +
			as364x_max_flash_cap_size + as364x_max_torch_cap_size,
			GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->i2c_client = client;
	if (client->dev.platform_data)
		info->pdata = client->dev.platform_data;
	else {
		info->pdata = &as364x_default_pdata;
		dev_dbg(&client->dev,
				"%s No platform data.  Using defaults.\n",
				__func__);
	}

	info->flash_cap = (void *)info + sizeof(*info);
	info->torch_cap = (void *)info->flash_cap + as364x_max_flash_cap_size;
	memcpy(&info->caps, &as364x_caps[info->pdata->type],
		sizeof(info->caps));

	as364x_config_init(info);

	info->flash_mode = AS364X_REG_CONTROL_MODE_ASSIST; /* torch mode */

	as364x_configure(info, false);

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);
	INIT_LIST_HEAD(&info->list);
	spin_lock(&as364x_spinlock);
	list_add_rcu(&info->list, &as364x_info_list);
	spin_unlock(&as364x_spinlock);

	as364x_power_get(info);

	err = as364x_get_dev_id(info);
	if (err < 0) {
		dev_err(&client->dev, "%s device not found\n", __func__);
		if (info->pdata->cfg & NVC_CFG_NODEV) {
			as364x_del(info);
			return -ENODEV;
		}
	} else
		dev_info(&client->dev, "%s device %02x found\n",
			__func__, info->regs.dev_id);

	if (info->pdata->dev_name != 0)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "as364x");
	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
			 dname, info->pdata->num);

	info->miscdev.name = dname;
	info->miscdev.fops = &as364x_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
				__func__, dname);
		as364x_del(info);
		return -ENODEV;
	}

	as364x_debugfs_init(info);
	return 0;
}

static int as364x_status_show(struct seq_file *s, void *data)
{
	struct as364x_info *k_info = s->private;
	struct as364x_config *pcfg = &k_info->config;

	pr_info("%s\n", __func__);

	seq_printf(s, "as364x status:\n"
		"    Flash type: %s, bus %d, addr: 0x%02x\n\n"
		"    Led Mask         = %01x\n"
		"    Led1 Current     = 0x%02x\n"
		"    Led2 Current     = 0x%02x\n"
		"    Flash Mode       = 0x%02x\n"
		"    Flash TimeOut    = 0x%02x\n"
		"    Flash Strobe     = 0x%02x\n"
		"    Max_Peak_Current = 0x%04dmA\n"
		"    Use_TxMask       = 0x%02x\n"
		"    TxMask_Current   = 0x%04dmA\n"
		"    Freq_Switch_on   = %s\n"
		"    VIN_low_run      = 0x%04dmV\n"
		"    VIN_low          = 0x%04dmV\n"
		"    LedOff_On_VIN_low = %s\n"
		"    PinState Mask    = 0x%04x\n"
		"    PinState Values  = 0x%04x\n"
		,
		(char *)as364x_id[k_info->pdata->type + 1].name,
		k_info->i2c_client->adapter->nr,
		k_info->i2c_client->addr,
		k_info->led_mask,
		k_info->regs.led1_curr,
		k_info->regs.led2_curr,
		k_info->flash_mode, k_info->regs.ftime,
		pcfg->strobe_type,
		pcfg->max_peak_current_mA,
		pcfg->use_tx_mask,
		pcfg->txmasked_current_mA,
		pcfg->freq_switch_on ? "TRUE" : "FALSE",
		pcfg->vin_low_v_run_mV,
		pcfg->vin_low_v_mV,
		pcfg->led_off_when_vin_low ? "TRUE" : "FALSE",
		k_info->pdata->pinstate.mask,
		k_info->pdata->pinstate.values
	);

	return 0;
}

static ssize_t as364x_attr_set(struct file *s,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct as364x_info *k_info =
		((struct seq_file *)s->private_data)->private;
	char buf[24];
	int buf_size;
	u32 val = 0;

	pr_info("%s\n", __func__);

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

	pr_info("SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	pr_info("new data = %x\n", val);
	switch (buf[0]) {
	case 'p':
		if (val & 0xffff)
			as364x_power(k_info, NVC_PWR_ON);
		else
			as364x_power(k_info, NVC_PWR_OFF);
		break;
	case 'c': /* change led 1/2 current settings */
		as364x_set_leds(k_info, k_info->led_mask,
			val & 0xff, (val >> 8) & 0xff);
		break;
	case 'l': /* enable/disable led 1/2 */
		k_info->pdata->led_mask = val;
		as364x_configure(k_info, false);
		break;
	case 'm': /* change pinstate setting */
		k_info->pdata->pinstate.mask = (val >> 16) & 0xffff;
		k_info->pdata->pinstate.values = val & 0xffff;
		break;
	case 'f': /* modify flash timeout reg */
		k_info->regs.ftime = val;
		as364x_set_leds(k_info, k_info->led_mask,
			k_info->regs.led1_curr,
			k_info->regs.led2_curr);
		break;
	case 't': /* change txmask/torch settings */
		k_info->config.use_tx_mask = (val >> 4) & 1;
		k_info->config.txmasked_current_mA = val & 0x0f;
		val = (val >> 8) & 0xffff;
		if (val)
			k_info->config.I_limit_mA = val;
		as364x_set_txmask(k_info);
		break;
	case 'v':
		if (val & 0xffff)
			k_info->config.vin_low_v_run_mV = val & 0xffff;
		val >>= 16;
		if (val & 0xffff)
			k_info->config.vin_low_v_mV = val & 0xffff;
		as364x_configure(k_info, true);
		break;
	case 'k':
		if (val & 0xffff)
			k_info->config.max_peak_current_mA = val & 0xffff;
		as364x_configure(k_info, true);
		break;
	case 'x':
		if (val & 0xf)
			k_info->flash_mode = (val & 0xf) - 1;
		if (val & 0xf0)
			k_info->config.strobe_type = ((val & 0xf0) >> 4) - 1;
		if (val & 0xf00)
			k_info->config.freq_switch_on =
				((val & 0xf00) == 0x200);
		if (val & 0xf000)
			k_info->config.led_off_when_vin_low =
				((val & 0xf000) == 0x2000);
		if (val & 0xf0000) {
			val = ((val & 0xf0000) >> 16) - 1;
			if (val >= AS364X_NUM) {
				pr_err("Invalid dev type %x\n", val);
				return -ENODEV;
			}
			k_info->pdata->type = val;
			memcpy(&k_info->caps,
				&as364x_caps[k_info->pdata->type],
				sizeof(k_info->caps));
		}
		as364x_configure(k_info, true);
		break;
	case 'g':
		k_info->pdata->gpio_strobe = val;
		as364x_strobe(k_info, 1);
		break;
	}

	return count;
}

static int as364x_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, as364x_status_show, inode->i_private);
}

static const struct file_operations as364x_debugfs_fops = {
	.open = as364x_debugfs_open,
	.read = seq_read,
	.write = as364x_attr_set,
	.llseek = seq_lseek,
	.release = single_release,
};

static int as364x_debugfs_init(struct as364x_info *info)
{
	struct dentry *d;

	info->d_as364x = debugfs_create_dir(
		info->miscdev.this_device->kobj.name, NULL);
	if (info->d_as364x == NULL) {
		pr_info("%s: debugfs create dir failed\n", __func__);
		return -ENOMEM;
	}

	d = debugfs_create_file("d", S_IRUGO|S_IWUSR, info->d_as364x,
		(void *)info, &as364x_debugfs_fops);
	if (!d) {
		pr_info("%s: debugfs create file failed\n", __func__);
		debugfs_remove_recursive(info->d_as364x);
		info->d_as364x = NULL;
	}

	return -EFAULT;
}

static struct i2c_driver as364x_driver = {
	.driver = {
		.name = "as364x",
		.owner = THIS_MODULE,
	},
	.id_table = as364x_id,
	.probe = as364x_probe,
	.remove = as364x_remove,
#ifdef CONFIG_PM
	.shutdown = as364x_shutdown,
	.suspend  = as364x_suspend,
	.resume   = as364x_resume,
#endif
};

module_i2c_driver(as364x_driver);

MODULE_DESCRIPTION("AS364x flash/torch driver");
MODULE_AUTHOR("Charlie Huang <chahuang@nvidia.com>");
MODULE_LICENSE("GPL");
