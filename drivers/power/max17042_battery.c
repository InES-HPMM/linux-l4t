/*
 * Fuel gauge driver for Maxim 17042 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver is based on max17040_battery.c
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/edp.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/of.h>

/* Status register bits */
#define STATUS_POR_BIT         (1 << 1)
#define STATUS_BST_BIT         (1 << 3)
#define STATUS_VMN_BIT         (1 << 8)
#define STATUS_TMN_BIT         (1 << 9)
#define STATUS_SMN_BIT         (1 << 10)
#define STATUS_BI_BIT          (1 << 11)
#define STATUS_VMX_BIT         (1 << 12)
#define STATUS_TMX_BIT         (1 << 13)
#define STATUS_SMX_BIT         (1 << 14)
#define STATUS_BR_BIT          (1 << 15)

/* Interrupt mask bits */
#define CONFIG_ALRT_BIT_ENBL	(1 << 2)
#define STATUS_INTR_SOCMIN_BIT	(1 << 10)
#define STATUS_INTR_SOCMAX_BIT	(1 << 14)

#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK		0x0080
#define MODEL_UNLOCK1	0X0059
#define MODEL_UNLOCK2	0X00C4
#define MODEL_LOCK1		0X0000
#define MODEL_LOCK2		0X0000

#define dQ_ACC_DIV	0x4
#define dP_ACC_100	0x1900
#define dP_ACC_200	0x3200

#define MAX17042_IC_VERSION	0x0092
#define MAX17047_IC_VERSION	0x00AC	/* same for max17050 */
#define MAX17047_DELAY		1000

/* Battery depletion constants */
#define DEPL_INTERVAL	60000
#define VSYS_MIN	3100000
#define AVG_CURRENT_MIN	100000
#define R_CONTACTS	20000
#define R_BOARD		30000
#define R_PASSFET	30000
#define RBAT_INIT	150000
#define NOMINAL_TEMP	25
#define NOMINAL_VOLTAGE	3800000
#define RBAT_HIST_COUNT	5

struct max17042_chip {
	struct i2c_client *client;
	struct power_supply battery;
	enum max170xx_chip_type chip_type;
	struct max17042_platform_data *pdata;
	struct delayed_work work;
	int    init_complete;
	s64 rbat_lastgood;
	unsigned int edp_req;
	struct delayed_work depl_work;
	int shutdown_complete;
	int status;
	int cap;
	int chgin_ilim;
	struct edp_manager *edp_manager;
};
struct max17042_chip *tmp_chip;
struct i2c_client *temp_client;

static int max17042_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret = 0;
	struct max17042_chip *chip = i2c_get_clientdata(client);

	if (chip && chip->shutdown_complete)
		return -ENODEV;

	ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17042_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = 0;
	struct max17042_chip *chip = i2c_get_clientdata(client);

	if (chip && chip->shutdown_complete)
		return -ENODEV;

	ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void max17042_set_reg(struct i2c_client *client,
			     struct max17042_reg_data *data, int size)
{
	int i;

	for (i = 0; i < size; i++)
		max17042_write_reg(client, data[i].addr, data[i].data);
}

static enum power_supply_property max17042_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_STATUS,
};

int maxim_get_temp()
{
	int ret = 0xff;
	if (temp_client != NULL) {
		ret = max17042_read_reg(temp_client, MAX17042_TEMP);
		if (ret < 0)
			return ret;

		/* The value is signed. */
		if (ret & 0x8000) {
			ret = (0x7fff & ~ret) + 1;
			ret *= -1;
		}
		/* The value is converted into deci-centigrade scale */
		/* Units of LSB = 1 / 256 degree Celsius */
		ret = ret * 10 / 256;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(maxim_get_temp);

void max17042_update_status(int status)
{
	if (!tmp_chip) {
		WARN_ON(1);
		return;
	}

	tmp_chip->status = status;
	power_supply_changed(&tmp_chip->battery);
	if (IS_ENABLED(CONFIG_EDP_FRAMEWORK) &&
			tmp_chip->chgin_ilim != status) {
		tmp_chip->chgin_ilim = status;
		schedule_delayed_work(&tmp_chip->depl_work, 0);
		flush_delayed_work(&tmp_chip->depl_work);
	}
}
EXPORT_SYMBOL_GPL(max17042_update_status);

static int max17042_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17042_chip *chip = container_of(psy,
				struct max17042_chip, battery);
	int ret;

	if (!chip->init_complete)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = max17042_read_reg(chip->client, MAX17042_STATUS);
		if (ret < 0)
			return ret;

		if (ret & MAX17042_STATUS_BattAbsent)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = max17042_read_reg(chip->client, MAX17042_Cycles);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = max17042_read_reg(chip->client, MAX17042_MinMaxVolt);
		if (ret < 0)
			return ret;

		val->intval = ret >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (chip->chip_type == MAX17042)
			ret = max17042_read_reg(chip->client, MAX17042_V_empty);
		else
			ret = max17042_read_reg(chip->client, MAX17047_V_empty);
		if (ret < 0)
			return ret;

		val->intval = ret >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max17042_read_reg(chip->client, MAX17042_VCELL);
		if (ret < 0)
			return ret;

		val->intval = ret * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max17042_read_reg(chip->client, MAX17042_AvgVCELL);
		if (ret < 0)
			return ret;

		val->intval = ret * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = max17042_read_reg(chip->client, MAX17042_OCVInternal);
		if (ret < 0)
			return ret;

		val->intval = ret * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max17042_read_reg(chip->client, MAX17042_RepSOC);
		if (ret < 0)
			return ret;

		val->intval = ret >> 8;
		chip->cap = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = max17042_read_reg(chip->client, MAX17042_FullCAP);
		if (ret < 0)
			return ret;

		val->intval = ret * 1000 / 2;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = max17042_read_reg(chip->client, MAX17042_QH);
		if (ret < 0)
			return ret;

		val->intval = ret * 1000 / 2;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max17042_read_reg(chip->client, MAX17042_TEMP);
		if (ret < 0)
			return ret;

		val->intval = ret;
		/* The value is signed. */
		if (val->intval & 0x8000) {
			val->intval = (0x7fff & ~val->intval) + 1;
			val->intval *= -1;
		}
		/* The value is converted into deci-centigrade scale */
		/* Units of LSB = 1 / 256 degree Celsius */
		val->intval = val->intval * 10 / 256;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->pdata->enable_current_sense) {
			ret = max17042_read_reg(chip->client, MAX17042_Current);
			if (ret < 0)
				return ret;

			val->intval = ret;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 1562500 / chip->pdata->r_sns;
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (chip->pdata->enable_current_sense) {
			ret = max17042_read_reg(chip->client,
						MAX17042_AvgCurrent);
			if (ret < 0)
				return ret;

			val->intval = ret;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 1562500 / chip->pdata->r_sns;
		} else {
			return -EINVAL;
		}
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (chip->status)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		if (chip->cap >= 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17042_write_verify_reg(struct i2c_client *client,
				u8 reg, u16 value)
{
	int retries = 8;
	int ret;
	u16 read_value;

	do {
		ret = max17042_write_reg(client, reg, value);
		read_value =  max17042_read_reg(client, reg);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static inline void max17042_override_por(
	struct i2c_client *client, u8 reg, u16 value)
{
	if (value)
		max17042_write_reg(client, reg, value);
}

static inline void max10742_unlock_model(struct max17042_chip *chip)
{
	struct i2c_client *client = chip->client;
	max17042_write_reg(client, MAX17042_MLOCKReg1, MODEL_UNLOCK1);
	max17042_write_reg(client, MAX17042_MLOCKReg2, MODEL_UNLOCK2);
}

static inline void max10742_lock_model(struct max17042_chip *chip)
{
	struct i2c_client *client = chip->client;
	max17042_write_reg(client, MAX17042_MLOCKReg1, MODEL_LOCK1);
	max17042_write_reg(client, MAX17042_MLOCKReg2, MODEL_LOCK2);
}

static inline void max17042_write_model_data(struct max17042_chip *chip,
					u8 addr, int size)
{
	struct i2c_client *client = chip->client;
	int i;
	for (i = 0; i < size; i++)
		max17042_write_reg(client, addr + i,
				chip->pdata->config_data->cell_char_tbl[i]);
}

static inline void max17042_read_model_data(struct max17042_chip *chip,
					u8 addr, u16 *data, int size)
{
	struct i2c_client *client = chip->client;
	int i;

	for (i = 0; i < size; i++)
		data[i] = max17042_read_reg(client, addr + i);
}

static inline int max17042_model_data_compare(struct max17042_chip *chip,
					u16 *data1, u16 *data2, int size)
{
	int i;

	if (memcmp(data1, data2, size)) {
		dev_err(&chip->client->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(&chip->client->dev, "0x%x, 0x%x",
				data1[i], data2[i]);
		dev_info(&chip->client->dev, "\n");
		return -EINVAL;
	}
	return 0;
}

static int max17042_init_model(struct max17042_chip *chip)
{
	int ret;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u16 *temp_data;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max10742_unlock_model(chip);
	max17042_write_model_data(chip, MAX17042_MODELChrTbl,
				table_size);
	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);

	ret = max17042_model_data_compare(
		chip,
		chip->pdata->config_data->cell_char_tbl,
		temp_data,
		table_size);

	max10742_lock_model(chip);
	kfree(temp_data);

	return ret;
}

static int max17042_verify_model_lock(struct max17042_chip *chip)
{
	int i;
	int table_size = ARRAY_SIZE(chip->pdata->config_data->cell_char_tbl);
	u16 *temp_data;
	int ret = 0;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);
	for (i = 0; i < table_size; i++)
		if (temp_data[i])
			ret = -EINVAL;

	kfree(temp_data);
	return ret;
}

static void max17042_write_config_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;

	max17042_write_reg(chip->client, MAX17042_CONFIG, config->config);
	max17042_write_reg(chip->client, MAX17042_LearnCFG, config->learn_cfg);
	max17042_write_reg(chip->client, MAX17042_FilterCFG,
			config->filter_cfg);
	max17042_write_reg(chip->client, MAX17042_RelaxCFG, config->relax_cfg);
	if (chip->chip_type == MAX17047)
		max17042_write_reg(chip->client, MAX17047_FullSOCThr,
						config->full_soc_thresh);
}

static void  max17042_write_custom_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;

	max17042_write_verify_reg(chip->client, MAX17042_RCOMP0,
				config->rcomp0);
	max17042_write_verify_reg(chip->client, MAX17042_TempCo,
				config->tcompc0);
	max17042_write_verify_reg(chip->client, MAX17042_ICHGTerm,
				config->ichgt_term);
	if (chip->chip_type == MAX17042) {
		max17042_write_reg(chip->client, MAX17042_EmptyTempCo,
					config->empty_tempco);
		max17042_write_verify_reg(chip->client, MAX17042_K_empty0,
					config->kempty0);
	} else {
		max17042_write_verify_reg(chip->client, MAX17047_QRTbl00,
						config->qrtbl00);
		max17042_write_verify_reg(chip->client, MAX17047_QRTbl10,
						config->qrtbl10);
		max17042_write_verify_reg(chip->client, MAX17047_QRTbl20,
						config->qrtbl20);
		max17042_write_verify_reg(chip->client, MAX17047_QRTbl30,
						config->qrtbl30);
	}
}

static void max17042_update_capacity_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->pdata->config_data;

	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
				config->fullcap);
	max17042_write_reg(chip->client, MAX17042_DesignCap,
			config->design_cap);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAPNom,
				config->fullcapnom);
}

static void max17042_reset_vfsoc0_reg(struct max17042_chip *chip)
{
	u16 vfSoc;

	vfSoc = max17042_read_reg(chip->client, MAX17042_VFSOC);
	max17042_write_reg(chip->client, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	max17042_write_verify_reg(chip->client, MAX17042_VFSOC0, vfSoc);
	max17042_write_reg(chip->client, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
}

static void max17042_load_new_capacity_params(struct max17042_chip *chip)
{
	u16 full_cap0, rep_cap, dq_acc, vfSoc;
	u32 rem_cap;

	struct max17042_config_data *config = chip->pdata->config_data;

	full_cap0 = max17042_read_reg(chip->client, MAX17042_FullCAP0);
	vfSoc = max17042_read_reg(chip->client, MAX17042_VFSOC);

	/* fg_vfSoc needs to shifted by 8 bits to get the
	 * perc in 1% accuracy, to get the right rem_cap multiply
	 * full_cap0, fg_vfSoc and devide by 100
	 */
	rem_cap = ((vfSoc >> 8) * full_cap0) / 100;
	max17042_write_verify_reg(chip->client, MAX17042_RemCap, (u16)rem_cap);

	rep_cap = (u16)rem_cap;
	max17042_write_verify_reg(chip->client, MAX17042_RepCap, rep_cap);

	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	dq_acc = config->fullcap / dQ_ACC_DIV;
	max17042_write_verify_reg(chip->client, MAX17042_dQacc, dq_acc);
	max17042_write_verify_reg(chip->client, MAX17042_dPacc, dP_ACC_200);

	max17042_write_verify_reg(chip->client, MAX17042_FullCAP,
			config->fullcap);
	max17042_write_reg(chip->client, MAX17042_DesignCap,
			config->design_cap);
	max17042_write_verify_reg(chip->client, MAX17042_FullCAPNom,
			config->fullcapnom);
	/* Update SOC register with new SOC */
	max17042_write_reg(chip->client, MAX17042_RepSOC, vfSoc);
}

/*
 * Block write all the override values coming from platform data.
 * This function MUST be called before the POR initialization proceedure
 * specified by maxim.
 */
static inline void max17042_override_por_values(struct max17042_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct max17042_config_data *config = chip->pdata->config_data;

	max17042_override_por(client, MAX17042_TGAIN, config->tgain);
	max17042_override_por(client, MAx17042_TOFF, config->toff);
	max17042_override_por(client, MAX17042_CGAIN, config->cgain);
	max17042_override_por(client, MAX17042_COFF, config->coff);

	max17042_override_por(client, MAX17042_VALRT_Th, config->valrt_thresh);
	max17042_override_por(client, MAX17042_TALRT_Th, config->talrt_thresh);
	max17042_override_por(client, MAX17042_SALRT_Th,
			config->soc_alrt_thresh);
	max17042_override_por(client, MAX17042_CONFIG, config->config);
	max17042_override_por(client, MAX17042_SHDNTIMER, config->shdntimer);

	max17042_override_por(client, MAX17042_DesignCap, config->design_cap);
	max17042_override_por(client, MAX17042_ICHGTerm, config->ichgt_term);

	max17042_override_por(client, MAX17042_AtRate, config->at_rate);
	max17042_override_por(client, MAX17042_LearnCFG, config->learn_cfg);
	max17042_override_por(client, MAX17042_FilterCFG, config->filter_cfg);
	max17042_override_por(client, MAX17042_RelaxCFG, config->relax_cfg);
	max17042_override_por(client, MAX17042_MiscCFG, config->misc_cfg);
	max17042_override_por(client, MAX17042_MaskSOC, config->masksoc);

	max17042_override_por(client, MAX17042_FullCAP, config->fullcap);
	max17042_override_por(client, MAX17042_FullCAPNom, config->fullcapnom);
	if (chip->chip_type == MAX17042)
		max17042_override_por(client, MAX17042_SOC_empty,
						config->socempty);
	max17042_override_por(client, MAX17042_LAvg_empty, config->lavg_empty);
	max17042_override_por(client, MAX17042_dQacc, config->dqacc);
	max17042_override_por(client, MAX17042_dPacc, config->dpacc);

	if (chip->chip_type == MAX17042)
		max17042_override_por(client, MAX17042_V_empty, config->vempty);
	else
		max17042_override_por(client, MAX17047_V_empty, config->vempty);
	max17042_override_por(client, MAX17042_TempNom, config->temp_nom);
	max17042_override_por(client, MAX17042_TempLim, config->temp_lim);
	max17042_override_por(client, MAX17042_FCTC, config->fctc);
	max17042_override_por(client, MAX17042_RCOMP0, config->rcomp0);
	max17042_override_por(client, MAX17042_TempCo, config->tcompc0);
	if (chip->chip_type) {
		max17042_override_por(client, MAX17042_EmptyTempCo,
					config->empty_tempco);
		max17042_override_por(client, MAX17042_K_empty0,
					config->kempty0);
	}
}

static int max17042_init_chip(struct max17042_chip *chip)
{
	int ret;
	int val;

	max17042_override_por_values(chip);
	/* After Power up, the MAX17042 requires 500mS in order
	 * to perform signal debouncing and initial SOC reporting
	 */
	msleep(500);

	/* Initialize configaration */
	max17042_write_config_regs(chip);

	/* write cell characterization data */
	ret = max17042_init_model(chip);
	if (ret) {
		dev_err(&chip->client->dev, "%s init failed\n",
			__func__);
		return -EIO;
	}

	ret = max17042_verify_model_lock(chip);
	if (ret) {
		dev_err(&chip->client->dev, "%s lock verify failed\n",
			__func__);
		return -EIO;
	}
	/* write custom parameters */
	max17042_write_custom_regs(chip);

	/* update capacity params */
	max17042_update_capacity_regs(chip);

	/* delay must be atleast 350mS to allow VFSOC
	 * to be calculated from the new configuration
	 */
	msleep(350);

	/* reset vfsoc0 reg */
	max17042_reset_vfsoc0_reg(chip);

	/* load new capacity params */
	max17042_load_new_capacity_params(chip);

	/* Init complete, Clear the POR bit */
	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	max17042_write_reg(chip->client, MAX17042_STATUS,
			val & (~STATUS_POR_BIT));
	chip->init_complete = 1;
	return 0;
}

static void max17042_set_soc_threshold(struct max17042_chip *chip, u16 off)
{
	u16 soc, soc_tr;

	/* program interrupt thesholds such that we should
	 * get interrupt for every 'off' perc change in the soc
	 */
	soc = max17042_read_reg(chip->client, MAX17042_RepSOC) >> 8;
	soc_tr = (soc + off) << 8;
	soc_tr |= (soc - off);
	max17042_write_reg(chip->client, MAX17042_SALRT_Th, soc_tr);
}

static irqreturn_t max17042_thread_handler(int id, void *dev)
{
	struct max17042_chip *chip = dev;
	u16 val;

	val = max17042_read_reg(chip->client, MAX17042_STATUS);
	if ((val & STATUS_INTR_SOCMIN_BIT) ||
		(val & STATUS_INTR_SOCMAX_BIT)) {
		dev_info(&chip->client->dev, "SOC threshold INTR\n");
		max17042_set_soc_threshold(chip, 1);
	}

	power_supply_changed(&chip->battery);
	return IRQ_HANDLED;
}

#ifdef CONFIG_EDP_FRAMEWORK
struct temp_ibat_map {
	unsigned int temp;
	unsigned int ibat;
};

struct temp_ibat_map safe_ibat_lut[] = {
	{ 25, 3700 },
};

static unsigned int max17042_safe_ibat(int temp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(safe_ibat_lut) - 1; i++)
		if (temp <= safe_ibat_lut[i].temp)
			break;

	return safe_ibat_lut[i].ibat;
}

static int max17042_get_bat_vars(struct max17042_chip *chip, s64 *avgcurrent,
		s64 *avgvcell, s64 *vfocv, s64 *temp)
{
	struct power_supply *psy;
	union power_supply_propval pv;

	psy = &chip->battery;

	if (max17042_get_property(psy, POWER_SUPPLY_PROP_CURRENT_AVG, &pv))
		return -EFAULT;
	*avgcurrent = -pv.intval;

	if (max17042_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_AVG, &pv))
		return -EFAULT;
	*avgvcell = pv.intval;

	if (max17042_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_OCV, &pv))
		return -EFAULT;
	*vfocv = pv.intval;

	if (max17042_get_property(psy, POWER_SUPPLY_PROP_TEMP, &pv))
		return -EFAULT;
	*temp = pv.intval;

	return 0;
}

static inline unsigned int max17042_max_depletion(struct max17042_chip *chip)
{
	return chip->pdata->edp_client->states[0];
}

static int rbat_hist_sum = RBAT_HIST_COUNT * RBAT_INIT;
static int rbat_hist[RBAT_HIST_COUNT];
static int rbat_hist_i;

static void max17042_rbat_hist_init(void)
{
	int i;
	for (i = 0; i < RBAT_HIST_COUNT; i++)
		rbat_hist[i] = RBAT_INIT;
}

static int max17042_rbat(struct max17042_chip *chip, s64 avgcurrent,
		s64 avgvcell, s64 vfocv)
{
	int rbat;

	if (avgcurrent < AVG_CURRENT_MIN) {
		rbat = chip->rbat_lastgood;
	} else {
		rbat = div64_s64(1000000 * (vfocv - avgvcell), avgcurrent);
		rbat = max(rbat, RBAT_INIT);
		chip->rbat_lastgood = rbat;
	}

	rbat_hist_sum -= rbat_hist[rbat_hist_i];
	rbat_hist[rbat_hist_i] = rbat;
	rbat_hist_sum += rbat;
	rbat = rbat_hist_sum / RBAT_HIST_COUNT;
	rbat_hist_i = (rbat_hist_i + 1) % RBAT_HIST_COUNT;

	return rbat;
}

static s64 max17042_ibat_possible(struct max17042_chip *chip, s64 avgcurrent,
		s64 vfocv, s64 rbat)
{
	s64 ibat;

	rbat += R_CONTACTS + R_BOARD + R_PASSFET;
	ibat = div64_s64(1000 * (vfocv - VSYS_MIN), rbat);
	if (avgcurrent < 0)
		ibat += chip->chgin_ilim;

	return ibat;
}

static unsigned int max17042_depletion(struct max17042_chip *chip)
{
	s64 avgcurrent;
	s64 avgvcell;
	s64 vfocv;
	s64 temp;

	s64 rbat;
	s64 ibat_possible;
	s64 ibat_tbat;
	s64 ibat_nominal;
	s64 pbat_nominal;
	s64 pbat_adjusted;
	s64 depl_temp;
	s64 depl_vdroop;
	s64 depl;

	if (max17042_get_bat_vars(chip, &avgcurrent, &avgvcell, &vfocv,
				&temp)) {
		WARN_ON(1);
		return max17042_max_depletion(chip);
	}

	rbat = max17042_rbat(chip, avgcurrent, avgvcell, vfocv);
	ibat_possible = max17042_ibat_possible(chip, avgcurrent, vfocv, rbat);

	ibat_tbat = max17042_safe_ibat(temp);
	ibat_nominal = max17042_safe_ibat(NOMINAL_TEMP);
	pbat_nominal = div64_s64(ibat_nominal * NOMINAL_VOLTAGE, 1000000);
	pbat_adjusted = div64_s64(pbat_nominal * ibat_tbat * vfocv,
			ibat_nominal * NOMINAL_VOLTAGE);

	depl_temp = pbat_nominal - pbat_adjusted;

	depl_vdroop = pbat_adjusted - div64_s64(vfocv * ibat_possible, 1000000);
	depl_vdroop = max_t(s64, 0, depl_vdroop);

	depl = depl_temp + depl_vdroop;
	depl = div64_s64(depl * NOMINAL_VOLTAGE * chip->edp_manager->max,
			vfocv * pbat_nominal);

	if (IS_ENABLED(CONFIG_DEBUG_KERNEL)) {
		printk(KERN_DEBUG "max17042\n");
		printk(KERN_DEBUG "    AVERAGE_ICELL: %lld uA\n", avgcurrent);
		printk(KERN_DEBUG "    AVERAGE_VCELL: %lld uV\n", avgvcell);
		printk(KERN_DEBUG "    VFOCV        : %lld uV\n", vfocv);
		printk(KERN_DEBUG "    TEMPERATURE  : %lld C\n", temp);
		printk(KERN_DEBUG "    RBAT         : %lld\n", rbat);
		printk(KERN_DEBUG "    CHGIN_ILIM   : %u\n", chip->chgin_ilim);
		printk(KERN_DEBUG "    IBAT_possible: %lld\n", ibat_possible);
		printk(KERN_DEBUG "    IBAT_tbat    : %lld\n", ibat_tbat);
		printk(KERN_DEBUG "    IBAT_nominal : %lld\n", ibat_nominal);
		printk(KERN_DEBUG "    PBAT_nominal : %lld\n", pbat_nominal);
		printk(KERN_DEBUG "    PBAT_adjusted: %lld\n", pbat_adjusted);
		printk(KERN_DEBUG "    DEPL_temp    : %lld\n", depl_temp);
		printk(KERN_DEBUG "    DEPL_vdroop  : %lld\n", depl_vdroop);
		printk(KERN_DEBUG "    depletion    : %lld\n", depl);
	}

	depl = clamp_t(s64, depl, 0, max17042_max_depletion(chip));
	return depl;
}

static void max17042_update_depletion(struct work_struct *work)
{
	struct max17042_chip *chip;
	struct edp_client *c;
	unsigned int depl;
	unsigned int i;
	int r;

	chip = container_of(work, struct max17042_chip, depl_work.work);
	c = chip->pdata->edp_client;
	depl = max17042_depletion(chip);
	i = c->num_states - 1;

	while (i && c->states[i] < depl)
		i--;

	if (chip->edp_req != i) {
		r = edp_update_client_request(c, i, NULL);
		WARN_ON(r);
		chip->edp_req = i;
	}

	schedule_delayed_work(to_delayed_work(work),
			msecs_to_jiffies(DEPL_INTERVAL));
}

/* Nothing to do */
static void max17042_throttle(unsigned int new_state, void *priv_data)
{
}

static int max17042_init_depletion(struct max17042_chip *chip)
{
	struct edp_client *c;
	int r;

	chip->edp_manager = edp_get_manager("battery");
	if (!chip->edp_manager) {
		dev_err(&chip->client->dev,
				"could not get the battery EDP manager\n");
		return -ENODEV;
	}

	c = chip->pdata->edp_client;
	chip->rbat_lastgood = RBAT_INIT;
	chip->edp_req = c->num_states;
	chip->chgin_ilim = 0;

	strncpy(c->name, chip->battery.name, EDP_NAME_LEN - 1);
	c->name[EDP_NAME_LEN - 1] = 0;
	c->throttle = max17042_throttle;

	r = edp_register_client(chip->edp_manager, c);
	if (r) {
		dev_err(&chip->client->dev,
				"failed to register depletion client (%d)\n",
				r);
		return r;
	}

	max17042_rbat_hist_init();
	INIT_DEFERRABLE_WORK(&chip->depl_work, max17042_update_depletion);
	schedule_delayed_work(&chip->depl_work,
			msecs_to_jiffies(DEPL_INTERVAL));
	return 0;
}

static void max17042_suspend_depletion_mon(struct max17042_chip *chip)
{
	cancel_delayed_work_sync(&chip->depl_work);
}

static void max17042_resume_depletion_mon(struct max17042_chip *chip)
{
	schedule_delayed_work(&chip->depl_work, 0);
}

#else

static inline int max17042_init_depletion(struct max17042_chip *chip)
{
	return 0;
}

static inline void max17042_suspend_depletion_mon(struct max17042_chip *chip)
{
}

static inline void max17042_resume_depletion_mon(struct max17042_chip *chip)
{
}
#endif

static void max17042_init_worker(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(work,
				struct max17042_chip, work.work);
	power_supply_changed(&chip->battery);
	schedule_delayed_work(&chip->work, MAX17047_DELAY);
}

#ifdef CONFIG_OF
static struct max17042_platform_data *
max17042_get_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 prop;
	struct max17042_platform_data *pdata;

	if (!np)
		return dev->platform_data;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/*
	 * Require current sense resistor value to be specified for
	 * current-sense functionality to be enabled at all.
	 */
	if (of_property_read_u32(np, "maxim,rsns-microohm", &prop) == 0) {
		pdata->r_sns = prop;
		pdata->enable_current_sense = true;
	}

	return pdata;
}
#else
static struct max17042_platform_data *
max17042_get_pdata(struct device *dev)
{
	return dev->platform_data;
}
#endif

static int max17042_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17042_chip *chip;
	int ret;
	int reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	temp_client = client;
	chip->pdata = max17042_get_pdata(&client->dev);
	if (!chip->pdata) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}
	tmp_chip = chip;
	i2c_set_clientdata(client, chip);

	ret = max17042_read_reg(chip->client, MAX17042_DevName);
	if (ret == MAX17042_IC_VERSION) {
		dev_dbg(&client->dev, "chip type max17042 detected\n");
		chip->chip_type = MAX17042;
	} else if (ret == MAX17047_IC_VERSION) {
		dev_dbg(&client->dev, "chip type max17047/50 detected\n");
		chip->chip_type = MAX17047;
	} else {
		dev_err(&client->dev, "device version mismatch: %x\n", ret);
		return -EIO;
	}

	chip->battery.name		= "max170xx_battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17042_get_property;
	chip->battery.properties	= max17042_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17042_battery_props);

	/* When current is not measured,
	 * CURRENT_NOW and CURRENT_AVG properties should be invisible. */
	if (!chip->pdata->enable_current_sense)
		chip->battery.num_properties -= 2;

	if (chip->pdata->r_sns == 0)
		chip->pdata->r_sns = MAX17042_DEFAULT_SNS_RESISTOR;

	if (chip->pdata->init_data)
		max17042_set_reg(client, chip->pdata->init_data,
				chip->pdata->num_init_data);

	if (!chip->pdata->enable_current_sense) {
		max17042_write_reg(client, MAX17042_CGAIN, 0x0000);
		max17042_write_reg(client, MAX17042_MiscCFG, 0x0003);
		max17042_write_reg(client, MAX17042_LearnCFG, 0x0007);
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
						max17042_thread_handler,
						IRQF_TRIGGER_FALLING,
						chip->battery.name, chip);
		if (!ret) {
			reg =  max17042_read_reg(client, MAX17042_CONFIG);
			reg |= CONFIG_ALRT_BIT_ENBL;
			max17042_write_reg(client, MAX17042_CONFIG, reg);
			max17042_set_soc_threshold(chip, 1);
		} else {
			client->irq = 0;
			dev_err(&client->dev, "%s(): cannot get IRQ\n",
				__func__);
		}
	}

	reg = max17042_read_reg(chip->client, MAX17042_STATUS);
	if (reg & STATUS_POR_BIT) {
		if (chip->pdata->enable_por_init && chip->pdata->config_data) {
			ret = max17042_init_chip(chip);
			if (ret)
				return ret;
		}
	} else {
		chip->init_complete = 1;
	}

	/* Check for battery presence */
	ret = maxim_get_temp();
	if (ret == 0xff) {
		dev_err(&client->dev, "failed in reading temperaure\n");
		return -ENODEV;
	} else if ((ret < MIN_TEMP) || (ret > MAX_TEMP)) {
		dev_err(&client->dev, "Battery not detected exiting driver\n");
		return -ENODEV;
	}

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		return ret;
	}

	ret = max17042_init_depletion(chip);
	if (ret) {
		dev_err(&client->dev, "failed to init depletion EDP client\n");
		return ret;
	}

	INIT_DEFERRABLE_WORK(&chip->work, max17042_init_worker);
	schedule_delayed_work(&chip->work, 0);

	return 0;
}

static int max17042_remove(struct i2c_client *client)
{
	struct max17042_chip *chip = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, chip);
	power_supply_unregister(&chip->battery);
	return 0;
}

static void max17042_shutdown(struct i2c_client *client)
{
	struct max17042_chip *chip = i2c_get_clientdata(client);

	if (client->irq)
		disable_irq(client->irq);

	cancel_delayed_work_sync(&chip->work);

	chip->shutdown_complete = 1;
}

#ifdef CONFIG_PM
static int max17042_suspend(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	max17042_suspend_depletion_mon(chip);

	/*
	 * disable the irq and enable irq_wake
	 * capability to the interrupt line.
	 */
	if (chip->client->irq) {
		disable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
	}

	return 0;
}

static int max17042_resume(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	max17042_resume_depletion_mon(chip);

	if (chip->client->irq) {
		disable_irq_wake(chip->client->irq);
		enable_irq(chip->client->irq);
		/* re-program the SOC thresholds to 1% change */
		max17042_set_soc_threshold(chip, 1);
	}

	return 0;
}

static const struct dev_pm_ops max17042_pm_ops = {
	.suspend	= max17042_suspend,
	.resume		= max17042_resume,
};

#define MAX17042_PM_OPS (&max17042_pm_ops)
#else
#define MAX17042_PM_OPS NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id max17042_dt_match[] = {
	{ .compatible = "maxim,max17042" },
	{ .compatible = "maxim,max17047" },
	{ .compatible = "maxim,max17050" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17042_dt_match);
#endif

static const struct i2c_device_id max17042_id[] = {
	{ "max17042", 0 },
	{ "max17047", 1 },
	{ "max17050", 2 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17042_id);

static struct i2c_driver max17042_i2c_driver = {
	.driver	= {
		.name	= "max17042",
		.of_match_table = of_match_ptr(max17042_dt_match),
		.pm	= MAX17042_PM_OPS,
	},
	.probe		= max17042_probe,
	.remove		= max17042_remove,
	.id_table	= max17042_id,
	.shutdown	= max17042_shutdown,
};
module_i2c_driver(max17042_i2c_driver);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("MAX17042 Fuel Gauge");
MODULE_LICENSE("GPL");
