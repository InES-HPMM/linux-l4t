
/*
 *  MAXIM MAX77660 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/max77660/max77660-charger.h>
#include <linux/mfd/max77660/max77660-core.h>

/* define register map */
#define MAX77660_REG_CHGINT	 0x5D

#define MAX77660_REG_CHGINTM	0x5E

#define MAX77660_REG_CHGSTAT	0x5F

#define MAX77660_DCV_MASK	   0x80
#define MAX77660_DCV_SHIFT	  7
#define MAX77660_DCI_MASK	   0x40
#define MAX77660_DCI_SHIFT	  6
#define MAX77660_DCOVP_MASK	 0x20
#define MAX77660_DCOVP_SHIFT	5
#define MAX77660_DCUVP_MASK	 0x10
#define MAX77660_DCUVP_SHIFT	4
#define MAX77660_CHG_MASK	   0x08
#define MAX77660_CHG_SHIFT	  3
#define MAX77660_BAT_MASK	   0x04
#define MAX77660_BAT_SHIFT	  2

#define MAX77660_REG_DETAILS1   0x60
#define MAX77660_DC_V_MASK	  0x80
#define MAX77660_DC_V_SHIFT	 7
#define MAX77660_DC_I_MASK	  0x40
#define MAX77660_DC_I_SHIFT	 6
#define MAX77660_DC_OVP_MASK	0x20
#define MAX77660_DC_OVP_SHIFT   5
#define MAX77660_DC_UVP_MASK	0x10
#define MAX77660_DC_UVP_SHIFT   4

#define MAX77660_REG_DETAILS2   0x61
#define MAX77660_BAT_DTLS_MASK  0x30
#define MAX77660_BAT_DTLS_SHIFT 4
#define MAX77660_CHG_DTLS_MASK  0x0F
#define MAX77660_CHG_DTLS_SHIFT 0

#define MAX77660_BAT_DTLS_BATDEAD	   0   /*  VBAT<2.1V  */
#define MAX77660_BAT_DTLS_TIMER_FAULT   1   /* The battery is taking longer than expected to charge */
#define MAX77660_BAT_DTLS_BATOK         2   /* VBAT is okay. */
#define MAX77660_BAT_DTLS_GTBATOVF      3   /* VBAT > BATOV  */

#define MAX77660_CHG_DTLS_DEAD_BAT              0   /* VBAT<2.1V, TJSHDN<TJ<TJREG  */
#define MAX77660_CHG_DTLS_PREQUAL               1   /* VBAT<3.0V, TJSHDN<TJ<TJREG  */
#define MAX77660_CHG_DTLS_FAST_CHARGE_CC        2   /* VBAT>3.0V, TJSHDN<TJ<TJREG  */
#define MAX77660_CHG_DTLS_FAST_CHARGE_CV        3   /* VBAT=VBATREG, TJSHDN<TJ<TJREG  */
#define MAX77660_CHG_DTLS_TOP_OFF               4   /* VBAT>=VBATREG, TJSHDN<TJ<TJREG  */
#define MAX77660_CHG_DTLS_DONE                  5   /* VBAT>VBATREG, T>Ttopoff+16s, TJSHDN<TJ<TJREG */
#define MAX77660_CHG_DTLS_DONE_QBAT_ON          6   /* VBAT<VBATOV, TJ<TJSHDN */
#define MAX77660_CHG_DTLS_TIMER_FAULT           7   /* TEMP<T1 or TEMP>T4 */
#define MAX77660_CHG_DTLS_DC_INVALID            8   /* charger is off, DC is invalid or chaarger is disabled(USBSUSPEND) */
#define MAX77660_CHG_DTLS_THERMAL_LOOP_ACTIVE   9   /* TJ > REGTEMP */
#define MAX77660_CHG_DTLS_CHG_OFF               10  /* charger is off and TJ >TSHDN */

#define MAX77660_REG_CHGCNTL1       0x64
#define MAX77660_VICHG_GAIN_MASK    0x40
#define MAX77660_VICHG_GAIN_SHIFT   2
#define MAX77660_DCMON_DIS_MASK     0x02
#define MAX77660_DCMON_DIS_SHIFT    1
#define MAX77660_USB_SUS_MASK       0x01
#define MAX77660_USB_SUS_SHIFT      0

#define MAX77660_REG_FCHGCRNT       0x8A
#define MAX77660_CHGCC_MASK         0x1F
#define MAX77660_CHGCC_SHIFT        0
#define MAX77660_FCHGTIME_MASK      0xE0
#define MAX77660_FCHGTIME_SHIFT     5


#define MAX77660_REG_DCCRNT         0x68
#define MAX77660_CHGRSTRT_MASK      0x40
#define MAX77660_CHGRSTRT_SHIFT     6
#define MAX77660_DCILMT_MASK        0x3F
#define MAX77660_DCILMT_SHIFT       0

#define MAX77660_REG_TOPOFF         0x66
#define MAX77660_TOPOFFTIME_MASK    0xE0
#define MAX77660_TOPOFFTIME_SHIFT   5
#define MAX77660_IFST2P8_MASK       0x10
#define MAX77660_IFST2P8_SHIFT      4
#define MAX77660_TOPOFFTSHLD_MASK   0x0C
#define MAX77660_TOPOFFTSHLD_SHIFT  2
#define MAX77660_CHGCV_MASK         0x03
#define MAX77660_CHGCV_SHIFT        0

#define MAX77660_REG_TEMPREG        0x8D
#define MAX77660_REGTEMP_MASK       0xC0
#define MAX77660_REGTEMP_SHIFT      6

#define MAX77660_REG_PROTCMD        0x8E
#define MAX77660_CHGPROT_MASK       0x0C
#define MAX77660_CHGPROT_SHIFT      2

enum charger_type {
	CHARGER_USB,
	CHARGER_DC,
};

struct max77660_charger {
	struct max77660_charger_platform_data *pdata;
	struct device *dev;
	struct power_supply usb_psy;
	struct power_supply dc_psy;
#ifndef CONFIG_MAX77660_BATTERY
	struct power_supply batt_psy;
#endif
	int irq;
	int usb_online;
	int dc_online;
};

static struct max77660_charger *max77660_chg;
static int usb_chg_current;
static void (*notify_vbus_state_func_ptr)(int);
static DEFINE_SPINLOCK(vbus_lock);

int max77660_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!max77660_chg) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB)
		return -EINVAL;

	max77660_chg->usb_psy.type = type;
	power_supply_changed(&max77660_chg->usb_psy);
	power_supply_changed(&max77660_chg->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(max77660_set_usb_power_supply_type);

int max77660_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(max77660_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void max77660_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(max77660_charger_unregister_vbus_sn);

/* USB calls these to tell us how much max usb current the system can draw */
void max77660_charger_vbus_draw(unsigned int mA)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);
	spin_lock_irqsave(&vbus_lock, flags);
	if (max77660_chg) {
		max77660_start_charging(mA);
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		usb_chg_current = mA;
	}
	spin_unlock_irqrestore(&vbus_lock, flags);
}
EXPORT_SYMBOL_GPL(max77660_charger_vbus_draw);

int max77660_disable_source_current(bool disable)
{
	if (!max77660_chg) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");
	return max77660_stop_charging();
}
EXPORT_SYMBOL(max77660_disable_source_current);

int max77660_charger_vbus_in(void)
{
	if (!max77660_chg) {
		pr_err("called before init\n");
		return 0;
	}
	pr_debug("max77660_chg->usb_online = %d\n", max77660_chg->usb_online);
	return max77660_chg->usb_online;
}
EXPORT_SYMBOL(max77660_charger_vbus_in);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

static int __set_charger(struct max77660_charger *chip, int enable)
{
	u8 reg_val = 0;

	/* unlock charger protection  */
	reg_val = MAX77660_CHGPROT_UNLOCKED<<MAX77660_CHGPROT_SHIFT;
	max77660_write(chip->dev, MAX77660_REG_PROTCMD, &reg_val, 1, MAX77660_I2C_CHG);

	if (enable) {
		/* enable charger */

		if (chip->usb_online) {
			/* Set fast charge current and timer */
			reg_val = ((chip->pdata[CHARGER_USB].chgcc<<MAX77660_CHGCC_SHIFT) |
					   (chip->pdata[CHARGER_USB].fchgtime<<MAX77660_FCHGTIME_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_FCHGCRNT, &reg_val, 1, MAX77660_I2C_CHG);

			/* Set input current limit and charger restart threshold */
			reg_val = ((chip->pdata[CHARGER_USB].chgrstrt<<MAX77660_CHGRSTRT_SHIFT) |
					   (chip->pdata[CHARGER_USB].dcilmt<<MAX77660_DCILMT_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_DCCRNT, &reg_val, 1, MAX77660_I2C_CHG);

			/* Set topoff condition  */
			reg_val = ((chip->pdata[CHARGER_USB].topofftime<<MAX77660_TOPOFFTIME_SHIFT) |
					   (chip->pdata[CHARGER_USB].topofftshld<<MAX77660_TOPOFFTSHLD_SHIFT) |
					   (chip->pdata[CHARGER_USB].chgcv<<MAX77660_CHGCV_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_TOPOFF, &reg_val, 1, MAX77660_I2C_CHG);

			/* USB Suspend and DC Voltage Monitoring
						Set DC Voltage Monitoring to disable and USB Suspend to Disable  */
			reg_val = (1<<MAX77660_DCMON_DIS_SHIFT) | (0<<MAX77660_USB_SUS_SHIFT);
			max77660_write(chip->dev, MAX77660_REG_CHGCNTL1, &reg_val, 1, MAX77660_I2C_CHG);
		}

		if (chip->dc_online) {
			/* Set fast charge current and timer  */
			reg_val = ((chip->pdata[CHARGER_DC].chgcc<<MAX77660_CHGCC_SHIFT) |
					   (chip->pdata[CHARGER_DC].fchgtime<<MAX77660_FCHGTIME_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_FCHGCRNT, &reg_val, 1, MAX77660_I2C_CHG);

			/* Set input current limit and charger restart threshold */
			reg_val = ((chip->pdata[CHARGER_DC].chgrstrt<<MAX77660_CHGRSTRT_SHIFT) |
					   (chip->pdata[CHARGER_DC].dcilmt<<MAX77660_DCILMT_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_DCCRNT, &reg_val, 1, MAX77660_I2C_CHG);

			/* Set topoff condition */
			reg_val = ((chip->pdata[CHARGER_DC].topofftime<<MAX77660_TOPOFFTIME_SHIFT) |
					   (chip->pdata[CHARGER_DC].topofftshld<<MAX77660_TOPOFFTSHLD_SHIFT) |
					   (chip->pdata[CHARGER_DC].chgcv<<MAX77660_CHGCV_SHIFT));
			max77660_write(chip->dev, MAX77660_REG_TOPOFF, &reg_val, 1, MAX77660_I2C_CHG);

			/* USB Suspend and DC Voltage Monitoring */
			/* Set DC Voltage Monitoring to disable and USB Suspend to Disable */
			reg_val = (1<<MAX77660_DCMON_DIS_SHIFT) | (0<<MAX77660_USB_SUS_SHIFT);
			max77660_write(chip->dev, MAX77660_REG_CHGCNTL1, &reg_val, 1, MAX77660_I2C_CHG);
		}
	} else {
		/* disable charge */
		max77660_set_bits(chip->dev, MAX77660_REG_CHGCNTL1, MAX77660_USB_SUS_MASK, 1, MAX77660_I2C_CHG);
	}
	dev_info(chip->dev, "%s\n", (enable) ? "Enable charger" : "Disable charger");
	return 0;
}

static int max77660_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max77660_charger *chip = (struct max77660_charger *)data;
	/* struct max77660_adc_result result; */
	u8 reg_val = 0;

	switch (irq) {
	case MAX77660_CHG_BAT_I:
	  /* dev_info(chip->dev, "Battery Interrupt: details-0x%x\n", (val[2] & MAX77660_BAT_DTLS_MASK)); */
		switch ((val[2] & MAX77660_BAT_MASK)>>MAX77660_BAT_SHIFT) {
		case MAX77660_BAT_DTLS_BATDEAD:
			dev_info(chip->dev, "Battery Interrupt: BATDEAD\n");
			break;
		case MAX77660_BAT_DTLS_TIMER_FAULT:
			dev_info(chip->dev, "Battery Interrupt: TIMER_FAULT\n");
			break;
		case MAX77660_BAT_DTLS_BATOK:
			dev_info(chip->dev, "Battery Interrupt: BATOK\n");
			break;
		case MAX77660_BAT_DTLS_GTBATOVF:
			dev_info(chip->dev, "Battery Interrupt: GTBATOVF\n");
			break;
		default:
			dev_info(chip->dev, "Battery Interrupt: details-0x%x\n", (val[2] & MAX77660_BAT_DTLS_MASK));
			break;
		}
		break;

	case MAX77660_CHG_CHG_I:
	  /*dev_info(chip->dev, "Fast Charge current Interrupt: details-0x%x\n", (val[2] & MAX77660_CHG_DTLS_MASK)); */
		switch (val[2] & MAX77660_CHG_DTLS_MASK) {
		case MAX77660_CHG_DTLS_DEAD_BAT:
			dev_info(chip->dev, "Fast Charge current Interrupt: DEAD_BAT\n");

			break;
		case MAX77660_CHG_DTLS_PREQUAL:
			dev_info(chip->dev, "Fast Charge current Interrupt: PREQUAL\n");

			break;
		case MAX77660_CHG_DTLS_FAST_CHARGE_CC:
			dev_info(chip->dev, "Fast Charge current Interrupt: FAST_CHARGE_CC\n");

			break;
		case MAX77660_CHG_DTLS_FAST_CHARGE_CV:
			dev_info(chip->dev, "Fast Charge current Interrupt: FAST_CHARGE_CV\n");

			break;
		case MAX77660_CHG_DTLS_TOP_OFF:
			dev_info(chip->dev, "Fast Charge current Interrupt: TOP_OFF\n");

			break;
		case MAX77660_CHG_DTLS_DONE:
			dev_info(chip->dev, "Fast Charge current Interrupt: DONE\n");

			break;
		case MAX77660_CHG_DTLS_DONE_QBAT_ON:
			dev_info(chip->dev, "Fast Charge current Interrupt: DONE_QBAT_ON\n");

			break;
		case MAX77660_CHG_DTLS_TIMER_FAULT:
			dev_info(chip->dev, "Fast Charge current Interrupt: TIMER_FAULT\n");

			break;
		case MAX77660_CHG_DTLS_DC_INVALID:
			dev_info(chip->dev, "Fast Charge current Interrupt: DC_INVALID\n");

			break;
		case MAX77660_CHG_DTLS_THERMAL_LOOP_ACTIVE:
			dev_info(chip->dev, "Fast Charge current Interrupt: THERMAL_LOOP_ACTIVE\n");

			break;
		case MAX77660_CHG_DTLS_CHG_OFF:
			dev_info(chip->dev, "Fast Charge current Interrupt: CHG_OFF\n");

			break;
		default:
			dev_info(chip->dev, "Fast Charge current Interrupt: details-0x%x\n", (val[2] & MAX77660_CHG_DTLS_MASK));
			break;
		}
		break;

	case MAX77660_CHG_DC_UVP:
		if ((val[1] & MAX77660_DC_UVP_MASK) == MAX77660_DC_UVP_MASK) {
			dev_info(chip->dev, "DC Under voltage Interrupt: VDC > VDC_UVLO\n");

			/* Read USB ID ADC */
			#if 0
			max77660_adc_read(CHANNEL11_AMUX2, &result);
			result.physical = 1000; /* TEST_ONLY */
			if (result.physical < 1000)	/* change later */
				max77660_chg->dc_online  = 1;
			else
			#endif
				max77660_chg->usb_online = 1;

			/* Set IRQ MASK register */
			chip->pdata->int_mask = (MAX77660_DCOVP_MASK | MAX77660_DCUVP_MASK |
											MAX77660_CHG_MASK   | MAX77660_BAT_MASK);
			reg_val = chip->pdata->int_mask;
			max77660_write(chip->dev, MAX77660_REG_CHGINTM, &reg_val, 1, MAX77660_I2C_CHG);

			notify_usb_of_the_plugin_event(max77660_chg->usb_online);
			__set_charger(chip, 1);
		} else if ((val[1] & MAX77660_DC_UVP_MASK) == 0) {
			dev_info(chip->dev, "DC Under voltage Interrupt: VDC < VDC_UVLO\n");
			/* VBUS is invalid. VDC < VDC_UVLO */
			max77660_chg->dc_online  = 0;
			max77660_chg->usb_online = 0;

			__set_charger(chip, 0);
			notify_usb_of_the_plugin_event(max77660_chg->usb_online);
		}
		/* dev_info(chip->dev, "DC Under voltage Interrupt: details-0x%x\n", (val[1] & MAX77660_DC_UVP_MASK)); */
		break;

	case MAX77660_CHG_DC_OVP:
		if ((val[1] & MAX77660_DC_OVP_MASK) == MAX77660_DC_OVP_MASK) {
			dev_info(chip->dev, "DC Over voltage Interrupt: VDC > VDC_OVLO\n");

			/*  VBUS is invalid. VDC > VDC_OVLO  */
			__set_charger(chip, 0);
		} else if ((val[1] & MAX77660_DC_OVP_MASK) == 0) {
			dev_info(chip->dev, "DC Over voltage Interrupt: VDC < VDC_OVLO\n");

			/*  VBUS is valid. VDC < VDC_OVLO  */
			__set_charger(chip, 1);
		}
		/* dev_info(chip->dev, "DC Over voltage Interrupt: details-0x%x\n", (val[1] & MAX77660_DC_OVP_MASK));  */
		break;

	case MAX77660_CHG_DC_I:
		dev_info(chip->dev, "DC Input Current Limit Interrupt: details-0x%x\n", (val[1] & MAX77660_DC_I_MASK));
		break;

	case MAX77660_CHG_DC_V:
		dev_info(chip->dev, "DC Input Voltage Limit Interrupt: details-0x%x\n", (val[1] & MAX77660_DC_V_MASK));
		break;

	}
	return 0;
}

static irqreturn_t max77660_charger_handler(int irq, void *data)
{
	struct max77660_charger *chip = (struct max77660_charger *)data;
	int irq_val, irq_mask, irq_name;
	int ret;
	u8 val[3];

	ret = max77660_read(chip->dev, MAX77660_REG_CHGINT, &irq_val, 1, MAX77660_I2C_CHG);
	ret = max77660_read(chip->dev, MAX77660_REG_CHGINTM, &irq_mask, 1, MAX77660_I2C_CHG);

	ret = max77660_read(chip->dev, MAX77660_REG_CHGSTAT, &val[0], 1, MAX77660_I2C_CHG);
	ret = max77660_read(chip->dev, MAX77660_REG_DETAILS1, &val[1], 1, MAX77660_I2C_CHG);
	ret = max77660_read(chip->dev, MAX77660_REG_DETAILS2, &val[2], 1, MAX77660_I2C_CHG);

	printk(KERN_ERR "max77660_charger_handler chgstat=0x%x, detail1=0x%x, detail2=0x%x\n",
				val[0], val[1], val[2]);

	for (irq_name = MAX77660_CHG_BAT_I; irq_name < MAX77660_CHG_NR_IRQS; irq_name++) {
		if ((irq_val & (0x01<<(irq_name+2))) && !(irq_mask & (0x01<<(irq_name+2))))
			max77660_charger_detail_irq(irq_name, data, val);
	}

	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#ifndef CONFIG_MAX77660_BATTERY
	power_supply_changed(&chip->batt_psy);
#endif
	return IRQ_HANDLED;
}

int max77660_stop_charging(void)
{
	/* Charger removed  */
	max77660_chg->usb_online = 0;
	max77660_chg->dc_online = 0;
	__set_charger(max77660_chg, 0);
	/* Disable GSM TEST MODE  */
	max77660_set_bits(max77660_chg->dev, MAX77660_REG_TOPOFF, MAX77660_IFST2P8_MASK, 0<<MAX77660_IFST2P8_SHIFT, MAX77660_I2C_CHG);
	return 0;
}
EXPORT_SYMBOL(max77660_stop_charging);

int max77660_start_charging(unsigned mA)
{
/*  struct max77660_adc_result result;  */

	if (mA == 2800) {
		/* GSM TEST MODE  */
		max77660_set_bits(max77660_chg->dev, MAX77660_REG_TOPOFF, MAX77660_IFST2P8_MASK, 1<<MAX77660_IFST2P8_SHIFT, MAX77660_I2C_CHG);
	} else {
	   /* charger inserted,  Read USB ID ADC */
	   #if 0
	   max77660_adc_read(CHANNEL11_AMUX2, &result);
	   if (result.physical < 1000)	/* Threshold to be changed later */
		   max77660_chg->dc_online = 1;
	   else
			max77660_chg->usb_online = 1;
	   #endif

	   max77660_chg->pdata->chgcc = FCHG_CURRENT(mA);
	   __set_charger(max77660_chg, 1);
	}
	return 0;
}
EXPORT_SYMBOL(max77660_start_charging);

static enum power_supply_property charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

#ifndef CONFIG_MAX77660_BATTERY
static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};
#endif

static char *pm_power_supplied_to[] = {
	"battery",
};

static int charger_get_property(struct power_supply *psy,
									enum power_supply_property psp,
									union power_supply_propval *val)
{
	struct max77660_charger *chip;
	int ret = 0;
	int chg_dtls_val;
	int chg_online;
	u8 reg_val;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	{
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			chip = container_of(psy, struct max77660_charger, dc_psy);
		} else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			chip = container_of(psy, struct max77660_charger, usb_psy);
		} else {
			chip = NULL;
			pr_err("error - charger didn't connect\n");
			return -ENODEV;
		}

		ret = max77660_read(chip->dev, MAX77660_REG_CHGSTAT, &reg_val, 1, MAX77660_I2C_CHG);
		val->intval =  (reg_val & MAX77660_DCUVP_MASK) ? 1 : 0;
	}
	break;

	case POWER_SUPPLY_PROP_STATUS:
	{
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			chip = container_of(psy, struct max77660_charger, dc_psy);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			chip = container_of(psy, struct max77660_charger, usb_psy);
		else {
			chip = NULL;
			pr_err("error - charger didn't connect\n");
			return -ENODEV;
		}

		ret = max77660_read(chip->dev, MAX77660_REG_DETAILS2, &reg_val, 1, MAX77660_I2C_CHG);
		chg_dtls_val = (reg_val & MAX77660_CHG_DTLS_MASK);

		ret = max77660_read(chip->dev, MAX77660_REG_CHGSTAT, &reg_val, 1, MAX77660_I2C_CHG);
		chg_online =  (reg_val & MAX77660_DCUVP_MASK) ? 1 : 0;

		if (chg_online) {
			if (chg_dtls_val == MAX77660_CHG_DTLS_DONE)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else if ((chg_dtls_val == MAX77660_CHG_DTLS_TIMER_FAULT) ||
					 (chg_dtls_val == MAX77660_CHG_DTLS_DC_INVALID) ||
					 (chg_dtls_val == MAX77660_CHG_DTLS_CHG_OFF))
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			chip = container_of(psy, struct max77660_charger, dc_psy);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			chip = container_of(psy, struct max77660_charger, usb_psy);
		else {
			chip = NULL;
			pr_err("error - charger didn't connect\n");
			return -ENODEV;
		}

		/* convert register value to mA. */
		val->intval = chip->pdata->chgcc * 50;
		break;

	default:
		ret = -ENODEV;
		break;
	}
	return 0;
}

static int charger_set_property(struct power_supply *psy,
									enum power_supply_property psp,
									const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 0;

	default:
		break;
	}
	return -ENODEV;
}

#ifndef CONFIG_MAX77660_BATTERY
static int get_prop_battery_uvolts(void)
{
/* To DO: Call fuel gauge driver to get uvolts */
	return 0;
}

static int get_prop_battery_capacity(void)
{
/* To DO: Call fuel gauge driver to get capacity */
	return 0;
}

static int battery_get_property(struct power_supply *psy,
								   enum power_supply_property psp,
								   union power_supply_propval *val)
{
/*  struct max77660_charger *chip = container_of(psy, struct max77660_charger, batt_psy); */

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_battery_uvolts();
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_battery_capacity();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif

static int __devinit max77660_charger_probe(struct platform_device *pdev)
{
	struct max77660_chip *pm_chip;
	struct max77660_charger *charger;
	int reg_val;
	int ret = 0;
	int chg_online = 0;

	pr_info("%s ==>\n", __func__);

	pm_chip = dev_get_drvdata(pdev->dev.parent);
	if (pm_chip == NULL) {
		pr_err("%s:no parent data passed in.\n", __func__);
		ret = -EFAULT;
		goto out;
	}

	charger = kzalloc(sizeof(struct max77660_charger),
						GFP_KERNEL);
	if (!charger) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	max77660_chg = charger;
	dev_set_drvdata(&pdev->dev, charger);
	charger->dev = &pdev->dev;

	charger->pdata = pdev->dev.platform_data;
	charger->irq = pdev->resource[0].start;

	charger->usb_psy.name = "usb",
	charger->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	charger->usb_psy.supplied_to = pm_power_supplied_to,
	charger->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	charger->usb_psy.properties = charger_props,
	charger->usb_psy.num_properties = ARRAY_SIZE(charger_props),
	charger->usb_psy.get_property = charger_get_property,
	charger->usb_psy.set_property = charger_set_property,

	charger->dc_psy.name = "dc",
	charger->dc_psy.type = POWER_SUPPLY_TYPE_MAINS,
	charger->dc_psy.supplied_to = pm_power_supplied_to,
	charger->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	charger->dc_psy.properties = charger_props,
	charger->dc_psy.num_properties = ARRAY_SIZE(charger_props),
	charger->dc_psy.get_property = charger_get_property,

#ifndef CONFIG_MAX77660_BATTERY
	charger->batt_psy.name = "battery",
	charger->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY,
	charger->batt_psy.properties = battery_props,
	charger->batt_psy.num_properties = ARRAY_SIZE(battery_props),
	charger->batt_psy.get_property = battery_get_property,
#endif

	ret = power_supply_register(charger->dev, &charger->usb_psy);
	if (ret < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", ret);
		goto free_chg;
	}

	ret = power_supply_register(charger->dev, &charger->dc_psy);
	if (ret < 0) {
		pr_err("power_supply_register dc failed rc = %d\n", ret);
		goto unregister_usb;
	}

#ifndef CONFIG_MAX77660_BATTERY
	ret = power_supply_register(charger->dev, &charger->batt_psy);
	if (ret < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", ret);
		goto unregister_dc;
	}
#endif

	platform_set_drvdata(pdev, charger);

	irq_set_handler_data(charger->irq, (void *)charger);

	ret = request_threaded_irq(charger->irq, NULL, max77660_charger_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW, "max77660-charger-irq", charger);
	if (unlikely(ret < 0)) {
		pr_debug("max77660: failed to request IRQ	%X\n", ret);
		goto unregister_batt;
	}

	charger->dc_online = 0;
	charger->usb_online = 0;
	ret = max77660_read(charger->dev, MAX77660_REG_CHGSTAT, &reg_val, 1, MAX77660_I2C_CHG);
	if (ret >= 0) {
		chg_online = (reg_val & MAX77660_DCUVP_MASK) ? 1 : 0;
		if (chg_online) {
			/* Read USB ID ADC  */
			#if 0
			max77660_adc_read(CHANNEL11_AMUX2, &result);
			if (result.physical < 1000)
				max77660_chg->dc_online = 1;
			else
			#endif
			charger->usb_online = 1;

			/* Set IRQ MASK register */
			reg_val = max77660_chg->pdata->int_mask;
			max77660_write(charger->dev, MAX77660_REG_CHGINTM, &reg_val, 1, MAX77660_I2C_CHG);

			__set_charger(charger, 1);
		}

		notify_usb_of_the_plugin_event(charger->usb_online);
	}

	ret = 0;
	goto out;

unregister_batt:
#ifndef CONFIG_MAX77660_BATTERY
	power_supply_unregister(&charger->batt_psy);
#endif
unregister_dc:
	power_supply_unregister(&charger->dc_psy);
unregister_usb:
	power_supply_unregister(&charger->usb_psy);
free_chg:
	kfree(charger);
out:
	pr_info("%s <== (%d)\n", __func__, ret);
	return ret;
}

static int __devexit max77660_charger_remove(struct platform_device *pdev)
{
	struct max77660_charger *chip = platform_get_drvdata(pdev);

	free_irq(chip->irq, chip);
	power_supply_unregister(&chip->usb_psy);
	power_supply_unregister(&chip->dc_psy);
#ifndef CONFIG_MAX77660_BATTERY
	power_supply_unregister(&chip->batt_psy);
#endif
	kfree(chip);

	return 0;
}

static struct platform_driver max77660_charger_driver = {
	.probe = max77660_charger_probe,
	.remove = __devexit_p(max77660_charger_remove),
	.driver = {
	.name = "max77660-charger",
	.owner = THIS_MODULE,
	},
};

static int __init max77660_charger_init(void)
{
	return platform_driver_register(&max77660_charger_driver);
}

static void __exit max77660_charger_exit(void)
{
	platform_driver_unregister(&max77660_charger_driver);
}

late_initcall(max77660_charger_init);
module_exit(max77660_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maxim Integrated");
MODULE_DESCRIPTION("Charger driver for MAX77660");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:max77660-charger");
