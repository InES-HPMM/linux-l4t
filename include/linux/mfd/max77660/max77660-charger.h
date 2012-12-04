
/*
 * MAX77660 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77660_CHARGER_H__
#define __MAX77660_CHARGER_H__

#include <linux/errno.h>
#include <linux/power_supply.h>

/* 0 and from 250mA to 1550mA in 50mA steps. */
#define FCHG_CURRENT(x) ((x-250)/50+5)

enum {
	MAX77660_FCHGTIME_DISABLE,
	MAX77660_FCHGTIME_4HRS,
	MAX77660_FCHGTIME_5HRS,
	MAX77660_FCHGTIME_6HRS,
	MAX77660_FCHGTIME_7HRS,
	MAX77660_FCHGTIME_8HRS,
	MAX77660_FCHGTIME_9HRS,
	MAX77660_FCHGTIME_10HRS,
};

enum {
	MAX77660_TOPOFFTIME_0MIN,
	MAX77660_TOPOFFTIME_10MIN,
	MAX77660_TOPOFFTIME_20MIN,
	MAX77660_TOPOFFTIME_30MIN,
	MAX77660_TOPOFFTIME_40MIN,
	MAX77660_TOPOFFTIME_50MIN,
	MAX77660_TOPOFFTIME_60MIN,
	MAX77660_TOPOFFTIME_70MIN,
};

enum {
	MAX77660_TOPOFFTSHLD_50mA,
	MAX77660_TOPOFFTSHLD_100mA,
	MAX77660_TOPOFFTSHLD_150mA,
	MAX77660_TOPOFFTSHLD_200mA,
};

enum {
	MAX77660_CHGCV_4P20V,
	MAX77660_CHGCV_4P10V,
	MAX77660_CHGCV_4P35V,
};

enum {
	MAX77660_CHGRSTRT_150mV,
	MAX77660_CHGRSTRT_100mV,
};

#define DCI_LIMIT(x) ((x <= 100) ? 0 :		\
		(x >= 250 && x <= 1550) ? ((x-250)/25+10) : -EINVAL)

enum {
	MAX77660_REGTEMP_105degree,
	MAX77660_REGTEMP_90degree,
	MAX77660_REGTEMP_120degree,
	MAX77660_REGTEMP_DISABLE,
};

#define MAX77660_CHGPROT_LOCKED	  0x00
#define MAX77660_CHGPROT_UNLOCKED	0x03

/* IRQ definitions */
enum {
	MAX77660_CHG_BAT_I = 0,
	MAX77660_CHG_CHG_I,
	MAX77660_CHG_DC_UVP,
	MAX77660_CHG_DC_OVP,
	MAX77660_CHG_DC_I,
	MAX77660_CHG_DC_V,
	MAX77660_CHG_NR_IRQS,
};

struct max77660_charger_platform_data {
	u8	  chgcc;			  /* Fast Charge Current */
	u8	  fchgtime;		   /* Fast Charge Time  */

	u8	  chgrstrt;		   /* Fast Charge Restart Threshold */
	u8	  dcilmt;			 /* Input Current Limit Selection */

	u8	  topofftime;		 /* Top Off Timer Setting  */
	u8	  topofftshld;		/* Done Current Threshold */
	u8	  chgcv;			  /* Charger Termination Voltage */

	u8	  regtemp;			/* Die temperature thermal regulation loop setpoint */

	u8	  int_mask;		   /* CHGINT_MASK */
};

int max77660_stop_charging(void);
int max77660_start_charging(unsigned mA);

int max77660_charger_register_vbus_sn(void (*callback)(int));
void max77660_charger_unregister_vbus_sn(void (*callback)(int));
int max77660_charger_vbus_in(void);
void max77660_charger_vbus_draw(unsigned int mA);
int max77660_disable_source_current(bool disable);
int max77660_set_usb_power_supply_type(enum power_supply_type type);
#endif /* __MAX77660_CHARGER_H__ */
