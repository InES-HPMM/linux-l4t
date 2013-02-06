/*
 * include/linux/mfd/max77660-core.h
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __LINUX_MFD_MAX77660_CORE_H__
#define __LINUX_MFD_MAX77660_CORE_H__

#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* i2c slave address */
#define MAX77660_PWR_I2C_ADDR			0x23
#define MAX77660_RTC_I2C_ADDR			0x68
#define MAX77660_CHG_I2C_ADDR			0x22
#define MAX77660_FG_I2C_ADDR			0x36
#define MAX77660_HAPTIC_I2C_ADDR		0x48


/* I2c Slave Id */
#define MAX77660_PWR_SLAVE			0
#define MAX77660_RTC_SLAVE			1
#define MAX77660_CHG_SLAVE			2
#define MAX77660_FG_SLAVE			3
#define MAX77660_HAPTIC_SLAVE			4
#define MAX77660_NUM_SLAVES			5

#define MAX77660_REG_IRQ_TOP1			0x05
#define MAX77660_REG_IRQ_TOP2			0x06
#define MAX77660_REG_IRQ_GLBINT1		0x07
#define MAX77660_REG_IRQ_GLBINT2		0x08
#define MAX77660_REG_IRQ_BUCKINT		0x09
#define MAX77660_REG_IRQ_LDOINT1		0x0A
#define MAX77660_REG_IRQ_LDOINT2		0x0B
#define MAX77660_REG_IRQ_LDOINT3		0x0C
#define MAX77660_REG_GPIO_IRQ1			0x0D
#define MAX77660_REG_GPIO_IRQ2			0x0E

#define MAX77660_REG_IRQ_TOP1_MASK		0x0F
#define MAX77660_REG_IRQ_TOP2_MASK		0x10
#define MAX77660_REG_IRQ_GLBINT1_MASK		0x11
#define MAX77660_REG_IRQ_GLBINT2_MASK		0x12
#define MAX77660_REG_IRQ_BUCKINT_MASK		0x13
#define MAX77660_REG_IRQ_LDOINT1_MASK		0x14
#define MAX77660_REG_IRQ_LDOINT2_MASK		0x15
#define MAX77660_REG_IRQ_LDOINT3_MASK		0x16

#define MAX77660_REG_BUCK_STAT			0x17
#define MAX77660_REG_LDO_STAT1			0x18
#define MAX77660_REG_LDO_STAT2			0x19
#define MAX77660_REG_LDO_STAT3			0x1A

#define MAX77660_REG_BUCK_PWR_MODE1		0x37
#define MAX77660_REG_BUCK_PWR_MODE2		0x38

#define MAX77660_REG_LDO_PWR_MODE1		0x3E
#define MAX77660_REG_LDO_PWR_MODE2		0x3F
#define MAX77660_REG_LDO_PWR_MODE3		0x40
#define MAX77660_REG_LDO_PWR_MODE4		0x41
#define MAX77660_REG_LDO_PWR_MODE5		0x42

/* GPIO Configuration registers */
#define MAX77660_REG_CNFG_GPIO0			0x6A
#define MAX77660_REG_CNFG_GPIO1			0x6B
#define MAX77660_REG_CNFG_GPIO2			0x6C
#define MAX77660_REG_CNFG_GPIO3			0x6D
#define MAX77660_REG_CNFG_GPIO4			0x6E
#define MAX77660_REG_CNFG_GPIO5			0x6F
#define MAX77660_REG_CNFG_GPIO6			0x70
#define MAX77660_REG_CNFG_GPIO7			0x71
#define MAX77660_REG_CNFG_GPIO8			0x72
#define MAX77660_REG_CNFG_GPIO9			0x73

/* Pins configuration registers */
#define MAX77660_REG_PUE1_GPIO			0x74
#define MAX77660_REG_PUE2_GPIO			0x75
#define MAX77660_REG_PDE1_GPIO			0x76
#define MAX77660_REG_PDE2_GPIO			0x77
#define MAX77660_REG_AME1_GPIO			0x78
#define MAX77660_REG_AME2_GPIO			0x79

/* ADC registers */
#define MAX77660_REG_ADCINT			0x7A
#define MAX77660_REG_ADCINTM			0x7B
#define MAX77660_REG_ADCCTRL			0x7C
#define MAX77660_REG_ADCDLY			0x7D
#define MAX77660_REG_ADCSEL0			0x7E
#define MAX77660_REG_ADCSEL1			0x7F
#define MAX77660_REG_ADCCHSEL			0x80
#define MAX77660_REG_ADCDATAL			0x81
#define MAX77660_REG_ADCDATAH			0x82
#define MAX77660_REG_IADC			0x83
#define MAX77660_REG_DTRL			0x84
#define MAX77660_REG_DTRH			0x85
#define MAX77660_REG_DTFL			0x86
#define MAX77660_REG_DTFH			0x87
#define MAX77660_REG_DTOFFL			0x88
#define MAX77660_REG_DTOFFH			0x89

#define MAX77660_ADCINT_DTRINT			BIT(1)
#define MAX77660_ADCINT_DTFINT			BIT(2)
#define MAX77660_ADCINT_ADCCONVINT		BIT(3)
#define MAX77660_ADCINT_ADCCONTINT		BIT(4)
#define MAX77660_ADCINT_ADCTRIGINT		BIT(5)

#define MAX77660_ADCCTRL_ADCEN			BIT(0)
#define MAX77660_ADCCTRL_ADCREFEN		BIT(1)
#define MAX77660_ADCCTRL_ADCAVG_MASK		0x0C
#define MAX77660_ADCCTRL_ADCAVG(n)		(((n) & 0x3) << 2)
#define MAX77660_ADCCTRL_ADCCONV		BIT(4)
#define MAX77660_ADCCTRL_ADCCONT		BIT(5)

#define MAX77660_IADC_IADC(n)			((n) & 0x3)
#define MAX77660_IADC_IADCMUX(ch)		(((ch) & 0x3) << 2)

/* LED controls */
#define MAX77660_REG_LEDEN			0x94
#define MAX77660_REG_LED0BRT			0x95
#define MAX77660_REG_LED1BRT			0x96
#define MAX77660_REG_LED2BRT			0x97
#define MAX77660_REG_LED3BRT			0x98
#define MAX77660_REG_LEDBLNK			0x99

#define MAX77660_REG_GLOBAL_STAT0		0x1
#define MAX77660_REG_GLOBAL_STAT1		0x2


#define MAX77660_REG_GLOBAL_CFG0		0x1C
#define MAX77660_REG_GLOBAL_CFG1		0x1D
#define MAX77660_REG_GLOBAL_CFG2		0x1E
#define MAX77660_REG_GLOBAL_CFG3		0x1F
#define MAX77660_REG_GLOBAL_CFG4		0x20
#define MAX77660_REG_GLOBAL_CFG5		0x21
#define MAX77660_REG_GLOBAL_CFG6		0x22
#define MAX77660_REG_GLOBAL_CFG7		0xC0
#define MAX77660_REG_CNFG32K1			0xA0
#define MAX77660_REG_CNFG32K2			0xA1

#define MAX77660_REG_CID0			0x9A
#define MAX77660_REG_CID1			0x9B
#define MAX77660_REG_CID2			0x9C
#define MAX77660_REG_CID3			0x9D
#define MAX77660_REG_CID4			0x9E
#define MAX77660_REG_CID5			0x9F

/* CID5 details */
#define MAX77660_CID5_DIDM_MASK			0xF0
#define MAX77660_CID5_DIDO_MASK			0x0F
/* Device Identification Metal */
#define MAX77660_CID5_DIDM(n)			(((n) >> 4) & 0xF)
/* Device Indentification OTP */
#define MAX77660_CID5_DIDO(n)			((n) & 0xF)

#define MAX77660_IRQ_TOP1_TOPSYS_MASK		BIT(7)
#define MAX77660_IRQ_TOP1_ADC_MASK		BIT(6)
#define MAX77660_IRQ_TOP1_SIM_MASK		BIT(5)
#define MAX77660_IRQ_TOP1_GPIO_MASK		BIT(4)
#define MAX77660_IRQ_TOP1_RTC_MASK		BIT(3)
#define MAX77660_IRQ_TOP1_CHARGER_MASK		BIT(2)
#define MAX77660_IRQ_TOP1_FUELG_MASK		BIT(1)
#define MAX77660_IRQ_TOP1_OVF_MASK		BIT(0)

#define MAX77660_IRQ_TOP2_BUCK_MASK		BIT(1)
#define MAX77660_IRQ_TOP2_LDO_MASK		BIT(0)

#define MAX77660_IRQ_GLBLINT1_EN0_R_MASK	BIT(7)
#define MAX77660_IRQ_GLBLINT1_EN0_F_MASK	BIT(6)
#define MAX77660_IRQ_GLBLINT1_EN0_1SEC_MASK	BIT(5)
#define MAX77660_IRQ_GLBLINT1_I2CWDT_MASK	BIT(4)
#define MAX77660_IRQ_GLBLINT1_SYSLOW_MASK	BIT(3)
#define MAX77660_IRQ_GLBLINT1_TJALRM1_MASK	BIT(2)
#define MAX77660_IRQ_GLBLINT1_TJALRM2_MASK	BIT(1)
#define MAX77660_IRQ_GLBLINT1_IRQ_M_MASK	BIT(0)

#define MAX77660_IRQ_GLBLINT2_MR_R_MASK		BIT(3)
#define MAX77660_IRQ_GLBLINT2_MR_F_MASK		BIT(2)
#define MAX77660_IRQ_GLBLINT2_WDTWRN_SYS_MASK	BIT(1)
#define MAX77660_IRQ_GLBLINT2_WDTWRN_CHG_MASK	BIT(0)

#define GLBLCNFG0_SFT_OFF_SYSRST_MASK		BIT(3)
#define GLBLCNFG0_SFT_OFF_SYSRST_SHIFT		3
#define GLBLCNFG0_SFT_OFF_OFFRST_MASK		BIT(2)
#define GLBLCNFG0_SFT_OFF_OFFRST_SHIFT		2
#define GLBLCNFG0_SFT_WRST_MASK			BIT(1)
#define GLBLCNFG0_SFT_WRST_SHIFT		1
#define GLBLCNFG0_SFT_CRST_MASK			BIT(0)
#define GLBLCNFG0_SFT_CRST_SHIFT		0

/* GLBLCNFG1: Global Configuration Register 1 */
#define MAX77660_GLBLCNFG1_DISCHGTL		BIT(7)
#define MAX77660_GLBLCNFG1_GLBL_LPM		BIT(6)
#define MAX77660_GLBLCNFG1_WDTEN_SYS		BIT(5)
#define MAX77660_GLBLCNFG1_MRT_MASK		(3 << 3)
#define MAX77660_GLBLCNFG1_SHDN_WRST		BIT(2)
#define MAX77660_GLBLCNFG1_ENCHGTL		BIT(1)
#define MAX77660_GLBLCNFG1_ENPGOC		BIT(0)

/* GLBLCNFG2: Global Configuration Register 2 */
#define MAX77660_GLBLCNFG2_TWD_CHG_MASK		0xC0
#define MAX77660_GLBLCNFG2_TWD_CHG_16		0x00
#define MAX77660_GLBLCNFG2_TWD_CHG_32		0x40
#define MAX77660_GLBLCNFG2_TWD_CHG_64		0x80
#define MAX77660_GLBLCNFG2_TWD_CHG_128		0xC0
#define MAX77660_GLBLCNFG2_TWD_CHG(n)		(((n) & 3) << 6)
#define MAX77660_GLBLCNFG2_TWD_SYS_MASK		0x30
#define MAX77660_GLBLCNFG2_TWD_SYS_16		0x00
#define MAX77660_GLBLCNFG2_TWD_SYS_32		0x10
#define MAX77660_GLBLCNFG2_TWD_SYS_64		0x20
#define MAX77660_GLBLCNFG2_TWD_SYS_128		0x30
#define MAX77660_GLBLCNFG2_TWD_SYS(n)		(((n) & 3) << 4)
#define MAX77660_GLBLCNFG2_RTCWKEN		BIT(3)
#define MAX77660_GLBLCNFG2_WDTWKEN		BIT(2)
#define MAX77660_GLBLCNFG2_MRTOWKEN		BIT(1)

/* GLBLCNFG4: Global Configuration Register 4 */
#define MAX77660_GLBLCNFG4_WDTC_SYS_MASK	0x3
#define MAX77660_GLBLCNFG4_WDTC_SYS_CLR		0x1

#define GLBLCNFG5_EN1_FPS6_MASK_MASK		BIT(4)
#define GLBLCNFG5_EN1_FPS6_MASK_SHIFT		4
#define GLBLCNFG5_EN5_MASK_MASK			BIT(3)
#define GLBLCNFG5_EN5_MASK_SHIFT		3
#define GLBLCNFG5_EN1_MASK_MASK			BIT(2)
#define GLBLCNFG5_EN1_MASK_SHIFT		2
#define GLBLCNFG5_TRSTO_MASK			(BIT(0) | BIT(1))
#define GLBLCNFG5_TRSTO_SHIFT			0


#define GLBLCNFG7_EN4_MASK_MASK			BIT(2)
#define GLBLCNFG7_EN4_MASK_SHIFT		2
#define GLBLCNFG7_EN3_MASK_MASK			BIT(1)
#define GLBLCNFG7_EN3_MASK_SHIFT		1
#define GLBLCNFG7_EN2_MASK_MASK			BIT(0)
#define GLBLCNFG7_EN2_MASK_SHIFT		0

/* MAX77660 GPIO registers */
#define MAX77660_CNFG_GPIO_DRV_MASK		BIT(0)
#define MAX77660_CNFG_GPIO_DRV_PUSHPULL		BIT(0)
#define MAX77660_CNFG_GPIO_DRV_OPENDRAIN	0
#define MAX77660_CNFG_GPIO_DIR_MASK		BIT(1)
#define MAX77660_CNFG_GPIO_DIR_INPUT		BIT(1)
#define MAX77660_CNFG_GPIO_DIR_OUTPUT		0
#define MAX77660_CNFG_GPIO_INPUT_VAL_MASK	BIT(2)
#define MAX77660_CNFG_GPIO_OUTPUT_VAL_MASK	BIT(3)
#define MAX77660_CNFG_GPIO_OUTPUT_VAL_HIGH	BIT(3)
#define MAX77660_CNFG_GPIO_OUTPUT_VAL_LOW	0
#define MAX77660_CNFG_GPIO_INT_MASK		(0x3 << 4)
#define MAX77660_CNFG_GPIO_INT_FALLING		BIT(4)
#define MAX77660_CNFG_GPIO_INT_RISING		BIT(5)
#define MAX77660_CNFG_GPIO_DBNC_MASK		(0x3 << 6)
#define MAX77660_CNFG_GPIO_DBNC_None		(0x0 << 6)
#define MAX77660_CNFG_GPIO_DBNC_8ms		(0x1 << 6)
#define MAX77660_CNFG_GPIO_DBNC_16ms		(0x2 << 6)
#define MAX77660_CNFG_GPIO_DBNC_32ms		(0x3 << 6)

#define MAX77660_REG_IRQ1_LVL2_GPIO		0x0D
#define MAX77660_REG_IRQ2_LVL2_GPIO		0x0E

#define MAX77660_IRQ1_LVL2_GPIO_EDGE0		BIT(0)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE1		BIT(1)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE2		BIT(2)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE3		BIT(3)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE4		BIT(4)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE5		BIT(5)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE6		BIT(6)
#define MAX77660_IRQ1_LVL2_GPIO_EDGE7		BIT(7)

#define MAX77660_IRQ2_LVL2_GPIO_EDGE8		BIT(0)
#define MAX77660_IRQ2_LVL2_GPIO_EDGE9		BIT(1)

#define PWR_MODE_32KCLK_MASK			(BIT(1) | BIT(0))
#define OUT1_EN_32KCLK_MASK			BIT(2)
#define OUT1_EN_32KCLK_SHIFT			2
#define OUT2_EN_32KCLK_MASK			BIT(3)
#define OUT2_EN_32KCLK_SHIFT			3

#define CID_DIDM_MASK				0xF0
#define CID_DIDM_SHIFT				4
#define CID_DIDO_MASK				0xF
#define CID_DIDO_SHIFT				0


#define IRQ_GPIO_BASE				MAX77660_IRQ_GPIO0
#define IRQ_GPIO_END				MAX77660_IRQ_GPIO9

/* RTC register set */
#define MAX77660_RTC_IRQ			0x00
#define MAX77660_RTC_IRQ_MASK			0x01
#define MAX77660_RTC_CTRL_MODE			0x02
#define MAX77660_RTC_CTRL			0x03
#define MAX77660_RTC_UPDATE0			0x04
#define MAX77660_RTC_UPDATE1			0x05
#define MAX77660_RTC_SMPL			0x06
#define MAX77660_RTC_SEC			0x07
#define MAX77660_RTC_MIN			0x08
#define MAX77660_RTC_HOUR			0x09
#define MAX77660_RTC_WEEKDAY			0x0A
#define MAX77660_RTC_MONTH			0x0B
#define MAX77660_RTC_YEAR			0x0C
#define MAX77660_RTC_MONTHDAY			0x0D
#define MAX77660_RTC_AE1			0x0E
#define MAX77660_RTC_ALARM_SEC1			0x0F
#define MAX77660_RTC_ALARM_MIN1			0x10
#define MAX77660_RTC_ALARM_HOUR1		0x11
#define MAX77660_RTC_ALARM_WEEKDAY1		0x12
#define MAX77660_RTC_ALARM_MONTH1		0x13
#define MAX77660_RTC_ALARM_YEAR1		0x14
#define MAX77660_RTC_ALARM_MONTHDAY1		0x15
#define MAX77660_RTC_AE2			0x16
#define MAX77660_RTC_ALARM_SEC2			0x17
#define MAX77660_RTC_ALARM_MIN2			0x18
#define MAX77660_RTC_ALARM_HOUR2		0x19
#define MAX77660_RTC_ALARM_WEEKDAY2		0x1A
#define MAX77660_RTC_ALARM_MONTH2		0x1B
#define MAX77660_RTC_ALARM_YEAR2		0x1C
#define MAX77660_RTC_ALARM_MONTHDAY2		0x1D

#define MAX77660_RTC_IRQ_60SEC_MASK		BIT(0)
#define MAX77660_RTC_IRQ_ALARM1_MASK		BIT(1)
#define MAX77660_RTC_IRQ_ALARM2_MASK		BIT(2)
#define MAX77660_RTC_IRQ_SMPL_MASK		BIT(3)
#define MAX77660_RTC_IRQ_1SEC_MASK		BIT(4)

#define MAX77660_RTC_WB_UPDATE_MASK		BIT(0)
#define MAX77660_RTC_FREEZE_SEC_MASK		BIT(2)
#define MAX77660_RTC_RTC_WAKE_MASK		BIT(3)
#define MAX77660_RTC_RB_UPDATE_MASK		BIT(4)

#define MAX77660_RTCCNTLM_MASK			(BIT(0) | BIT(1))
#define MAX77660_RTCCNTL_BCD_MODE		BIT(0)
#define MAX77660_RTCCNTL_HRMODE_24		BIT(1)

#define MAX77660_RTC_SEC_MASK			0x7F
#define MAX77660_RTC_MIN_MASK			0x7F
#define MAX77660_RTC_HOUR_MASK			0x3F
#define MAX77660_RTC_WEEKDAY_MASK		0x7F
#define MAX77660_RTC_MONTH_MASK			0x1F
#define MAX77660_RTC_YEAR_MASK			0xFF
#define MAX77660_RTC_MONTHDAY_MASK		0x3F

/* Charger registers */

#define MAX77660_CHARGER_USBCHGCTRL		0x00
#define MAX77660_CHARGER_CHGINT			0x5D
#define MAX77660_CHARGER_CHGINTM		0x5E
#define MAX77660_CHARGER_CHGSTAT		0x5F
#define MAX77660_CHARGER_DETAILS1		0x60
#define MAX77660_CHARGER_DETAILS2		0x61
#define MAX77660_CHARGER_DETAILS3		0x62
#define MAX77660_CHARGER_BAT2SYS		0x63
#define MAX77660_CHARGER_CHGCTRL1		0x64
#define MAX77660_CHARGER_FCHGCRNT		0x65
#define MAX77660_CHARGER_TOPOFF			0x66
#define MAX77660_CHARGER_BATREGCTRL		0x67
#define MAX77660_CHARGER_DCCRNT			0x68
#define MAX77660_CHARGER_AICLCNTL		0x69
#define MAX77660_CHARGER_RBOOST			0x6A
#define MAX77660_CHARGER_CHGCTRL2		0x6B
#define MAX77660_CHARGER_BATDET			0x6C
#define MAX77660_CHARGER_CHGCCMAX		0x6D
#define MAX77660_CHARGER_MBATREGMAX		0x6E

#define MAX77660_CHG_CHGINT_DC_UVP		BIT(4)

#define MAX77660_RBOOST_RBOOSTEN		BIT(0)
#define MAX77660_RBOOST_RBOUT_MASK		0x1E
#define MAX77660_RBOOST_RBOUT_VOUT(n)		(((n) & 0xF) << 1)
#define MAX77660_RBOOST_RBFORCEPWM		BIT(5)
#define MAX77660_RBOOST_BSTSLEWRATE_MASK	0xC0

#define MAX77660_BUCK2_PWR_MODE_MASK		(BIT(2) | BIT(3))

/*
 * Interrupts
 */
enum {
	MAX77660_IRQ_INT_TOP_OVF,	/* If this bit is set read from TOP2  */
	MAX77660_IRQ_FG,		/* FG */
	MAX77660_IRQ_CHG,		/* CHG */
	MAX77660_IRQ_RTC,		/* RTC */
	MAX77660_IRQ_INT_TOP_GPIO,	/* TOP GPIO internal int to max77660 */
	MAX77660_IRQ_SIM,		/* SIM interrupt */
	MAX77660_IRQ_ADC,		/* ADC interrupt */
	MAX77660_IRQ_TOPSYSINT,		/* TOPSYS interrupt */
	MAX77660_IRQ_LDOINT,		/* LDO power fail */
	MAX77660_IRQ_BUCKINT,		/* BUCK power fail */

	MAX77660_IRQ_GLBL_BASE,
	/* Thermal alarm status, > 140C */
	MAX77660_IRQ_GLBL_TJALRM2 = MAX77660_IRQ_GLBL_BASE,
	MAX77660_IRQ_GLBL_TJALRM1,	/* Thermal alarm status, > 120C */
	MAX77660_IRQ_GLBL_SYSLOW,	/* Low main battery interrupt */
	MAX77660_IRQ_GLBL_I2C_WDT,	/* I2C watchdog timeout interrupt */
	MAX77660_IRQ_GLBL_EN0_1SEC,	/* EN0 Active for 1 sec interrupt */
	MAX77660_IRQ_GLBL_EN0_F,	/* EN0 Falling interrupt */
	MAX77660_IRQ_GLBL_EN0_R,	/* EN0 Rising interrupt */
	MAX77660_IRQ_GLBL_WDTWRN_CHG,	/* Charger watchdog timer warning int */
	MAX77660_IRQ_GLBL_WDTWRN_SYS,	/* System watchdog timer warning int */
	MAX77660_IRQ_GLBL_MR_F,		/* Manual reset falling interrupt */
	MAX77660_IRQ_GLBL_MR_R,		/* Manual reset rising interrupt */

	MAX77660_IRQ_GPIO0,		/* GPIO0 edge detection */
	MAX77660_IRQ_GPIO1,		/* GPIO1 edge detection */
	MAX77660_IRQ_GPIO2,		/* GPIO2 edge detection */
	MAX77660_IRQ_GPIO3,		/* GPIO3 edge detection */
	MAX77660_IRQ_GPIO4,		/* GPIO4 edge detection */
	MAX77660_IRQ_GPIO5,		/* GPIO5 edge detection */
	MAX77660_IRQ_GPIO6,		/* GPIO6 edge detection */
	MAX77660_IRQ_GPIO7,		/* GPIO7 edge detection */
	MAX77660_IRQ_GPIO8,		/* GPIO7 edge detection */
	MAX77660_IRQ_GPIO9,		/* GPIO7 edge detection */

	MAX77660_IRQ_HAPTIC,		/* HAPTIC */
	MAX77660_IRQ_NR,
};

/*
 *GPIOs
 */
enum {
	MAX77660_GPIO0,
	MAX77660_GPIO1,
	MAX77660_GPIO2,
	MAX77660_GPIO3,
	MAX77660_GPIO4,
	MAX77660_GPIO5,
	MAX77660_GPIO6,
	MAX77660_GPIO7,
	MAX77660_GPIO8,
	MAX77660_GPIO9,

	MAX77660_GPIO_NR,
};

/* Max77660 Chip data */
struct max77660_chip {
	struct device *dev;

	struct i2c_client *clients[MAX77660_NUM_SLAVES];
	struct regmap *rmap[MAX77660_NUM_SLAVES];

	struct max77660_platform_data *pdata;

	int chip_irq;
	int irq_base;

	int es_minor_version;
	int es_major_version;

	struct regmap_irq_chip_data *top_irq_data;
	struct regmap_irq_chip_data *global_irq_data;
	struct regmap_irq_chip_data *gpio_irq_data;
};

enum max77660_pull_up_down {
	MAX77660_PIN_DEFAULT,
	MAX77660_PIN_PULL_UP,
	MAX77660_PIN_PULL_DOWN,
	MAX77660_PIN_PULL_NORMAL,
};

enum MAX77660_PINS {
	MAX77660_PINS_GPIO0,
	MAX77660_PINS_GPIO1,
	MAX77660_PINS_GPIO2,
	MAX77660_PINS_GPIO3,
	MAX77660_PINS_GPIO4,
	MAX77660_PINS_GPIO5,
	MAX77660_PINS_GPIO6,
	MAX77660_PINS_GPIO7,
	MAX77660_PINS_GPIO8,
	MAX77660_PINS_GPIO9,
	MAX77660_PINS_MAX,
};

enum MAX77660_ADC_CHANNELS {
	MAX77660_ADC_CH_VBYP,
	MAX77660_ADC_CH_TDIE,
	MAX77660_ADC_CH_VBBATT,
	MAX77660_ADC_CH_VSYS,
	MAX77660_ADC_CH_VDCIN,
	MAX77660_ADC_CH_VWCSNS,
	MAX77660_ADC_CH_VTHM,
	MAX77660_ADC_CH_VICHG,
	MAX77660_ADC_CH_VMBATDET,
	MAX77660_ADC_CH_VMBAT,
	MAX77660_ADC_CH_ADC0,
	MAX77660_ADC_CH_ADC1,
	MAX77660_ADC_CH_ADC2,
	MAX77660_ADC_CH_ADC3,
	MAX77660_ADC_CH_MAX,
};

enum max77660_regulator_id {
	MAX77660_REGULATOR_ID_BUCK1,
	MAX77660_REGULATOR_ID_BUCK2,
	MAX77660_REGULATOR_ID_BUCK3,
	MAX77660_REGULATOR_ID_BUCK4,
	MAX77660_REGULATOR_ID_BUCK5,
	MAX77660_REGULATOR_ID_BUCK6,
	MAX77660_REGULATOR_ID_BUCK7,
	MAX77660_REGULATOR_ID_LDO1,
	MAX77660_REGULATOR_ID_LDO2,
	MAX77660_REGULATOR_ID_LDO3,
	MAX77660_REGULATOR_ID_LDO4,
	MAX77660_REGULATOR_ID_LDO5,
	MAX77660_REGULATOR_ID_LDO6,
	MAX77660_REGULATOR_ID_LDO7,
	MAX77660_REGULATOR_ID_LDO8,
	MAX77660_REGULATOR_ID_LDO9,
	MAX77660_REGULATOR_ID_LDO10,
	MAX77660_REGULATOR_ID_LDO11,
	MAX77660_REGULATOR_ID_LDO12,
	MAX77660_REGULATOR_ID_LDO13,
	MAX77660_REGULATOR_ID_LDO14,
	MAX77660_REGULATOR_ID_LDO15,
	MAX77660_REGULATOR_ID_LDO16,
	MAX77660_REGULATOR_ID_LDO17,
	MAX77660_REGULATOR_ID_LDO18,
	MAX77660_REGULATOR_ID_SW1,
	MAX77660_REGULATOR_ID_SW2,
	MAX77660_REGULATOR_ID_SW3,
	MAX77660_REGULATOR_ID_SW4,
	MAX77660_REGULATOR_ID_SW5,
	MAX77660_REGULATOR_ID_NR,
};

/* FPS Power Up/Down Period */
enum max77660_regulator_fps_power_period {
	FPS_POWER_PERIOD_0,
	FPS_POWER_PERIOD_1,
	FPS_POWER_PERIOD_2,
	FPS_POWER_PERIOD_3,
	FPS_POWER_PERIOD_DEF = -1,
};

/* FPS Time Period */
enum max77660_regulator_fps_time_period {
	FPS_TIME_PERIOD_31US,	/* 0b000 */
	FPS_TIME_PERIOD_61US,	/* 0b001 */
	FPS_TIME_PERIOD_122US,	/* 0b010 */
	FPS_TIME_PERIOD_244US,	/* 0b011 */
	FPS_TIME_PERIOD_488US,	/* 0b100 */
	FPS_TIME_PERIOD_977US,	/* 0b101 */
	FPS_TIME_PERIOD_1953US,	/* 0b110 */
	FPS_TIME_PERIOD_3960US,	/* 0b111 */
	FPS_TIME_PERIOD_DEF = -1,
};

/* FPS Source */
enum max77660_regulator_fps_src {
	FPS_SRC_0,
	FPS_SRC_1,
	FPS_SRC_2,
	FPS_SRC_3,
	FPS_SRC_4,
	FPS_SRC_5,
	FPS_SRC_6,
	FPS_SRC_NONE,
	FPS_SRC_DEF = -1,
};

#define max77660_rails(_name)		"max77660_"#_name

/* SD Forced PWM Mode */
#define SD_FORCED_PWM_MODE		0x20

/* SD Failling Slew Rate Active-Discharge Mode */
#define SD_FSRADE_DISABLE		0x40

/* Group Low-Power Mode */
#define GLPM_ENABLE			0x80

/* EN enable */
#define ENABLE_EN			0x01
#define ENABLE_EN1			(0x02 | ENABLE_EN)
#define ENABLE_EN2			(0x04 | ENABLE_EN)
#define ENABLE_EN3			(0x08 | ENABLE_EN)

/* Disable DVFS */
#define DISABLE_DVFS			0x10

/* Tracking for LDO4 */
#define LDO4_EN_TRACKING		0x100

struct max77660_regulator_fps_cfg {
	enum max77660_regulator_fps_time_period tu_ap_off;
	enum max77660_regulator_fps_time_period td_ap_off;
	enum max77660_regulator_fps_time_period tu_ap_slp;
	enum max77660_regulator_fps_time_period td_ap_slp;
	enum max77660_regulator_fps_time_period tu_fps_6;
	enum max77660_regulator_fps_time_period td_fps_6;
};

struct max77660_regulator_platform_data {
	struct regulator_init_data *reg_init_data;
	enum max77660_regulator_fps_src fps_src;
	enum max77660_regulator_fps_power_period fps_pu_period;
	enum max77660_regulator_fps_power_period fps_pd_period;

	int num_fps_cfgs;
	struct max77660_regulator_fps_cfg *fps_cfgs;

	unsigned int flags;
};

/*
 * max77660_pinctrl_platform_data: Pin control platform data.
 * @pin_id: Pin ID.
 * @gpio_pin_mode: GPIO pin mode, 1 for GPIO, 0 for Alternate.
 * @open_drain: Open drain, 1 for open drain mode, 0 for normal push pull.
 * pullup_dn_normal: Pull up/down/normal.
 */
struct max77660_pinctrl_platform_data {
	int pin_id;
	unsigned gpio_pin_mode:1;
	unsigned open_drain:1;
	int pullup_dn_normal;
};

/*
 * max77660_charger_platform_data: Platform data for charger.
 */
struct max77660_charger_platform_data {
	struct regulator_init_data *vbus_reg_init_data;
};

/*
 * max77660_adc_platform_data: Platform data for ADC.
 * @adc_current_uA: ADC current source in uA.
 * @adc_avg_sample: Average ADC sample. 0 Means 1 sample.
 * @adc_ref_enabled: ADC reference enabled.
 */
struct max77660_adc_platform_data {
	int adc_current_uA;
	int adc_avg_sample;
	unsigned adc_ref_enabled:1;
};

/*
 * Flags
 */
#define SLP_LPM_ENABLE		0x01

/*
 * max77660_pwm_dvfs_init_data: PWM based DVFS init data.
 * @en_pwm: Enable PWM.
 * @step_voltage_uV: step voltage for DVFS VOUT: 6.125mV, 12.25mV and 25mV.
 * @default_voltage_uV: default voltage for DVFS: 0.6V to 1.5V.
 * @base_voltage_uV: base voltage for DVFS voltage calculation: 0.6V to 1.5V.
 * @max_voltage_uV: maximum voltage for DVFS: 0.6V to 1.5V.
 */
struct max77660_pwm_dvfs_init_data {
	bool	en_pwm;
	int	step_voltage_uV;
	int	default_voltage_uV;
	int	base_voltage_uV;
	int	max_voltage_uV;
};

struct max77660_fg_platform_data {
	u8 valrt_min;
	u8 valrt_max;
	bool alsc;
	bool alrt;
	bool athd;
 };

/*
 * max77660_platform_data: Platform data for MAX77660.
 * @pinctrl_pdata: Pincontrol configurations.
 * @num_pinctrl: Number of pin control data.
 * charger_pdata: Charger platform data.
 * @system_watchdog_timeout: System wathdog timeout in seconds. If this value
 *		is -ve then timer will not start during initialisation.
 * led_disable: Disable LEDs.
 */
struct max77660_platform_data {
	int irq_base;
	int gpio_base;

	struct max77660_regulator_platform_data *regulator_pdata[MAX77660_REGULATOR_ID_NR];

	struct max77660_pinctrl_platform_data *pinctrl_pdata;
	int num_pinctrl;

	struct max77660_charger_platform_data *charger_pdata;

	struct max77660_adc_platform_data *adc_pdata;
	unsigned int flags;

	bool en_buck2_ext_ctrl;
	bool en_clk32out1;
	bool en_clk32out2;
	bool use_power_off;

	int system_watchdog_timeout;
	bool led_disable;
	struct max77660_pwm_dvfs_init_data dvfs_pd;
};

static inline int max77660_reg_write(struct device *dev, int sid,
		int reg, u8 val)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_write(chip->rmap[sid], reg, val);
}

static inline int max77660_reg_writes(struct device *dev, int sid, int reg,
		int len, void *val)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_bulk_write(chip->rmap[sid], reg, val, len);
}

static inline int max77660_reg_read(struct device *dev, int sid,
		int reg, u8 *val)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);
	unsigned int ival;
	int ret;

	ret = regmap_read(chip->rmap[sid], reg, &ival);
	if (ret < 0) {
		dev_err(dev, "failed reading from reg 0x%02x\n", reg);
		return ret;
	}
	*val = ival;
	return ret;
}

static inline int max77660_reg_reads(struct device *dev, int sid,
		int reg, int len, void *val)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_bulk_read(chip->rmap[sid], reg, val, len);
}

static inline int max77660_reg_set_bits(struct device *dev, int sid,
		int reg, u8 bit_mask)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_update_bits(chip->rmap[sid], reg,
				bit_mask, bit_mask);
}

static inline int max77660_reg_clr_bits(struct device *dev, int sid,
		int reg, u8 bit_mask)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_update_bits(chip->rmap[sid], reg, bit_mask, 0);
}

static inline int max77660_reg_update(struct device *dev, int sid,
		int reg, u8 val, uint8_t mask)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	return regmap_update_bits(chip->rmap[sid], reg, mask, val);
}

static inline int max77660_get_es_version(struct device *dev,
		int *major, int *minor)
{
	struct max77660_chip *chip = dev_get_drvdata(dev);

	*minor = chip->es_minor_version;
	*major = chip->es_major_version;

	return 0;
}

static inline int max77660_is_es_1_0(struct device *dev)
{
	int minor, major;

	max77660_get_es_version(dev->parent, &major, &minor);
	if ((major == 1) && (minor == 0))
		return true;
	return false;
}

#endif /* __LINUX_MFD_MAX77660_CORE_H__ */
