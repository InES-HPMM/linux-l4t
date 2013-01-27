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


/* i2c slave address */
#define MAX77660_PWR_I2C_ADDR		0x23
#define MAX77660_RTC_I2C_ADDR		0x68
#define MAX77660_CHG_I2C_ADDR		0x22
#define MAX77660_FG_I2C_ADDR		0x36
#define MAX77660_HAPTIC_I2C_ADDR	0x48


/* I2c Slave Id */
#define MAX77660_PWR_SLAVE		0
#define MAX77660_RTC_SLAVE		1
#define MAX77660_CHG_SLAVE		2
#define MAX77660_FG_SLAVE		3
#define MAX77660_HAPTIC_SLAVE		4
#define MAX77660_NUM_SLAVES		5

/* Registers */
#if 0
#define MAX77660_REG_IRQ_TOP		0x05
#define MAX77660_REG_LBT_IRQ		0x06
#define MAX77660_REG_SD_IRQ		0x07
#define MAX77660_REG_LDOX_IRQ		0x08
#define MAX77660_REG_LDO8_IRQ		0x09
#define MAX77660_REG_GPIO_IRQ		0x0A
#define MAX77660_REG_ONOFF_IRQ		0x0B
#define MAX77660_REG_NVER		0x0C
#define MAX77660_REG_IRQ_TOP_MASK	0x0D
#define MAX77660_REG_LBT_IRQ_MASK	0x0E
#define MAX77660_REG_SD_IRQ_MASK	0x0F
#define MAX77660_REG_LDOX_IRQ_MASK	0x10
#define MAX77660_REG_LDO8_IRQ_MASK	0x11
#define MAX77660_REG_ONOFF_IRQ_MASK	0x12
#else
#define MAX77660_REG_IRQ_TOP1		0x05
#define MAX77660_REG_IRQ_TOP2		0x06
#define MAX77660_REG_IRQ_GLBINT1	0x07
#define MAX77660_REG_IRQ_GLBINT2	0x08
#define MAX77660_REG_IRQ_BUCKINT    0x09
#define MAX77660_REG_IRQ_LDOINT1    0x0A
#define MAX77660_REG_IRQ_LDOINT2    0x0B
#define MAX77660_REG_IRQ_LDOINT3    0x0C
#define MAX77660_REG_GPIO_IRQ1      0x0D
#define MAX77660_REG_GPIO_IRQ2      0x0E

#define MAX77660_REG_IRQ_TOP1_MASK	0x0F
#define MAX77660_REG_IRQ_TOP2_MASK	0x10
#define MAX77660_REG_IRQ_GLBINT1_MASK	 0x11
#define MAX77660_REG_IRQ_GLBINT2_MASK	 0x12
#define MAX77660_REG_IRQ_BUCKINT_MASK    0x13
#define MAX77660_REG_IRQ_LDOINT1_MASK    0x14
#define MAX77660_REG_IRQ_LDOINT2_MASK    0x15
#define MAX77660_REG_IRQ_LDOINT3_MASK    0x16

#define MAX77660_REG_BUCK_STAT           0x17
#define MAX77660_REG_LDO_STAT1           0x18
#define MAX77660_REG_LDO_STAT2           0x19
#define MAX77660_REG_LDO_STAT3           0x1A


#endif
#define MAX77660_REG_BUCK_PWR_MODE1	0x37
#define MAX77660_REG_BUCK_PWR_MODE2	0x38

#define MAX77660_REG_LDO_PWR_MODE1	0x3E
#define MAX77660_REG_LDO_PWR_MODE2	0x3F
#define MAX77660_REG_LDO_PWR_MODE3	0x40
#define MAX77660_REG_LDO_PWR_MODE4	0x41
#define MAX77660_REG_LDO_PWR_MODE5	0x42

#define MAX77660_REG_GPIO_CTRL0		0x6A
#define MAX77660_REG_GPIO_CTRL1		0x6B
#define MAX77660_REG_GPIO_CTRL2		0x6C
#define MAX77660_REG_GPIO_CTRL3		0x6D
#define MAX77660_REG_GPIO_CTRL4		0x6E
#define MAX77660_REG_GPIO_CTRL5		0x6F
#define MAX77660_REG_GPIO_CTRL6		0x70
#define MAX77660_REG_GPIO_CTRL7		0x71
#define MAX77660_REG_GPIO_CTRL8		0x72
#define MAX77660_REG_GPIO_CTRL9		0x73

/* GPIO Configuration registers */
#define MAX77660_REG_CNFG_GPIO0         0x6A
#define MAX77660_REG_CNFG_GPIO1         0x6B
#define MAX77660_REG_CNFG_GPIO2         0x6C
#define MAX77660_REG_CNFG_GPIO3         0x6D
#define MAX77660_REG_CNFG_GPIO4         0x6E
#define MAX77660_REG_CNFG_GPIO5         0x6F
#define MAX77660_REG_CNFG_GPIO6         0x70
#define MAX77660_REG_CNFG_GPIO7         0x71
#define MAX77660_REG_CNFG_GPIO8         0x72
#define MAX77660_REG_CNFG_GPIO9         0x73

/* Pins configuration registers */
#define MAX77660_REG_PUE1_GPIO          0x74
#define MAX77660_REG_PUE2_GPIO          0x75
#define MAX77660_REG_PDE1_GPIO          0x76
#define MAX77660_REG_PDE2_GPIO          0x77
#define MAX77660_REG_AME1_GPIO          0x78
#define MAX77660_REG_AME2_GPIO          0x78

#if 0
#define MAX77660_REG_GPIO_PU		0x3E
#define MAX77660_REG_GPIO_PD		0x3F
#define MAX77660_REG_GPIO_ALT		0x40
#define MAX77660_REG_ONOFF_CFG1		0x41
#define MAX77660_REG_ONOFF_CFG2		0x42
#define MAX77660_REG_CID5			0x5D
#else
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

#define MAX77660_REG_CID0				0x9A
#define MAX77660_REG_CID1				0x9B
#define MAX77660_REG_CID2				0x9C
#define MAX77660_REG_CID3				0x9D
#define MAX77660_REG_CID4				0x9E
#define MAX77660_REG_CID5				0x9F

#endif

#define MAX77660_IRQ_TOP1_TOPSYS_MASK			BIT(7)
#define MAX77660_IRQ_TOP1_ADC_MASK			BIT(6)
#define MAX77660_IRQ_TOP1_SIM_MASK			BIT(5)
#define MAX77660_IRQ_TOP1_GPIO_MASK			BIT(4)
#define MAX77660_IRQ_TOP1_RTC_MASK			BIT(3)
#define MAX77660_IRQ_TOP1_CHARGER_MASK			BIT(2)
#define MAX77660_IRQ_TOP1_FUELG_MASK			BIT(1)
#define MAX77660_IRQ_TOP1_OVF_MASK			BIT(0)

#define MAX77660_IRQ_TOP2_BUCK_MASK			BIT(1)
#define MAX77660_IRQ_TOP2_LDO_MASK			BIT(0)

#define MAX77660_IRQ_GLBLINT1_EN0_R_MASK		BIT(7)
#define MAX77660_IRQ_GLBLINT1_EN0_F_MASK		BIT(6)
#define MAX77660_IRQ_GLBLINT1_EN0_1SEC_MASK		BIT(5)
#define MAX77660_IRQ_GLBLINT1_I2CWDT_MASK		BIT(4)
#define MAX77660_IRQ_GLBLINT1_SYSLOW_MASK		BIT(3)
#define MAX77660_IRQ_GLBLINT1_TJALRM1_MASK		BIT(2)
#define MAX77660_IRQ_GLBLINT1_TJALRM2_MASK		BIT(1)
#define MAX77660_IRQ_GLBLINT1_IRQ_M_MASK		BIT(0)

#define MAX77660_IRQ_GLBLINT2_MR_R_MASK			BIT(3)
#define MAX77660_IRQ_GLBLINT2_MR_F_MASK			BIT(2)
#define MAX77660_IRQ_GLBLINT2_WDTWRN_SYS_MASK		BIT(1)
#define MAX77660_IRQ_GLBLINT2_WDTWRN_CHG_MASK		BIT(0)

#define GLBLCNFG0_SFT_OFF_SYSRST_MASK		BIT(3)
#define GLBLCNFG0_SFT_OFF_SYSRST_SHIFT			3
#define GLBLCNFG0_SFT_OFF_OFFRST_MASK		BIT(2)
#define GLBLCNFG0_SFT_OFF_OFFRST_SHIFT			2
#define GLBLCNFG0_SFT_WRST_MASK				BIT(1)
#define GLBLCNFG0_SFT_WRST_SHIFT				1
#define GLBLCNFG0_SFT_CRST_MASK				BIT(0)
#define GLBLCNFG0_SFT_CRST_SHIFT				0


/* GLBLCNFG1: Global Configuration Register 1 */
#define MAX77660_GLBLCNFG1_DISCHGTL			BIT(7)
#define MAX77660_GLBLCNFG1_GLBL_LPM			BIT(6)
#define MAX77660_GLBLCNFG1_WDTEN_SYS			BIT(5)
#define MAX77660_GLBLCNFG1_MRT_MASK			(3 << 3)
#define MAX77660_GLBLCNFG1_SHDN_WRST			BIT(2)
#define MAX77660_GLBLCNFG1_ENCHGTL			BIT(1)
#define MAX77660_GLBLCNFG1_ENPGOC			BIT(0)

/* GLBLCNFG2: Global Configuration Register 2 */
#define MAX77660_GLBLCNFG2_TWD_CHG_MASK			0xC0
#define MAX77660_GLBLCNFG2_TWD_CHG_16			0x00
#define MAX77660_GLBLCNFG2_TWD_CHG_32			0x40
#define MAX77660_GLBLCNFG2_TWD_CHG_64			0x80
#define MAX77660_GLBLCNFG2_TWD_CHG_128			0xC0
#define MAX77660_GLBLCNFG2_TWD_CHG(n)			(((n) & 3) << 6)
#define MAX77660_GLBLCNFG2_TWD_SYS_MASK			0x30
#define MAX77660_GLBLCNFG2_TWD_SYS_16			0x00
#define MAX77660_GLBLCNFG2_TWD_SYS_32			0x10
#define MAX77660_GLBLCNFG2_TWD_SYS_64			0x20
#define MAX77660_GLBLCNFG2_TWD_SYS_128			0x30
#define MAX77660_GLBLCNFG2_TWD_SYS(n)			(((n) & 3) << 4)
#define MAX77660_GLBLCNFG2_RTCWKEN			BIT(3)
#define MAX77660_GLBLCNFG2_WDTWKEN			BIT(2)
#define MAX77660_GLBLCNFG2_MRTOWKEN			BIT(1)

/* GLBLCNFG4: Global Configuration Register 4 */
#define MAX77660_GLBLCNFG4_WDTC_SYS_MASK		0x3
#define MAX77660_GLBLCNFG4_WDTC_SYS_CLR			0x1

#define GLBLCNFG5_EN1_FPS6_MASK_MASK	BIT(4)
#define GLBLCNFG5_EN1_FPS6_MASK_SHIFT	4
#define GLBLCNFG5_EN5_MASK_MASK			BIT(3)
#define GLBLCNFG5_EN5_MASK_SHIFT		3
#define GLBLCNFG5_EN1_MASK_MASK		BIT(2)
#define GLBLCNFG5_EN1_MASK_SHIFT		2
#define GLBLCNFG5_TRSTO_MASK			(BIT(0) | BIT(1))
#define GLBLCNFG5_TRSTO_SHIFT			0


#define GLBLCNFG7_EN4_MASK_MASK		BIT(2)
#define GLBLCNFG7_EN4_MASK_SHIFT	2
#define GLBLCNFG7_EN3_MASK_MASK		BIT(1)
#define GLBLCNFG7_EN3_MASK_SHIFT	1
#define GLBLCNFG7_EN2_MASK_MASK		BIT(0)
#define GLBLCNFG7_EN2_MASK_SHIFT	0



#define PWR_MODE_32KCLK_MASK		(BIT(1) | BIT(0))
#define OUT1_EN_32KCLK_MASK		BIT(2)
#define OUT1_EN_32KCLK_SHIFT		2
#define OUT2_EN_32KCLK_MASK		BIT(3)
#define OUT2_EN_32KCLK_SHIFT		3

#define CID_DIDM_MASK           (BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define CID_DIDM_SHIFT          4
#define CID_DIDO_MASK           (BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define CID_DIDO_SHIFT          0


#define IRQ_GPIO_BASE			MAX77660_IRQ_GPIO0
#define IRQ_GPIO_END			MAX77660_IRQ_GPIO9

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


#define MAX77660_BUCK2_PWR_MODE_MASK	(BIT(2) | BIT(3))

#if 0
#define ONOFF_SFT_RST_MASK		(1 << 7)
#define ONOFF_SLPEN_MASK		(1 << 2)
#define ONOFF_PWR_OFF_MASK		(1 << 1)

#define ONOFF_SLP_LPM_MASK		(1 << 5)

#define ONOFF_IRQ_EN0_RISING		(1 << 3)
#endif

enum {
	CACHE_IRQ_GLBLINT1,
	CACHE_IRQ_GLBLINT2,
	CACHE_IRQ_BUCK,
	CACHE_IRQ_LDO,
	CACHE_IRQ_NR,
};

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

#if 0
	MAX77660_IRQ_LBT_LB,		/* Low-Battery */
	MAX77660_IRQ_LBT_THERM_ALRM1,	/* Thermal alarm status, > 120C */
	MAX77660_IRQ_LBT_THERM_ALRM2,	/* Thermal alarm status, > 140C */
#endif
	MAX77660_IRQ_GLBL_BASE,
	MAX77660_IRQ_GLBL_TJALRM2 = MAX77660_IRQ_GLBL_BASE,	/* Thermal alarm status, > 140C */
	MAX77660_IRQ_GLBL_TJALRM1,	/* Thermal alarm status, > 120C */
	MAX77660_IRQ_GLBL_SYSLOW,    /* Low main battery interrupt */
	MAX77660_IRQ_GLBL_I2C_WDT,   /* I2C watchdog timeout interrupt */
	MAX77660_IRQ_GLBL_EN0_1SEC,  /* EN0 Active for 1 sec interrupt */
	MAX77660_IRQ_GLBL_EN0_F,     /* EN0 Falling interrupt */
	MAX77660_IRQ_GLBL_EN0_R,     /* EN0 Rising interrupt */
	MAX77660_IRQ_GLBL_WDTWRN_CHG,	/* Charger watchdog timer warning interrupt */
	MAX77660_IRQ_GLBL_WDTWRN_SYS,	/* System watchdog timer warning interrupt */
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

#if 0
	MAX77660_IRQ_ONOFF_HRDPOWRN,	/* Hard power off warnning */
	MAX77660_IRQ_ONOFF_EN0_1SEC,	/* EN0 active for 1s */
	MAX77660_IRQ_ONOFF_EN0_FALLING,	/* EN0 falling */
	MAX77660_IRQ_ONOFF_EN0_RISING,	/* EN0 rising */
	MAX77660_IRQ_ONOFF_LID_FALLING,	/* LID falling */
	MAX77660_IRQ_ONOFF_LID_RISING,	/* LID rising */
	MAX77660_IRQ_ONOFF_ACOK_FALLING,/* ACOK falling */
	MAX77660_IRQ_ONOFF_ACOK_RISING,	/* ACOK rising */
#endif
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

/* Direction */
enum max77660_gpio_dir {
	GPIO_DIR_DEF,
	GPIO_DIR_IN,
	GPIO_DIR_OUT,
};

/* Data output */
enum max77660_gpio_data_out {
	GPIO_DOUT_DEF,
	GPIO_DOUT_HIGH,
	GPIO_DOUT_LOW,
};

/* Output drive */
enum max77660_gpio_out_drv {
	GPIO_OUT_DRV_DEF,
	GPIO_OUT_DRV_PUSH_PULL,
	GPIO_OUT_DRV_OPEN_DRAIN,
};

/* Pull-up */
enum max77660_gpio_pull_up {
	GPIO_PU_DEF,
	GPIO_PU_ENABLE,
	GPIO_PU_DISABLE,
};

/* Pull-down */
enum max77660_gpio_pull_down {
	GPIO_PD_DEF,
	GPIO_PD_ENABLE,
	GPIO_PD_DISABLE,
};

/* Alternate */
enum max77660_gpio_alt {
	GPIO_ALT_DEF,
	GPIO_ALT_ENABLE,
	GPIO_ALT_DISABLE,
};


/* Max77660 Chip data */
struct max77660_chip {
	struct device *dev;
	struct i2c_client *i2c_power;
	struct i2c_client *i2c_rtc;
	struct i2c_client *i2c_fg;
	struct i2c_client *i2c_chg;
	struct i2c_client *i2c_haptic;

	struct regmap *regmap_power;
	struct regmap *regmap_rtc;
	struct regmap *regmap_chg;	      /* charger */
	struct regmap *regmap_fg;	       /* fuel gauge */
	struct regmap *regmap_haptic;   /* haptic */

	struct i2c_client *clients[MAX77660_NUM_SLAVES];
	struct regmap* rmap[MAX77660_NUM_SLAVES];


	struct max77660_platform_data *pdata;
	struct mutex io_lock;

	struct irq_chip irq;
	int chip_irq;
	struct mutex irq_lock;
	int irq_base;
	int irq_top_count[8];
	u8 cache_irq_top_mask;
	u32 cache_irq_mask[CACHE_IRQ_NR];

	u8 rtc_i2c_addr;
	u8 fg_i2c_addr;
	u8 chg_i2c_addr;
	u8 haptic_i2c_addr;

	struct regmap_irq_chip_data *top_irq_data;
	struct regmap_irq_chip_data *global_irq_data;
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
 * Flags
 */
#define SLP_LPM_ENABLE		0x01

struct max77660_gpio_config {
	int gpio;	/* gpio number */
	enum max77660_gpio_dir dir;
	enum max77660_gpio_data_out dout;
	enum max77660_gpio_out_drv out_drv;
	enum max77660_gpio_pull_up pull_up;
	enum max77660_gpio_pull_down pull_down;
	enum max77660_gpio_alt alternate;
};

/*
 * max77660_platform_data: Platform data for MAX77660.
 * @pinctrl_pdata: Pincontrol configurations.
 * @num_pinctrl: Number of pin control data.
 * @system_watchdog_timeout: System wathdog timeout in seconds. If this value
 *		is -ve then timer will not start during initialisation.
 */

struct max77660_platform_data {
	int irq_base;
	int gpio_base;

	int num_gpio_cfgs;
	struct max77660_gpio_config *gpio_cfgs;

	int num_subdevs;
	struct mfd_cell *sub_devices;

	struct max77660_regulator_platform_data **regulator_pdata;
	int num_regulator_pdata;

	struct max77660_pinctrl_platform_data *pinctrl_pdata;
	int num_pinctrl;

	unsigned int flags;

	unsigned char rtc_i2c_addr;
	unsigned char chg_i2c_addr;
	unsigned char fg_i2c_addr;
	unsigned char haptic_i2c_addr;
	bool en_buck2_ext_ctrl;
	bool en_clk32out1;
	bool en_clk32out2;
	bool use_power_off;

	int system_watchdog_timeout;
};

enum max77660_i2c_slave {
	MAX77660_I2C_CORE = 0,  /* Parent device */
	MAX77660_I2C_GPIO,
	MAX77660_I2C_PMIC,
	MAX77660_I2C_RTC,
	MAX77660_I2C_CHG,
	MAX77660_I2C_FG,
	MAX77660_I2C_HAPTIC,
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

#if defined(CONFIG_MFD_MAX77660)
int max77660_read(struct device *dev, u8 addr, void *values, u32 len,
		  enum max77660_i2c_slave slave);
int max77660_write(struct device *dev, u8 addr, void *values, u32 len,
		  enum max77660_i2c_slave slve);
int max77660_set_bits(struct device *dev, u8 addr, u8 mask, u8 value,
		      enum max77660_i2c_slave slave);
int max77660_gpio_set_alternate(int gpio, int alternate);
#else
static inline int max77660_read(struct device *dev, u8 addr, void *values,
				u32 len, enum max77660_i2c_slave slave)
{
	return 0;
}

static inline int max77660_write(struct device *dev, u8 addr, void *values,
				 u32 len, enum max77660_i2c_slave slave)
{
	return 0;
}

static inline int max77660_set_bits(struct device *dev, u8 addr, u8 mask,
				    u8 value, enum max77660_i2c_slave slave)
{
	return 0;
}

static inline int max77660_gpio_set_alternate(int gpio, int alternate)
{
	return 0;
}
#endif /* defined(CONFIG_MFD_MAX77660) */

#endif /* __LINUX_MFD_MAX77660_CORE_H__ */
