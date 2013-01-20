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

#define MAX77660_REG_CID0				0x9A
#define MAX77660_REG_CID1				0x9B
#define MAX77660_REG_CID2				0x9C
#define MAX77660_REG_CID3				0x9D
#define MAX77660_REG_CID4				0x9E
#define MAX77660_REG_CID5				0x9F

#endif


#define IRQ_TOP1_TOPSYS_MASK	BIT(7)
#define IRQ_TOP1_TOPSYS_SHIFT	7
#define IRQ_TOP1_ADC_MASK		BIT(6)
#define IRQ_TOP1_ADC_SHIFT		6
#define IRQ_TOP1_SIM_MASK		BIT(5)
#define IRQ_TOP1_SIM_SHIFT		5
#define IRQ_TOP1_GPIO_MASK		BIT(4)
#define IRQ_TOP1_GPIO_SHIFT		4
#define IRQ_TOP1_RTC_MASK		BIT(3)
#define IRQ_TOP1_RTC_SHIFT		3
#define IRQ_TOP1_CHARGER_MASK	BIT(2)
#define IRQ_TOP1_CHARGER_SHIFT	2
#define IRQ_TOP1_FUELG_MASK		BIT(1)
#define IRQ_TOP1_FUELG_SHIFT	1
#define IRQ_TOP1_OVF_MASK		BIT(0)
#define IRQ_TOP1_OVF_SHIFT		0

#define IRQ_TOP2_BUCK_MASK		BIT(1)
#define IRQ_TOP2_BUCK_SHIFT		1
#define IRQ_TOP2_LDO_MASK		BIT(0)
#define IRQ_TOP2_LDO_SHIFT		0

#define IRQ_GLBLINT1_EN0_R_MASK		BIT(7)
#define IRQ_GLBLINT1_EN0_R_SHIFT		7
#define IRQ_GLBLINT1_EN0_F_MASK		BIT(6)
#define IRQ_GLBLINT1_EN0_F_SHIFT		6
#define IRQ_GLBLINT1_EN0_1SEC_MASK	BIT(5)
#define IRQ_GLBLINT1_EN0_1SEC_SHIFT		5
#define IRQ_GLBLINT1_I2CWDT_MASK	BIT(4)
#define IRQ_GLBLINT1_I2CWDT_SHIFT		4
#define IRQ_GLBLINT1_SYSLOW_MASK	BIT(3)
#define IRQ_GLBLINT1_SYSLOW_SHIFT		3
#define IRQ_GLBLINT1_TJALRM1_MASK	BIT(2)
#define IRQ_GLBLINT1_TJALRM1_SHIFT		2
#define IRQ_GLBLINT1_TJALRM2_MASK	BIT(1)
#define IRQ_GLBLINT1_TJALRM2_SHIFT		1

#define IRQ_GLBLINT2_MR_R_MASK			BIT(3)
#define IRQ_GLBLINT2_MR_R_SHIFT				3
#define IRQ_GLBLINT2_MR_F_MASK			BIT(2)
#define IRQ_GLBLINT2_MR_F_SHIFT				2
#define IRQ_GLBLINT2_WDTWRN_SYS_MASK	BIT(1)
#define IRQ_GLBLINT2_WDTWRN_SYS_SHIFT		1
#define IRQ_GLBLINT2_WDTWRN_CHG_MASK	BIT(0)
#define IRQ_GLBLINT2_WDTWRN_CHG_SHIFT		0

#define GLBLCNFG0_SFT_OFF_SYSRST_MASK		BIT(3)
#define GLBLCNFG0_SFT_OFF_SYSRST_SHIFT			3
#define GLBLCNFG0_SFT_OFF_OFFRST_MASK		BIT(2)
#define GLBLCNFG0_SFT_OFF_OFFRST_SHIFT			2
#define GLBLCNFG0_SFT_WRST_MASK				BIT(1)
#define GLBLCNFG0_SFT_WRST_SHIFT				1
#define GLBLCNFG0_SFT_CRST_MASK				BIT(0)
#define GLBLCNFG0_SFT_CRST_SHIFT				0

#define GLBLCNFG1_DISCHGTL_MASK			BIT(7)
#define GLBLCNFG1_DISCHGTL_SHIFT			7
#define GLBLCNFG1_GLBL_LPM_MASK			BIT(6)
#define GLBLCNFG1_GLBL_LPM_SHIFT			6
#define GLBLCNFG1_WDTEN_SYS_MASK		BIT(5)
#define GLBLCNFG1_WDTEN_SYS_SHIFT			5
#define GLBLCNFG1_MRT_MASK				(BIT(3) | BIT(4))
#define GLBLCNFG1_MRT_SHIFT					3
#define GLBLCNFG1_SHDN_WRST_MASK		BIT(2)
#define GLBLCNFG1_SHDN_WRST_SHIFT			2
#define GLBLCNFG1_EN_CHGTL_MASK			BIT(1)
#define GLBLCNFG1_EN_CHGTL_SHIFT			1
#define GLBLCNFG1_EN_PGOC_MASK			BIT(0)
#define GLBLCNFG1_EN_PGOC_SHIFT				0

#define GLBLCNFG2_RTC_WKEN_MASK				BIT(3)
#define GLBLCNFG2_RTC_WKEN_SHIFT			3
#define GLBLCNFG2_WDT_WKEN_MASK				BIT(2)
#define GLBLCNFG2_WDT_WKEN_SHIFT			2
#define GLBLCNFG2_MRTO_WKEN_MASK			BIT(1)
#define GLBLCNFG2_MRTO_WKEN_SHIFT			1

#define GLBLCNFG5_EN1_FPS6_MASK_MASK	BIT(4)
#define GLBLCNFG5_EN1_FPS6_MASK_SHIFT	4
#define GLBLCNFG5_EN5_MASK_MASK			BIT(3)
#define GLBLCNFG5_EN5_MASK_SHIFT		3
#define GLBLCNFG5_EN1234_MASK_MASK		BIT(2)
#define GLBLCNFG5_EN1234_MASK_SHIFT		2
#define GLBLCNFG5_TRSTO_MASK			(BIT(0) | BIT(1))
#define GLBLCNFG5_TRSTO_SHIFT			0

#define CID_DIDM_MASK           (BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define CID_DIDM_SHIFT          4
#define CID_DIDO_MASK           (BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define CID_DIDO_SHIFT          0


#define IRQ_GPIO_BASE			MAX77660_IRQ_GPIO0
#define IRQ_GPIO_END			MAX77660_IRQ_GPIO9



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
	MAX77660_IRQ_INT_TOP_GPIO,	/* TOP GPIO internal int to max77660 */
	MAX77660_IRQ_INT_TOP_OVF,	/* If this bit is set read from TOP2  */
	MAX77660_IRQ_INT_TOP_SYSINT,/* If this bit is set read from GLOBAL INT1 and INT2 */
#if 0
	MAX77660_IRQ_LBT_LB,		/* Low-Battery */
	MAX77660_IRQ_LBT_THERM_ALRM1,	/* Thermal alarm status, > 120C */
	MAX77660_IRQ_LBT_THERM_ALRM2,	/* Thermal alarm status, > 140C */
#endif
	MAX77660_IRQ_GLBL_EN0_R,     /* EN0 Rising interrupt */
	MAX77660_IRQ_GLBL_EN0_F,     /* EN0 Falling interrupt */
	MAX77660_IRQ_GLBL_EN0_1SEC,  /* EN0 Active for 1 sec interrupt */
	MAX77660_IRQ_GLBL_I2C_WDT,   /* I2C watchdog timeout interrupt */
	MAX77660_IRQ_GLBL_SYSLOW,    /* Low main battery interrupt */
	MAX77660_IRQ_GLBL_TJALRM1,	/* Thermal alarm status, > 120C */
	MAX77660_IRQ_GLBL_TJALRM2,	/* Thermal alarm status, > 140C */
	MAX77660_IRQ_GLBL_MR_R,		/* Manual reset rising interrupt */
	MAX77660_IRQ_GLBL_MR_F,		/* Manual reset falling interrupt */
	MAX77660_IRQ_GLBL_WDTWRN_SYS,	/* System watchdog timer warning interrupt */
	MAX77660_IRQ_GLBL_WDTWRN_CHG,	/* Charger watchdog timer warning interrupt */

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
	MAX77660_IRQ_FG,		/* FG */
	MAX77660_IRQ_CHG,		/* CHG */
	MAX77660_IRQ_RTC,		/* RTC */
	MAX77660_IRQ_HAPTIC,		/* HAPTIC */
	MAX77660_IRQ_BUCK_PF,		/* BUCK power fail */
	MAX77660_IRQ_LDO_PF,		/* LDO power fail */
	MAX77660_IRQ_SIM,		/* SIM interrupt */
	MAX77660_IRQ_ADC,		/* ADC interrupt */

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

	unsigned int flags;

	unsigned char rtc_i2c_addr;
	unsigned char chg_i2c_addr;
	unsigned char fg_i2c_addr;
	unsigned char haptic_i2c_addr;

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
