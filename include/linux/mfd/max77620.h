/*
 * Maxim MAX77620
 *
 * Copyright (C) 2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* RTC Registers */
#define MAX77620_REG_RTCINT		0x00
#define MAX77620_REG_RTCINTM		0x01
#define MAX77620_REG_RTCCNTLM		0x02
#define MAX77620_REG_RTCCNTL		0x03
#define MAX77620_REG_RTCUPDATE0		0x04
#define MAX77620_REG_RTCUPDATE1		0x05
#define MAX77620_REG_RTCSMPL		0x06
#define MAX77620_REG_RTCSEC		0x07
#define MAX77620_REG_RTCMIN		0x08
#define MAX77620_REG_RTCHOUR		0x09
#define MAX77620_REG_RTCDOW		0x0A
#define MAX77620_REG_RTCMONTH		0x0B
#define MAX77620_REG_RTCYEAR		0x0C
#define MAX77620_REG_RTCDOM		0x0D
#define MAX77620_REG_RTCSECA1		0x0E
#define MAX77620_REG_RTCMINA1		0x0F
#define MAX77620_REG_RTCHOURA1		0x10
#define MAX77620_REG_RTCDOWA1		0x11
#define MAX77620_REG_RTCMONTHA1		0x12
#define MAX77620_REG_RTCYEARA1		0x13
#define MAX77620_REG_RTCDOMA1		0x14
#define MAX77620_REG_RTCSECA2		0x15
#define MAX77620_REG_RTCMINA2		0x16
#define MAX77620_REG_RTCHOURA2		0x17
#define MAX77620_REG_RTCDOWA2		0x18
#define MAX77620_REG_RTCMONTHA2		0x19
#define MAX77620_REG_RTCYEARA2		0x1A
#define MAX77620_REG_RTCDOMA2		0x1B


/* GLOBAL, PMIC, GPIO, FPS, ONOFFC, CID Registers */
#define MAX77620_REG_CNFGGLBL1		0x00
#define MAX77620_REG_CNFGGLBL2		0x01
#define MAX77620_REG_CNFGGLBL3		0x02
#define MAX77620_REG_CNFG1_32K		0x03
#define MAX77620_REG_CNFGBBC		0x04
#define MAX77620_REG_IRQTOP		0x05
#define MAX77620_REG_INTLBT		0x06
#define MAX77620_REG_IRQSD		0x07
#define MAX77620_REG_IRQ_LVL2_L0_7	0x08
#define MAX77620_REG_IRQ_LVL2_L8	0x09
#define MAX77620_REG_IRQ_LVL2_GPIO	0x0A
#define MAX77620_REG_ONOFFIRQ		0x0B
#define MAX77620_REG_NVERC		0x0C
#define MAX77620_REG_IRQTOPM		0x0D
#define MAX77620_REG_INTENLBT		0x0E
#define MAX77620_REG_IRQMASKSD		0x0F
#define MAX77620_REG_IRQ_MSK_L0_7	0x10
#define MAX77620_REG_IRQ_MSK_L8		0x11
#define MAX77620_REG_ONOFFIRQM		0x12
#define MAX77620_REG_STATLBT		0x13
#define MAX77620_REG_STATSD		0x14
#define MAX77620_REG_ONOFFSTAT		0x15
#define MAX77620_REG_VSD0		0x16
#define MAX77620_REG_VSD1		0x17
#define MAX77620_REG_VSD2		0x18
#define MAX77620_REG_VSD3		0x19
#define MAX77620_REG_VSD4		0x1A
#define MAX77620_REG_VDVSSD0		0x1B
#define MAX77620_REG_VDVSSD1		0x1C
#define MAX77620_REG_VDVSSD4		0x5E
#define MAX77620_REG_CNFG1SD0		0x1D
#define MAX77620_REG_CNFG1SD1		0x1E
#define MAX77620_REG_CNFG1SD2		0x1F
#define MAX77620_REG_CNFG1SD3		0x20
#define MAX77620_REG_CNFG1SD4		0x21
#define MAX77620_REG_CNFG2SD		0x22
#define MAX77620_REG_CNFG1_L0		0x23
#define MAX77620_REG_CNFG2_L0		0x24
#define MAX77620_REG_CNFG1_L1		0x25
#define MAX77620_REG_CNFG2_L1		0x26
#define MAX77620_REG_CNFG1_L2		0x27
#define MAX77620_REG_CNFG2_L2		0x28
#define MAX77620_REG_CNFG1_L3		0x29
#define MAX77620_REG_CNFG2_L3		0x2A
#define MAX77620_REG_CNFG1_L4		0x2B
#define MAX77620_REG_CNFG2_L4		0x2C
#define MAX77620_REG_CNFG1_L5		0x2D
#define MAX77620_REG_CNFG2_L5		0x2E
#define MAX77620_REG_CNFG1_L6		0x2F
#define MAX77620_REG_CNFG2_L6		0x30
#define MAX77620_REG_CNFG1_L7		0x31
#define MAX77620_REG_CNFG2_L7		0x32
#define MAX77620_REG_CNFG1_L8		0x33
#define MAX77620_REG_CNFG2_L8		0x34
#define MAX77620_REG_CNFG3_LDO		0x35
#define MAX77620_REG_GPIO0		0x36
#define MAX77620_REG_GPIO1		0x37
#define MAX77620_REG_GPIO2		0x38
#define MAX77620_REG_GPIO3		0x39
#define MAX77620_REG_GPIO4		0x3A
#define MAX77620_REG_GPIO5		0x3B
#define MAX77620_REG_GPIO6		0x3C
#define MAX77620_REG_GPIO7		0x3D
#define MAX77620_REG_PUE_GPIO		0x3E
#define MAX77620_REG_PDE_GPIO		0x3F
#define MAX77620_REG_AME_GPIO		0x40
#define MAX77620_REG_ONOFFCNFG1		0x41
#define MAX77620_REG_ONOFFCNFG2		0x42
#define MAX77620_REG_CNFGFPS0		0x43
#define MAX77620_REG_CNFGFPS1		0x44
#define MAX77620_REG_CNFGFPS2		0x45
#define MAX77620_REG_FPS_L0		0x46
#define MAX77620_REG_FPS_L1		0x47
#define MAX77620_REG_FPS_L2		0x48
#define MAX77620_REG_FPS_L3		0x49
#define MAX77620_REG_FPS_L4		0x4A
#define MAX77620_REG_FPS_L5		0x4B
#define MAX77620_REG_FPS_L6		0x4C
#define MAX77620_REG_FPS_L7		0x4D
#define MAX77620_REG_FPS_L8		0x4E
#define MAX77620_REG_FPS_SD0		0x4F
#define MAX77620_REG_FPS_SD1		0x50
#define MAX77620_REG_FPS_SD2		0x51
#define MAX77620_REG_FPS_SD3		0x52
#define MAX77620_REG_FPS_SD4		0x53
#define MAX77620_REG_FPS_GPIO1		0x54
#define MAX77620_REG_FPS_GPIO2		0x55
#define MAX77620_REG_FPS_GPIO3		0x56
#define MAX77620_REG_FPS_RSO		0x57
#define MAX77620_REG_CID0		0x58
#define MAX77620_REG_CID1		0x59
#define MAX77620_REG_CID2		0x5A
#define MAX77620_REG_CID3		0x5B
#define MAX77620_REG_CID4		0x5C
#define MAX77620_REG_CID5		0x5D

#define MAX77620_IRQ_TOP_GLBL_MASK	BIT(7)
#define MAX77620_IRQ_TOP_SD_MASK	BIT(6)
#define MAX77620_IRQ_TOP_LDO_MASK	BIT(5)
#define MAX77620_IRQ_TOP_GPIO_MASK	BIT(4)
#define MAX77620_IRQ_TOP_RTC_MASK	BIT(3)
#define MAX77620_IRQ_TOP_32K_MASK	BIT(2)
#define MAX77620_IRQ_TOP_ONOFF_MASK	BIT(1)

#define MAX77620_PWR_I2C_ADDR		0x3c
#define MAX77620_RTC_I2C_ADDR		0x68

#define MAX77620_CNFG_GPIO_DRV_MASK		BIT(0)
#define MAX77620_CNFG_GPIO_DRV_PUSHPULL		BIT(0)
#define MAX77620_CNFG_GPIO_DRV_OPENDRAIN	0
#define MAX77620_CNFG_GPIO_DIR_MASK		BIT(1)
#define MAX77620_CNFG_GPIO_DIR_INPUT		BIT(1)
#define MAX77620_CNFG_GPIO_DIR_OUTPUT		0
#define MAX77620_CNFG_GPIO_INPUT_VAL_MASK	BIT(2)
#define MAX77620_CNFG_GPIO_OUTPUT_VAL_MASK	BIT(3)
#define MAX77620_CNFG_GPIO_OUTPUT_VAL_HIGH	BIT(3)
#define MAX77620_CNFG_GPIO_OUTPUT_VAL_LOW	0
#define MAX77620_CNFG_GPIO_INT_MASK		(0x3 << 4)
#define MAX77620_CNFG_GPIO_INT_FALLING		BIT(4)
#define MAX77620_CNFG_GPIO_INT_RISING		BIT(5)
#define MAX77620_CNFG_GPIO_DBNC_MASK		(0x3 << 6)
#define MAX77620_CNFG_GPIO_DBNC_None		(0x0 << 6)
#define MAX77620_CNFG_GPIO_DBNC_8ms		(0x1 << 6)
#define MAX77620_CNFG_GPIO_DBNC_16ms		(0x2 << 6)
#define MAX77620_CNFG_GPIO_DBNC_32ms		(0x3 << 6)

#define MAX77620_IRQ_LVL2_GPIO_EDGE0		BIT(0)
#define MAX77620_IRQ_LVL2_GPIO_EDGE1		BIT(1)
#define MAX77620_IRQ_LVL2_GPIO_EDGE2		BIT(2)
#define MAX77620_IRQ_LVL2_GPIO_EDGE3		BIT(3)
#define MAX77620_IRQ_LVL2_GPIO_EDGE4		BIT(4)
#define MAX77620_IRQ_LVL2_GPIO_EDGE5		BIT(5)
#define MAX77620_IRQ_LVL2_GPIO_EDGE6		BIT(6)
#define MAX77620_IRQ_LVL2_GPIO_EDGE7		BIT(7)


/* I2c Slave Id */
enum {
	MAX77620_PWR_SLAVE,
	MAX77620_RTC_SLAVE,

	MAX77620_NUM_SLAVES,
};

/*
 *GPIOs
 */
enum {
	MAX77620_GPIO0,
	MAX77620_GPIO1,
	MAX77620_GPIO2,
	MAX77620_GPIO3,
	MAX77620_GPIO4,
	MAX77620_GPIO5,
	MAX77620_GPIO6,
	MAX77620_GPIO7,

	MAX77620_GPIO_NR,
};

/*
 * Interrupts
 *
 */

enum {
	MAX77620_IRQ_TOP_GLBL,		/* Low-Battery */
	MAX77620_IRQ_TOP_SD,		/* SD power fail */
	MAX77620_IRQ_TOP_LDO,		/* LDO power fail */
	MAX77620_IRQ_TOP_GPIO,		/* TOP GPIO internal int to MAX77620 */
	MAX77620_IRQ_TOP_RTC,		/* RTC */
	MAX77620_IRQ_TOP_32K,		/* 32kHz oscillator */
	MAX77620_IRQ_TOP_ONOFF,		/* ON/OFF oscillator */

	MAX77620_IRQ_LBT_MBATLOW,	/* Thermal alarm status, > 120C */
	MAX77620_IRQ_LBT_TJALRM1,	/* Thermal alarm status, > 120C */
	MAX77620_IRQ_LBT_TJALRM2,	/* Thermal alarm status, > 140C */

	MAX77620_IRQ_GPIO0,		/* GPIO0 edge detection */
	MAX77620_IRQ_GPIO1,		/* GPIO1 edge detection */
	MAX77620_IRQ_GPIO2,		/* GPIO2 edge detection */
	MAX77620_IRQ_GPIO3,		/* GPIO3 edge detection */
	MAX77620_IRQ_GPIO4,		/* GPIO4 edge detection */
	MAX77620_IRQ_GPIO5,		/* GPIO5 edge detection */
	MAX77620_IRQ_GPIO6,		/* GPIO6 edge detection */
	MAX77620_IRQ_GPIO7,		/* GPIO7 edge detection */

	MAX77620_IRQ_ONOFF_MRWRN,	/* Hard power off warnning */
	MAX77620_IRQ_ONOFF_EN0_1SEC,	/* EN0 active for 1s */
	MAX77620_IRQ_ONOFF_EN0_F,	/* EN0 falling */
	MAX77620_IRQ_ONOFF_EN0_R,	/* EN0 rising */
	MAX77620_IRQ_ONOFF_LID_F,	/* LID falling */
	MAX77620_IRQ_ONOFF_LID_R,	/* LID rising */
	MAX77620_IRQ_ONOFF_ACOK_F,	/* ACOK falling */
	MAX77620_IRQ_ONOFF_ACOK_R,	/* ACOK rising */

	MAX77620_IRQ_NVER,		/* Non-Volatile Event Recorder */
	MAX77620_IRQ_NR,
};

struct max77620_chip {
	struct device *dev;

	struct i2c_client *clients[MAX77620_NUM_SLAVES];
	struct regmap *rmap[MAX77620_NUM_SLAVES];

	int chip_irq;
	int irq_base;

	struct regmap_irq_chip_data *top_irq_data;
	struct regmap_irq_chip_data *gpio_irq_data;
};

static inline int max77620_reg_write(struct device *dev, int sid,
		unsigned int reg, u8 val)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);

	return regmap_write(chip->rmap[sid], reg, val);
}

static inline int max77620_reg_writes(struct device *dev, int sid,
		unsigned int reg, int len, void *val)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);

	return regmap_bulk_write(chip->rmap[sid], reg, val, len);
}

static inline int max77620_reg_read(struct device *dev, int sid,
		unsigned int reg, u8 *val)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);
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

static inline int max77620_reg_reads(struct device *dev, int sid,
		unsigned int reg, int len, void *val)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);

	return regmap_bulk_read(chip->rmap[sid], reg, val, len);
}

static inline int max77620_reg_update(struct device *dev, int sid,
		unsigned int reg, unsigned int mask, unsigned int val)
{
	struct max77620_chip *chip = dev_get_drvdata(dev);

	return regmap_update_bits(chip->rmap[sid], reg, mask, val);
}
