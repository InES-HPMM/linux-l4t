/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
 *  @addtogroup DRIVERS
 *  @brief      Hardware drivers.
 *
 *  @{
 *      @file  inv_gyro.h
 *      @brief Struct definitions for the Invensense gyro driver.
 */

#ifndef _INV_GYRO_H_
#define _INV_GYRO_H_

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include "dmpKey.h"

/**
 *  struct inv_reg_map_s - Notable slave registers.
 *  @who_am_i:		Upper 6 bits of the device's slave address.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @product_id:	Product revision.
 *  @bank_sel:		Selects between memory banks.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro		Address of first gyro register.
 *  @raw_accl		Address of first accel register.
 *  @temperature	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 *  @mem_start_addr:	Address of first memory read.
 *  @mem_r_w:		Access to memory.
 *  @prgm_strt_addrh	firmware program start address register
 */
struct inv_reg_map_s {
	unsigned char who_am_i;
	unsigned char sample_rate_div;
	unsigned char lpf;
	unsigned char product_id;
	unsigned char bank_sel;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char gyro_config;
	unsigned char accl_config;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accl;
	unsigned char temperature;
	unsigned char int_enable;
	unsigned char int_status;
	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;
	unsigned char mem_start_addr;
	unsigned char mem_r_w;
	unsigned char prgm_strt_addrh;
};

enum inv_devices {
	INV_ITG3500 = 0,
	INV_MPU3050 = 1,
	INV_MPU6050 = 2,
	INV_MPU9150 = 3,
	INV_NUM_PARTS
};

/**
 *  struct test_setup_t - set up parameters for self test.
 *  @sample_rate: sensitity for gyro.
 *  @sample_rate: sample rate, i.e, fifo rate.
 *  @lpf:	low pass filter.
 *  @fsr:	full scale range.
 *  @accl_fs:	accel full scale range.
 *  @accl_sens:	accel sensitivity
 */
struct test_setup_t {
	int gyro_sens;
	int sample_rate;
	int lpf;
	int fsr;
	int accl_fs;
	unsigned int accl_sens[3];
};

/**
 *  struct inv_hw_s - Other important hardware information.
 *  @num_reg:	Number of registers on device.
 *  @name:      name of the chip
 */
struct inv_hw_s {
	unsigned char num_reg;
	unsigned char *name;
};

/**
 *  struct inv_chip_config_s - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @clk_src:		Clock source.
 *  @accl_fs:		accel full scale range.
 *  @fifo_rate:		FIFO update rate.
 *  @enable:		master enable to enable output
 *  @accl_enable:	enable accel functionality
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_enable:	enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @compass_enable:	enable compass
 *  @is_asleep:		1 if chip is powered down.
 *  @dmp_on:		dmp is on/off
 *  @firmware_loaded:	flag indicate firmware loaded or not.
 *  @lpa_mod:		low power mode
 *  @lpa_freq:          low power accel frequency.
 *  @prog_start_addr:	firmware program start address
 */
struct inv_chip_config_s {
	unsigned char fsr;
	unsigned char lpf;
	unsigned char clk_src;
	unsigned char accl_fs;
	unsigned short fifo_rate;
	unsigned char enable;
	unsigned char accl_enable;
	unsigned char accl_fifo_enable;
	unsigned char gyro_enable;
	unsigned char gyro_fifo_enable;
	unsigned char compass_enable;
	unsigned char is_asleep;
	unsigned char dmp_on;
	unsigned char firmware_loaded;
	unsigned char lpa_mode;
	unsigned char lpa_freq;
	unsigned int  prog_start_addr;
};

/**
 *  struct inv_chip_info_s - Chip related information.
 *  @product_id:	Product id.
 *  @product_revision:	Product revision.
 *  @silicon_revision:	Silicon revision.
 *  @software_revision:	software revision.
 *  @multi:		accel specific multiplier.
 *  @compass_sens:	compass sensitivity.
 *  @gyro_sens_trim:	Gyro sensitivity trim factor.
 *  @accl_sens_trim:    accel sensitivity trim factor.
 */
struct inv_chip_info_s {
	unsigned char product_id;
	unsigned char product_revision;
	unsigned char silicon_revision;
	unsigned char software_revision;
	unsigned char multi;
	unsigned char compass_sens[3];
	unsigned long gyro_sens_trim;
	unsigned long accl_sens_trim;
};

/**
 *  struct inv_trigger_s - Variables passed between interrupt and kernel space.
 *  @irq:		Interrupt number.
 *  @timestamps:	Timestamp buffer.
 */
struct inv_trigger_s {
#define TIMESTAMP_FIFO_SIZE 16
	unsigned long irq;
	DECLARE_KFIFO(timestamps, long long, TIMESTAMP_FIFO_SIZE);
};

/**
 *  struct inv_flick_s structure to store flick data.
 *  @lower:	lower bound of flick.
 *  @upper:     upper bound of flick.
 *  @counter:	counterof flick.
 *  @int_on:	interrupt on for flick.
 *  @msg_on;    message to carry flick
 *  @axis:      axis of flick
 */
struct inv_flick_s {
	int lower;
	int upper;
	int counter;
	char int_on;
	char msg_on;
	char axis;
};

/**
 *  struct inv_tap_s structure to store tap data.
 *  @tap_on:	tap on
 *  @min_taps:  minimut taps counted.
 *  @thresh:    tap threshold.
 *  @time:	tap time.
 */
struct inv_tap_s {
	char tap_on;
	char min_tap;
	short thresh;
	short time;
};

/**
 * struct inv_regulator_s structure to store regulator
 */
struct inv_regulator_s {
	struct regulator *regulator_vlogic;
	struct regulator *regulator_vdd;
};

struct inv_mpu_slave;
/**
 *  struct inv_gyro_state_s - Driver state variables.
 *  @chip_config:	Cached attribute information.
 *  @chip_info:		Chip information from read-only registers.
 *  @flick:		flick data structure
 *  @reg:		Map of important registers.
 *  @hw:		Other hardware-specific information.
 *  @idev:		Handle to input device.
 *  @idev_dmp:		Handle to input device for DMP.
 *  @idev_compass:	Handle to input device for compass.
 *  @chip_type:		chip type.
 *  @time_stamp_lock:	spin lock to time stamp.
 *  @inv_class:		store class handle.
 *  @inv_dev:		store device handle.
 *  @i2c:		i2c client handle.
 *  @plat_data:		platform data.
 *  @mpu_slave:		mpu slave handle.
 *  @fifo_counter:	MPU3050 specific work around.
 *  @has_compass:	has compass or not.
 *  @compass_scale:	compass scale.
 *  @i2c_addr:		i2c address.
 *  @compass_divider:	slow down compass rate.
 *  @compass_counter:	slow down compass rate.
 *  @sample_divider:    sample divider for dmp.
 *  @fifo_divider:      fifo divider for dmp.
 *  @sl_handle:		Handle to I2C port.
 *  @irq_dur_us:	duration between each irq.
 *  @last_isr_time:	last isr time.
 *  @early_suspend:     struct for early suspend.
 *  @early_suspend_enable: sysfs interface to store current early_suspend.
 *  @inv_regulator_s:	Regulator sturcture to store regulator.
 */
struct inv_gyro_state_s {
	struct inv_chip_config_s chip_config;
	struct inv_chip_info_s chip_info;
	struct inv_flick_s flick;
	struct inv_tap_s   tap;
	struct inv_reg_map_s *reg;
	struct inv_hw_s *hw;
	struct inv_trigger_s trigger;
	struct input_dev *idev;
	struct input_dev *idev_dmp;
	struct input_dev *idev_compass;
	enum   inv_devices chip_type;
	spinlock_t time_stamp_lock;
	struct class *inv_class;
	struct device   *inv_dev;
	struct i2c_client *i2c;
	struct mpu_platform_data plat_data;
	struct inv_mpu_slave *mpu_slave;
	short compass_st_upper[3];
	short compass_st_lower[3];
	unsigned char fifo_counter;
	unsigned char has_compass;
	unsigned char compass_scale;
	unsigned char i2c_addr;
	unsigned char compass_divider;
	unsigned char compass_counter;
	unsigned char sample_divider;
	unsigned char fifo_divider;
	void *sl_handle;
	unsigned int irq_dur_us;
	long long last_isr_time;
	struct inv_regulator_s inv_regulator;
	bool i2c_shutdown;
};

/* produces an unique identifier for each device based on the
   combination of product version and product revision */
struct prod_rev_map_t {
	unsigned short mpl_product_key;
	unsigned char silicon_rev;
	unsigned short gyro_trim;
	unsigned short accel_trim;
};
/**
 *  struct inv_mpu_slave - MPU slave structure.
 *  @suspend:		suspend operation.
 *  @resume:		resume operation.
 *  @setup:		setup chip. initialization.
 *  @create_sysfs:	create chip specific sysfs entries.
 *  @remove_sysfs:	remove chip specific sysfs entries.
 *  @combine_data:	combine raw data into meaningful data.
 *  @get_mode:		get current chip mode.
 */
struct inv_mpu_slave {
	int (*suspend)(struct inv_gyro_state_s *);
	int (*resume)(struct inv_gyro_state_s *);
	int (*setup)(struct inv_gyro_state_s *);
	int (*combine_data)(unsigned char *in, short *out);
	int (*get_mode)(struct inv_gyro_state_s *);
	int (*set_lpf)(struct inv_gyro_state_s *, int rate);
	int (*set_fs)(struct inv_gyro_state_s *, int fs);
};
/* AKM definitions */
#define REG_AKM_ID              (0x00)
#define REG_AKM_STATUS          (0x02)
#define REG_AKM_MEASURE_DATA    (0x03)
#define REG_AKM_MODE            (0x0A)
#define REG_AKM_ST_CTRL         (0x0C)
#define REG_AKM_SENSITIVITY     (0x10)
#define REG_AKM8963_CNTL1       (0x0A)

#define DATA_AKM_ID             (0x48)
#define DATA_AKM_MODE_PW_DN     (0x00)
#define DATA_AKM_MODE_PW_SM     (0x01)
#define DATA_AKM_MODE_PW_ST     (0x08)
#define DATA_AKM_MODE_PW_FR     (0x0F)
#define DATA_AKM_SELF_TEST      (0x40)
#define DATA_AKM_DRDY           (0x01)
#define DATA_AKM8963_BIT        (0x10)
#define DATA_AKM_STAT_MASK      (0x0C)

#define DATA_AKM8975_SCALE      (9830)
#define DATA_AKM8972_SCALE      (19661)
#define DATA_AKM8963_SCALE0     (19661)
#define DATA_AKM8963_SCALE1     (4915)

#define DATA_AKM8975_ST_X_UP    (100)
#define DATA_AKM8975_ST_X_LW    (-100)
#define DATA_AKM8975_ST_Y_UP    (100)
#define DATA_AKM8975_ST_Y_LW    (-100)
#define DATA_AKM8975_ST_Z_UP    (-300)
#define DATA_AKM8975_ST_Z_LW    (-1000)

/*need to verify the range for 8972 */
#define DATA_AKM8972_ST_X_UP    (50)
#define DATA_AKM8972_ST_X_LW    (-50)
#define DATA_AKM8972_ST_Y_UP    (50)
#define DATA_AKM8972_ST_Y_LW    (-50)
#define DATA_AKM8972_ST_Z_UP    (-100)
#define DATA_AKM8972_ST_Z_LW    (-500)

#define DATA_AKM8963_ST_X_UP    (200)
#define DATA_AKM8963_ST_X_LW    (-200)
#define DATA_AKM8963_ST_Y_UP    (200)
#define DATA_AKM8963_ST_Y_LW    (-200)
#define DATA_AKM8963_ST_Z_UP    (-800)
#define DATA_AKM8963_ST_Z_LW    (-3200)


/* register definition*/
#define REG_3050_AUX_VDDIO      (0x13)
#define REG_3050_SLAVE_ADDR     (0x14)
#define REG_3050_SLAVE_REG      (0x18)
#define REG_3050_AUX_XOUT_H     (0x23)

#define REG_3500_OTP            (0x00)

#define REG_ST_GCT_X            (0x0D)
#define REG_I2C_MST_CTRL        (0x24)
#define REG_I2C_SLV0_ADDR       (0x25)
#define REG_I2C_SLV0_REG        (0x26)
#define REG_I2C_SLV0_CTRL       (0x27)
#define REG_I2C_SLV1_ADDR       (0x28)
#define REG_I2C_SLV1_REG        (0x29)
#define REG_I2C_SLV1_CTRL       (0x2A)

#define REG_I2C_SLV4_CTRL       (0x34)
#define REG_INT_PIN_CFG         (0x37)
#define REG_DMP_INT_STATUS      (0x39)
#define REG_EXT_SENS_DATA_00    (0x49)
#define REG_I2C_SLV1_DO         (0x64)
#define REG_I2C_MST_DELAY_CTRL  (0x67)
#define REG_BANK_SEL            (0x6D)
#define REG_MEM_START           (0x6E)
#define REG_MEM_RW              (0x6F)

/* bit definitions */
#define BIT_3050_VDDIO          (0x04)
#define BIT_3050_AUX_IF_EN      (0x20)
#define BIT_3050_AUX_IF_RST     (0x08)
#define BIT_3050_FIFO_RST       (0x02)

#define BIT_BYPASS_EN           (0x2)
#define BIT_WAIT_FOR_ES         (0x40)
#define BIT_I2C_READ            (0x80)
#define BIT_SLV_EN              (0x80)

#define BIT_DMP_EN              (0x80)
#define BIT_FIFO_EN		(0x40)
#define BIT_I2C_MST_EN          (0x20)
#define BIT_DMP_RST             (0x08)
#define BIT_FIFO_RST		(0x04)
#define BIT_I2C_MST_RST         (0x02)

#define BIT_SLV0_DLY_EN         (0x01)
#define BIT_SLV1_DLY_EN         (0x02)

#define BIT_FIFO_OVERFLOW	(0x10)
#define BIT_DATA_RDY_EN		(0x01)
#define BIT_DMP_INT_EN          (0x02)

#define BIT_PWR_ACCL_STBY       (0x38)
#define BIT_PWR_GYRO_STBY       (0x07)

#define BIT_GYRO_XOUT		(0x40)
#define BIT_GYRO_YOUT		(0x20)
#define BIT_GYRO_ZOUT		(0x10)
#define BIT_ACCEL_OUT		(0x08)
#define BITS_GYRO_OUT		(0x70)
#define BITS_SELF_TEST_EN       (0xE0)
#define BITS_3050_ACCL_OUT	(0x0E)
#define BITS_3050_POWER1        (0x30)
#define BITS_3050_POWER2        (0x10)
#define BITS_3050_GYRO_STANDBY  (0x38)
#define BITS_FSR		(0x18)
#define BITS_LPF		(0x07)
#define BITS_CLK		(0x07)
#define BIT_3500_FIFO_OVERFLOW	(0x10)
#define BIT_RESET               (0x80)
#define BIT_SLEEP		(0x40)
#define BIT_CYCLE               (0x20)
#define BIT_LPA_FREQ            (0xC0)

#define DMP_START_ADDR          (0x400)
#define BYTES_FOR_DMP           (16)
#define BYTES_PER_SENSOR        (6)
#define POWER_UP_TIME           (40)
#define MPU_MEM_BANK_SIZE        (256)
#define MPL_PROD_KEY(ver, rev) (ver * 100 + rev)
#define NUM_OF_PROD_REVS (ARRAY_SIZE(prod_rev_map))
/*---- MPU6050 Silicon Revisions ----*/
#define MPU_SILICON_REV_A2              1       /* MPU6050A2 Device */
#define MPU_SILICON_REV_B1              2       /* MPU6050B1 Device */

#define BIT_PRFTCH_EN                           0x40
#define BIT_CFG_USER_BANK                       0x20
#define BITS_MEM_SEL                            0x1f
/* time stamp tolerance */
#define TIME_STAMP_TOR           (5)
#define MAX_CATCH_UP             (5)
#define DEFAULT_ACCL_TRIM        (16384)
#define MAX_FIFO_RATE            (1000)
#define MIN_FIFO_RATE            (4)
#define ONE_K_HZ                 (1000)

/* authenticate key */
#define D_AUTH_OUT               (32)
#define D_AUTH_IN                (36)
#define D_AUTH_A                 (40)
#define D_AUTH_B                 (44)

/* flick related defines */
#define DATA_INT            (2097)
#define DATA_MSG_ON         (262144)

/*tap related defines */
#define INV_TAP                               0x08
#define INV_NUM_TAP_AXES (3)

#define INV_TAP_AXIS_X_POS                    0x20
#define INV_TAP_AXIS_X_NEG                    0x10
#define INV_TAP_AXIS_Y_POS                    0x08
#define INV_TAP_AXIS_Y_NEG                    0x04
#define INV_TAP_AXIS_Z_POS                    0x02
#define INV_TAP_AXIS_Z_NEG                    0x01
#define INV_TAP_ALL_DIRECTIONS                0x3f

#define INV_TAP_AXIS_X                        0x1
#define INV_TAP_AXIS_Y                        0x2
#define INV_TAP_AXIS_Z                        0x4

#define INV_TAP_AXIS_ALL                      \
		(INV_TAP_AXIS_X            |   \
		INV_TAP_AXIS_Y             |   \
		INV_TAP_AXIS_Z)

#define INT_SRC_TAP    0x01
#define INT_SRC_ORIENT 0x02

/*orientation related */
#define INV_X_UP                          0x01
#define INV_X_DOWN                        0x02
#define INV_Y_UP                          0x04
#define INV_Y_DOWN                        0x08
#define INV_Z_UP                          0x10
#define INV_Z_DOWN                        0x20
#define INV_ORIENTATION_ALL               0x3F

#define INV_ORIENTATION_FLIP              0x40
#define INV_X_AXIS_INDEX                 (0x00)
#define INV_Y_AXIS_INDEX                 (0x01)
#define INV_Z_AXIS_INDEX                 (0x02)

#define INV_ELEMENT_1                    (0x0001)
#define INV_ELEMENT_2                    (0x0002)
#define INV_ELEMENT_3                    (0x0004)
#define INV_ELEMENT_4                    (0x0008)
#define INV_ELEMENT_5                    (0x0010)
#define INV_ELEMENT_6                    (0x0020)
#define INV_ELEMENT_7                    (0x0040)
#define INV_ELEMENT_8                    (0x0080)
#define INV_ALL                          (0xFFFF)
#define INV_ELEMENT_MASK                 (0x00FF)
#define INV_GYRO_ACC_MASK                (0x007E)

enum inv_filter_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};
/*==== MPU6050B1 MEMORY ====*/
enum MPU_MEMORY_BANKS {
	MEM_RAM_BANK_0 = 0,
	MEM_RAM_BANK_1,
	MEM_RAM_BANK_2,
	MEM_RAM_BANK_3,
	MEM_RAM_BANK_4,
	MEM_RAM_BANK_5,
	MEM_RAM_BANK_6,
	MEM_RAM_BANK_7,
	MEM_RAM_BANK_8,
	MEM_RAM_BANK_9,
	MEM_RAM_BANK_10,
	MEM_RAM_BANK_11,
	MPU_MEM_NUM_RAM_BANKS,
	MPU_MEM_OTP_BANK_0 = 16
};

enum inv_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_FSR
};
enum inv_accl_fs_e {
	INV_FS_02G = 0,
	INV_FS_04G,
	INV_FS_08G,
	INV_FS_16G,
	NUM_ACCL_FSR
};

enum inv_clock_sel_e {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

int inv_hw_self_test(struct inv_gyro_state_s *st, int *gyro_bias_regular);
int inv_get_silicon_rev_mpu6050(struct inv_gyro_state_s *st);
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
	unsigned char reg, unsigned short length, unsigned char *data);
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
	unsigned short i2c_addr, unsigned char reg, unsigned char data);
#define inv_i2c_read(st, reg, len, data) \
	inv_i2c_read_base(st, st->i2c_addr, reg, len, data)
#define inv_i2c_single_write(st, reg, data) \
	inv_i2c_single_write_base(st, st->i2c_addr, reg, data)
#define inv_secondary_read(reg, len, data) \
	inv_i2c_read_base(st, st->plat_data.secondary_i2c_addr, reg, len, data)
#define inv_secondary_write(reg, data) \
	inv_i2c_single_write_base(st, st->plat_data.secondary_i2c_addr, \
		reg, data)
int inv_set_power_state(struct inv_gyro_state_s *st, unsigned char power_on);
int set_inv_enable(struct inv_gyro_state_s *st, unsigned long enable);
int mpu_memory_write(struct i2c_adapter *i2c_adap,
			    unsigned char mpu_addr,
			    unsigned short mem_addr,
			    unsigned int len, unsigned char const *data);
int mpu_memory_read(struct i2c_adapter *i2c_adap,
			   unsigned char mpu_addr,
			   unsigned short mem_addr,
			   unsigned int len, unsigned char *data);
void inv_setup_reg_mpu3050(struct inv_reg_map_s *reg);
int inv_init_config_mpu3050(struct inv_gyro_state_s *st);
irqreturn_t inv_read_fifo_mpu3050(int irq, void *dev_id);
int inv_reset_fifo(struct inv_gyro_state_s *st);
void inv_clear_kfifo(struct inv_gyro_state_s *st);
int inv_setup_mpu3050(struct inv_gyro_state_s *st);
int inv_register_kxtf9_slave(struct inv_gyro_state_s *st);
int create_device_attributes(struct device *dev,
	struct device_attribute **attrs);
void remove_device_attributes(struct device *dev,
	struct device_attribute **attrs);
int set_3050_bypass(struct inv_gyro_state_s *st, int enable);
s64 get_time_ns(void);
int inv_mpu3050_create_sysfs(struct inv_gyro_state_s *st);
int inv_mpu3050_remove_sysfs(struct inv_gyro_state_s *st);
int inv_get_accl_bias(struct inv_gyro_state_s *st, int *accl_bias_regular);
int set_power_mpu3050(struct inv_gyro_state_s *st, unsigned char power_on);
int reset_fifo_mpu3050(struct inv_gyro_state_s *st);
int inv_enable_tap_dmp(struct inv_gyro_state_s *st, unsigned char on);
int inv_enable_orientation_dmp(struct inv_gyro_state_s *st);
unsigned short inv_dmp_get_address(unsigned short key);
ssize_t inv_gyro_fifo_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t inv_gyro_fifo_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t inv_gyro_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t inv_gyro_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t inv_accl_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t inv_accl_fifo_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t inv_accl_fs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t inv_accl_fs_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t inv_accl_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t inv_dmp_firmware_write(struct file *fp, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t pos, size_t size);

ssize_t inv_dmp_firmware_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *bin_attr, char *buf,
			      loff_t off, size_t count);

#define mem_w(a, b, c) mpu_memory_write(st->sl_handle, \
			st->i2c_addr, a, b, c)
#define mem_w_key(key, b, c) mpu_memory_write(st->sl_handle, \
			st->i2c_addr, inv_dmp_get_address(key), b, c)

#endif  /* #ifndef _INV_GYRO_H_ */

