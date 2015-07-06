/*
 * Temperature Monitor Driver
 *
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * drivers/hwmon/tmon-tmp411.c
 *
 */

/* Note: Copied temperature conversion code from tmp401 driver */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_data/tmon_tmp411.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>

/*
 * The TMP411/NVT210 registers, note some registers have different addresses for
 * reading and writing
 */
#define REG_STATUS			0x02
#define REG_CFG_READ			0x03
#define REG_CFG_WRITE			0x09
#define REG_CON_RATE_READ		0x04
#define REG_CONV_RATE_WRITE		0x0A
#define REG_MAN_ID			0xFE

/* Flags */
#define CFG_EXTEND		(0x1<<2)
#define CFG_ALERT_MASK		(0x1<<7)
#define CFG_STANDBY		(0x1<<6)
#define CFG_ALERT_DISABLE	(0x1<<5)
#define STATUS_LOCAL_CRIT	0x01
#define STATUS_REMOTE_CRIT	0x02
#define STATUS_REMOTE_OPEN	0x04
#define STATUS_REMOTE_LOW	0x08
#define STATUS_REMOTE_HIGH	0x10
#define STATUS_LOCAL_LOW	0x20
#define STATUS_LOCAL_HIGH	0x40
#define STATUS_RTHRM		0x02
#define STATUS_LTHRM		0x01
#define FUTURE_USE		0


/* Based on POR values of CONVERSION RATE REGISTER (125ms) +
   RESOLUTION REGISTER(conversion time: 12.5ms) */
#define CONVER_TIME 140

#define TMON_RD(clnt, reg, val) \
do { \
	err = tmon_read(clnt, reg, val); \
	if (err) \
		return err; \
} while (0)

#define TMON_WRT(clnt, reg, val) \
do { \
	err = i2c_smbus_write_byte_data(clnt, reg, val); \
	if (err < 0) \
		return err; \
} while (0)

enum alert_type {
	NO_ALERT = 0,
	REMOTE_HIGH_ALERT,
	REMOTE_LOW_ALERT,
};

enum tmp_sensor {
	TMP411 = 0,
	NVT210,
};

enum temp_type {
	LOCAL = 0,
	REMOTE,
};

static enum tmp_sensor tmon_device;

/* TMP411/NVT210 registers
Note: Local temperature low bytes not applicable in case of NVT210
index 0 for local temperatures and 1 for remote temperatures */
static const u8 REG_TEMP_MSB[2]			= { 0x00, 0x01 };
static const u8 REG_TEMP_LSB[2]			= { 0x15, 0x10 };
static const u8 REG_TEMP_LOW_LIMIT_MSB_RD[2]	= { 0x06, 0x08 };
static const u8 REG_TEMP_LOW_LIMIT_MSB_WRT[2]	= { 0x0C, 0x0E };
static const u8 REG_TEMP_LOW_LIMIT_LSB[2]	= { 0x17, 0x14 };
static const u8 REG_TEMP_HIGH_LIMIT_MSB_RD[2]	= { 0x05, 0x07 };
static const u8 REG_TEMP_HIGH_LIMIT_MSB_WRT[2]	= { 0x0B, 0x0D };
static const u8 REG_TEMP_HIGH_LIMIT_LSB[2]	= { 0x16, 0x13 };
static const u8 REG_TEMP_CRIT_LIMIT[2]		= { 0x20, 0x19 };

#ifdef CONFIG_PM
/* index 0 for local temperatures and 1 for remote temperatures */
static u16 temp_low_limit[2];
static u16 temp_high_limit[2];
static u8 temp_crit_limit[2];
static u8 conv_rate;
static u8 config;
#endif

struct tmon_info {
	int mode;
	struct i2c_client *client;
	struct i2c_client *ara; /*Alert Response Address for clearing alert */
	struct tmon_plat_data *pdata;
	struct mutex update_lock; /*  For TMP411 registers read/writes */
	struct mutex sysfs_alert_lock; /* For block alert sysfs protection */
	spinlock_t alert_spinlock; /* Used for protection between ISR
						and user process */
	struct i2c_smbus_alert_setup *alert_info;
	wait_queue_head_t alert_wait_queue;
	char therm_alert;
	char disable_intr_reqr;
	int irq_num;
	long sysfs_therm_alert_timeout;
	unsigned long last_updated; /*  in jiffies */
	struct thermal_zone_device *thz;

};


/* Return Both High byte and Low byte */
static u16 temp_to_reg(long temp, u8 config)
{
	if (config & CFG_EXTEND) {
		temp = clamp_val(temp, -64000, 191000);
		temp += 64000;
	} else
		temp = clamp_val(temp, 0, 127000);

	return (temp * 160 + 312) / 625; /* Copied from TMP411 driver */
}

static s32 tmon_read(struct i2c_client *client, u8 reg, u8 *value)
{
	s32 tmp;

	tmp = i2c_smbus_read_byte_data(client, reg);
	if (tmp < 0)
		return -EINVAL;

	*value = tmp;

	return 0;
}

/* Expects both High byte and Low byte, If not Low bytes then
   make Low byte as 0 */
static s32 reg_to_temp(u16 reg, u8 config)
{
	s32 temp = reg;

	if (config & CFG_EXTEND)
		temp -= 64 * 256;

	return (temp * 625 + 80) / 160; /* Copied from TMP411 driver */
}

static s32 tmon_read_remote_temp(struct i2c_client *client,
					     s32 *ptemp)
{
	u8 config;
	u8 tmp;
	int err;
	u16 temperature = 0;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	TMON_RD(client, REG_TEMP_MSB[1], &tmp);
	temperature = ((u16)tmp) << 8;
	TMON_RD(client, REG_TEMP_LSB[1], &tmp);
	temperature |= tmp;

	*ptemp = reg_to_temp(temperature, config) +
			data->pdata->remote_offset;

	return 0;
}

static s32 tmon_read_local_temp(struct i2c_client *client,
					    int *ptemp)
{
	u8 config;
	u8 tmp;
	int err;
	u16 temperature = 0;
	TMON_RD(client, REG_CFG_READ, &config);
	TMON_RD(client, REG_TEMP_MSB[0], &tmp);
	temperature = ((u16)tmp) << 8;
	if (tmon_device == TMP411) {
		TMON_RD(client, REG_TEMP_LSB[0], &tmp);
		temperature |= tmp;
	}

	*ptemp = reg_to_temp(temperature, config);

	return 0;
}


static s32 tmon_read_low_limit(struct i2c_client *client,
				s32 *ptemp, enum temp_type typ)
{
	u8 config;
	u8 tmp;
	int err;
	u16 temperature = 0;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	TMON_RD(client, REG_TEMP_LOW_LIMIT_MSB_RD[typ], &tmp);
	temperature = ((u16)tmp) << 8;
	if ((typ == REMOTE) || (typ == LOCAL && tmon_device == TMP411)) {
		TMON_RD(client, REG_TEMP_LOW_LIMIT_LSB[typ], &tmp);
		temperature |= tmp;
	}
	*ptemp = reg_to_temp(temperature, config);
	if (typ == REMOTE)
		*ptemp = *ptemp + data->pdata->remote_offset;
	return 0;
}

static s32 tmon_read_high_limit(struct i2c_client *client,
						s32 *ptemp, enum temp_type typ)
{
	u8 config;
	u8 tmp;
	int err;
	u16 temperature = 0;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	TMON_RD(client, REG_TEMP_HIGH_LIMIT_MSB_RD[typ], &tmp);
	temperature = ((u16)tmp) << 8;
	if ((typ == REMOTE) || (typ == LOCAL && tmon_device == TMP411)) {
		TMON_RD(client, REG_TEMP_HIGH_LIMIT_LSB[typ], &tmp);
		temperature |= tmp;
	}
	*ptemp = reg_to_temp(temperature, config);
	if (typ == REMOTE)
		*ptemp = *ptemp + data->pdata->remote_offset;
	return 0;
}

static s32 tmon_read_critical_limit(struct i2c_client *client,
					s32 *ptemp, enum temp_type typ)
{
	u8 config;
	u8 tmp;
	int err;
	u16 temperature = 0;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	TMON_RD(client, REG_TEMP_CRIT_LIMIT[typ], &tmp);
	temperature =  ((u16)tmp) << 8;
	*ptemp = reg_to_temp(temperature, config);
	if (typ == REMOTE)
		*ptemp = *ptemp + data->pdata->remote_offset;
	return 0;
}

static int tmon_write_high_limit(struct i2c_client *client,
				s32 temp, enum temp_type typ)
{
	u8 config;
	u8 reg8_val;
	int err;
	u16 reg16_val;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	if (typ == REMOTE)
		temp = temp - data->pdata->remote_offset;

	reg16_val = temp_to_reg(temp, config);
	reg8_val = reg16_val >> 8;

	mutex_lock(&data->update_lock);
	TMON_WRT(client, REG_TEMP_HIGH_LIMIT_MSB_WRT[typ], reg8_val);
	if (tmon_device == TMP411)
		TMON_WRT(client, REG_TEMP_HIGH_LIMIT_LSB[typ],
			(reg16_val & 0xFF));
	mutex_unlock(&data->update_lock);
	return 0;
}

static int tmon_write_low_limit(struct i2c_client *client,
				s32 temp, enum temp_type typ)
{
	u8 config;
	u8 reg8_val;
	int err;
	u16 reg16_val;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);

	if (typ == REMOTE)
		temp = temp - data->pdata->remote_offset;

	reg16_val = temp_to_reg(temp, config);
	reg8_val = reg16_val >> 8;

	mutex_lock(&data->update_lock);
	TMON_WRT(client, REG_TEMP_LOW_LIMIT_MSB_WRT[typ], reg8_val);
	if (tmon_device == TMP411)
		TMON_WRT(client, REG_TEMP_LOW_LIMIT_LSB[typ],
			(reg16_val & 0xFF));
	mutex_unlock(&data->update_lock);
	return 0;
}

#if FUTURE_USE
static int tmon_write_critical_limit(struct i2c_client *client,
					s32 temp, enum temp_type typ)
{
	u8 config;
	u16 reg16_val;
	u8 reg8_val;
	int err;
	struct tmon_info *data = i2c_get_clientdata(client);

	TMON_RD(client, REG_CFG_READ, &config);
	if (typ == REMOTE)
		temp = temp - data->pdata->remote_offset;

	reg16_val = temp_to_reg(temp, config);
	reg8_val =  reg16_val >> 8;
	mutex_lock(&data->update_lock);
	TMON_WRT(client, REG_TEMP_CRIT_LIMIT[typ], reg8_val);
	mutex_unlock(&data->update_lock);
	return 0;
}
#endif

static ssize_t show_temp_value(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int index = to_sensor_dev_attr(devattr)->index;
	int temperature = 0;

	if (index == 0) /* local temperature */
		tmon_read_local_temp(to_i2c_client(dev), &temperature);
	else
		tmon_read_remote_temp(to_i2c_client(dev), &temperature);

	return sprintf(buf, "%d\n", temperature);
}

static ssize_t show_remote_low_limit(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	s32 low_limit = 0;

	tmon_read_low_limit(to_i2c_client(dev), &low_limit, REMOTE);
	return sprintf(buf, "%d\n", low_limit);
}

static ssize_t show_remote_high_limit(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	s32 high_limit = 0;

	tmon_read_high_limit(to_i2c_client(dev), &high_limit, REMOTE);
	return sprintf(buf, "%d\n", high_limit);
}

static int clear_alert(struct tmon_info *data)
{
	struct i2c_client *ara = data->ara;
	int alrt_stat;

	alrt_stat = i2c_smbus_read_byte(ara);
	return 0;
}

static u8  is_alert_present(u8 status)
{
	if ((status & STATUS_REMOTE_LOW) || (status & STATUS_REMOTE_HIGH) ||
		(status & STATUS_LOCAL_LOW) || (status & STATUS_LOCAL_HIGH))
		return 1;
	else
		return 0;
}

static u8 manage_alert(struct i2c_client *client)
{
	u8 status;
	int err;
	struct tmon_info *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);
	/* If gpio line is asserted then try to clear the alert  */
	if (data->pdata->alert_gpio != -1)
		if (!gpio_get_value(data->pdata->alert_gpio))
			clear_alert(data);

	/* Read the status register twice to get rid of stale alert */
	TMON_RD(client, REG_STATUS, &status);
	if (is_alert_present(status)) {
		msleep(CONVER_TIME); /* Wait for another conversion to happen */
		TMON_RD(client, REG_STATUS, &status);
	}
	mutex_unlock(&data->update_lock);
	return status;
}

static ssize_t return_status(char *buf, u8 status)
{
	enum alert_type alert = NO_ALERT;
	if (status & STATUS_REMOTE_HIGH)
		alert = REMOTE_HIGH_ALERT;
	else if (status & STATUS_REMOTE_LOW)
		alert = REMOTE_LOW_ALERT;

	return sprintf(buf, "%d\n", alert);
}

static ssize_t show_alert(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	u8 status;
	status = manage_alert(to_i2c_client(dev));
	return return_status(buf, status);
}

static ssize_t store_alert_block_timeout(struct device *dev,
	struct device_attribute *devattr, const char *buf,  size_t count)
{

	struct tmon_info *data = i2c_get_clientdata(to_i2c_client(dev));
	long val;
	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	if (!mutex_trylock(&data->sysfs_alert_lock))
		return -EAGAIN;
	data->sysfs_therm_alert_timeout = val;
	mutex_unlock(&data->sysfs_alert_lock);

	return count;
}

/* This function blocks until alert occurs or
   timeout occurs(If any timeout value > 0 is set) */
static ssize_t show_alert_blocking(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct tmon_info *data = i2c_get_clientdata(to_i2c_client(dev));
	u8 status;
	int err;
	unsigned long intr_flags;

	mutex_lock(&data->sysfs_alert_lock);
	status = manage_alert(to_i2c_client(dev));

	if ((status & STATUS_REMOTE_LOW) || (status & STATUS_REMOTE_HIGH)) {
		mutex_unlock(&data->sysfs_alert_lock);
		return return_status(buf, status);
	}


	/* Even though data->therm_alert is shared between ISR and process,
	   protection is not required here. Because before any process access
	   this varaible here, alert interrupt would have been disabled */
	data->therm_alert = 0;

	data->disable_intr_reqr = 1;
	/* Enable Interrupt for thermal alert */
	enable_irq(data->irq_num);

	/* Wait event on therm alert */
	if (data->sysfs_therm_alert_timeout > 0) {
		wait_event_interruptible_timeout(data->alert_wait_queue,
			(data->therm_alert & 0x1),
			msecs_to_jiffies(data->sysfs_therm_alert_timeout));

	} else {
		wait_event_interruptible(
				data->alert_wait_queue,
				(data->therm_alert & 0x1));
	}

	/* data->therm_alert is shared between ISR and process.
	   So, spinlock used for protection */
	spin_lock_irqsave(&data->alert_spinlock, intr_flags);
	if (data->disable_intr_reqr) {
		disable_irq_nosync(data->irq_num);
		data->disable_intr_reqr = 0;
	}
	spin_unlock_irqrestore(&data->alert_spinlock, intr_flags);
	mutex_lock(&data->update_lock);
	TMON_RD(to_i2c_client(dev), REG_STATUS, &status);
	mutex_unlock(&data->update_lock);
	mutex_unlock(&data->sysfs_alert_lock);

	return return_status(buf, status);

}

static ssize_t show_remote_shutdown_limit(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	s32 temp;
	s32 err;
	err = tmon_read_critical_limit(to_i2c_client(dev), &temp, REMOTE);
	if (err)
		return err;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t store_remote_low_limit(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	long val;
	s32 err;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	err = tmon_write_low_limit(to_i2c_client(dev), val, REMOTE);
	if (err)
		return err;
	return count;
}

static ssize_t store_remote_high_limit(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	long val;
	int err;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	err = tmon_write_high_limit(to_i2c_client(dev), val, REMOTE);
	if (err)
		return err;
	return count;
}

static struct sensor_device_attribute tmp411_attr[] = {
	SENSOR_ATTR(local_temp, S_IRUGO, show_temp_value, NULL, 0),

	SENSOR_ATTR(remote_temp, S_IRUGO, show_temp_value, NULL, 1),

	SENSOR_ATTR(remote_temp_low_limit, S_IWUSR | S_IRUGO,
			show_remote_low_limit, store_remote_low_limit, 1),

	SENSOR_ATTR(remote_temp_high_limit, S_IWUSR | S_IRUGO,
			show_remote_high_limit, store_remote_high_limit, 1),

	SENSOR_ATTR(remote_temp_alert, S_IRUGO, show_alert, NULL,
			STATUS_REMOTE_LOW | STATUS_REMOTE_HIGH),

	SENSOR_ATTR(remote_temp_alert_blocking, S_IWUSR | S_IRUGO,
				show_alert_blocking, store_alert_block_timeout,
				STATUS_REMOTE_LOW | STATUS_REMOTE_HIGH),
	SENSOR_ATTR(remote_shutdown_limit, S_IRUGO, show_remote_shutdown_limit,
				NULL, 1),
};

static irqreturn_t  tmon_alert_isr(int irq, void *d)
{
	struct tmon_info *data = d;
	unsigned long int intr_flags;

	spin_lock_irqsave(&data->alert_spinlock, intr_flags);
	data->therm_alert = 1;
	/* Disable the interrupt */
	if (data->disable_intr_reqr) {
		disable_irq_nosync(irq);
		data->disable_intr_reqr = 0;
	}
	spin_unlock_irqrestore(&data->alert_spinlock, intr_flags);

	/* Wake up the process waiting on therm alert */
	if (waitqueue_active(&data->alert_wait_queue))
		wake_up_all(&data->alert_wait_queue);

	return IRQ_HANDLED;

}

/* returns trip temperature, it considers hysteresis if current
 * temperature is greater or equal to trip temp, next trip
 * temp would be trip_temp - hyst.
 */
static int tmon_get_trip_temp(struct thermal_zone_device *thz,
						int trip, long *temp) {
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;
	unsigned int num_trips = pdata->num_trips;

	long trip_temp = pdata->trips[trip].trip_temp;
	long hysteresis = pdata->trips[trip].hysteresis;

	BUG_ON(trip < 0 || trip >= num_trips);

	if (thz->temperature >= trip_temp) {
		trip_temp -= hysteresis;
		pdata->trips[trip].tripped = true;
	} else if (pdata->trips[trip].tripped) {
		trip_temp -= hysteresis;
		if (thz->temperature <= trip_temp)
			pdata->trips[trip].tripped = false;
	}

	*temp = trip_temp;

	return 0;
}

static int tmon_loc_get_trip_temp(struct thermal_zone_device *thz,
						int trip, long *temp) {
	return tmon_get_trip_temp(thz, trip, temp);
}

/* returns hysteresis,this is a common function can be used by remote
 * thermal zone
 */
static int tmon_get_trip_hyst(struct thermal_zone_device *thz,
						int trip, long *temp) {
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;
	unsigned int num_trips = pdata->num_trips;

	BUG_ON(trip < 0 || trip >= num_trips);

	*temp = pdata->trips[trip].hysteresis;

	return 0;
}

/* returns hysterisis of local temperature based thermal zone */
static int tmon_loc_get_trip_hyst(struct thermal_zone_device *thz,
						int trip, long *temp) {
	return tmon_get_trip_hyst(thz, trip, temp);
}

/* returns the local temperature, temperature of sensor not tegra
 * chip temperature
 */
static int tmon_loc_get_temp(struct thermal_zone_device *thz,
							long *temp)
{
	int ret;
	int temp1;
	struct tmon_info *tmon_data = thz->devdata;
	ret = tmon_read_local_temp(tmon_data->client,
						&temp1);
	*temp = (long)temp1;
	if (ret < 0)
		return -1;

	return 0;
}

/* returns trend ,which is used in kernel thermal framework to
 * decide the next state of cooling device
 * if trend is THERMAL_TREND_RAISING thermal framework tries to
 * set the next state to next cooling state, possible at that
 * trip point
 */
static int tmon_loc_get_trend(struct thermal_zone_device *thz,
				int trip, enum thermal_trend *trend)
{
	long trip_temp;

	thz->ops->get_trip_temp(thz, trip, &trip_temp);

	if (thz->temperature > trip_temp)
		*trend = THERMAL_TREND_RAISING;
	else if (thz->temperature <= trip_temp)
		*trend = THERMAL_TREND_DROPPING;
	return 0;
}

/* binds cooling device to the corresponding trip point */
static int tmon_loc_bind(struct thermal_zone_device *thz,
			struct thermal_cooling_device *cdev)
{
	int i;
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;
	for (i = 0; i < pdata->num_trips; i++) {
		if (!strcmp(pdata->trips[i].cdev_type, cdev->type)) {
			thermal_zone_bind_cooling_device(thz, i, cdev,
				pdata->trips[i].upper,
				pdata->trips[i].lower);
			pdata->trips[i].bound = true;
		}

	}
	return 0;
}

static int tmon_loc_unbind(struct thermal_zone_device *thz,
			struct thermal_cooling_device *cdev)
{
	int i;
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;

	for (i = 0; i < pdata->num_trips; i++) {
		if (!strcmp(pdata->trips[i].cdev_type, cdev->type) &&
						pdata->trips[i].bound)
			thermal_zone_unbind_cooling_device(thz, i, cdev);
			pdata->trips[i].bound = false;
	}
	return 0;
}

/* returns the trip type
 * trip type can be
 * THERMAL_TRIP_ACTIVE,
 * THERMAL_TRIP_PASSIVE,
 * THERMAL_TRIP_HOT,
 * THERMAL_TRIP_CRITICAL
 */
static int tmon_loc_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type)
{
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;
	unsigned int num_trips = pdata->num_trips;

	BUG_ON(trip < 0 || trip >= num_trips);

	*type = pdata->trips[trip].trip_type;
	return 0;
}

/* used to change trip point in the case if trip_temp debugfs
 * entry is exposed as writable through board file via trip.mode = 0x1
 */
static int tmon_loc_set_trip_temp(struct thermal_zone_device *thz,
					int trip,
					long temp)
{
	struct tmon_info *tmon_data = thz->devdata;
	struct tmon_plat_data *pdata = tmon_data->pdata;
	unsigned int num_trips = pdata->num_trips;

	BUG_ON(trip < 0 || trip >= num_trips);
	pdata->trips[trip].trip_temp = temp;
	return 0;
}

static struct thermal_zone_device_ops tmon_loc_ops = {
	.get_temp = tmon_loc_get_temp,
	.bind = tmon_loc_bind,
	.unbind = tmon_loc_unbind,
	.get_trip_type = tmon_loc_get_trip_type,
	.get_trip_temp = tmon_loc_get_trip_temp,
	.get_trip_hyst = tmon_loc_get_trip_hyst,
	.get_trend = tmon_loc_get_trend,
	.set_trip_temp = tmon_loc_set_trip_temp,
};

static int tmon_tmp411_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct tmon_plat_data *tmon_pdata =
	    client->dev.platform_data;
	struct tmon_info *data;
	int err;
	int i;
	u8 man_id;
	u8 config;
	u64 mask;
	char extern_tsensor_loc_name[] = "extern_tsensor_loc";

	if (tmon_pdata == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "insufficient functionality!\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct tmon_info), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	TMON_RD(client, REG_MAN_ID, &man_id);
	if (man_id == 0x41) {
		tmon_device = NVT210;
		dev_info(&client->dev, "detected NVT210\n");
	} else if (man_id == 0x55) {
		tmon_device = TMP411;
		dev_info(&client->dev, "detected TMP411\n");
	} else {
		dev_warn(&client->dev,
		"unsuported t-sensor with manufacturer-id:0x%x\n",
		man_id);
		return -EINVAL;
	}

	TMON_RD(client, REG_CFG_READ, &config);

	/* Enable Alert, Extended mode and disable stand by */
	config |= CFG_EXTEND;
	config &= ~CFG_ALERT_MASK;
	config &= ~CFG_STANDBY;
	config &= ~CFG_ALERT_DISABLE;
	err = i2c_smbus_write_byte_data(client, REG_CFG_WRITE, config);

	/*FIXME: Is it required to wait for one temperature conversion? */

	if (err < 0) {
		dev_warn(&client->dev,
		"\n Failed to write config for temperature sensor\n");
		return err;
	}

	data->client = client;
	i2c_set_clientdata(client, data);
	data->pdata = tmon_pdata;
	mutex_init(&data->update_lock);
	mutex_init(&data->sysfs_alert_lock);
	spin_lock_init(&data->alert_spinlock);
	data->sysfs_therm_alert_timeout = 0;
	data->therm_alert = 0;

	/*
	 * Suppose, if we get alert interrupts immediately after irq
	 * registeration and before disable_irq_nosync, then once entered
	 * in to Isr, if the below variable is set then irq will be disabled,
	 * otherwise it's likely that continious interrupts comes.
	 */
	data->disable_intr_reqr = 1;
	init_waitqueue_head(&data->alert_wait_queue);

	if (data->pdata->alert_gpio != -1) {
		/*  IRQ for therm alert */
		data->irq_num = gpio_to_irq(data->pdata->alert_gpio);
		err = request_irq(data->irq_num,
				tmon_alert_isr,
				IRQF_TRIGGER_LOW,
				"tmon alert",
				data);
		dev_info(&client->dev, "\n Tmon IRQ registered for gpio:%d\n",
					data->pdata->alert_gpio);

		if (!err) {
			/*Disable now and enable only when sysfs
							alert is opened  */
			disable_irq_nosync(data->irq_num);
			data->alert_info =
				devm_kzalloc(&client->dev,
					sizeof(struct i2c_smbus_alert_setup),
					GFP_KERNEL);
			if (!data->alert_info) {
				err = -ENOMEM;
				goto err_irq;
			}
			data->alert_info->alert_edge_triggered = 1;
			data->ara = i2c_setup_smbus_alert(client->adapter,
							data->alert_info);
		} else {
			dev_warn(&client->dev,
				"failed to get irq for alert gpio:%d\n",
				data->pdata->alert_gpio);
			return err;
		}
	}


	for (i = 0; i < ARRAY_SIZE(tmp411_attr); i++) {
		err = device_create_file(&client->dev,
					&tmp411_attr[i].dev_attr);
		if (err)
			goto err_exit;
	}

	mask = 0;
	/* if mask of ith trip is 1 then make the debugfs entry for
	 * this trip point writable
	 */
	for (i = 0; i < tmon_pdata->num_trips; i++)
		if (tmon_pdata->trips[i].mask)
			mask |= 1ULL << i;

	data->thz = thermal_zone_device_register(extern_tsensor_loc_name,
				tmon_pdata->num_trips,
				mask,
				data,
				&tmon_loc_ops,
				tmon_pdata->tzp,
				tmon_pdata->passive_delay,
				tmon_pdata->polling_delay);

	if (IS_ERR_OR_NULL(data->thz))
		dev_err(&client->dev,
		"extern_tsensor_loc thermal zone device not registered\n");

	dev_info(&client->dev, "Temperature Monitor enabled\n");
	return 0;

err_exit:
	for (i = 0; i < ARRAY_SIZE(tmp411_attr); i++)
		device_remove_file(&client->dev, &tmp411_attr[i].dev_attr);
	i2c_unregister_device(data->ara);
err_irq:
	free_irq(data->irq_num, data);
	return err;
}

static int tmon_tmp411_remove(struct i2c_client *client)
{
	struct tmon_info *data = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < ARRAY_SIZE(tmp411_attr); i++)
		device_remove_file(&client->dev, &tmp411_attr[i].dev_attr);

	free_irq(data->irq_num, data);
	i2c_unregister_device(data->ara);
	kfree(data->alert_info);
	data->alert_info = NULL;
	kfree(data);
	data = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tmon_tmp411_suspend(struct device *dev)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	u8 tmp;
	int err;

	/* save temperature limits for restore during resume */
	for (i = 0; i < 2; i++) {
		/*
		 * High byte must be read first immediately followed
		 * by the low byte
		 */
		TMON_RD(client, REG_TEMP_LOW_LIMIT_MSB_RD[i], &tmp);
		temp_low_limit[i] = ((u16)tmp) << 8;
		if ((i == REMOTE) || (i == LOCAL && tmon_device == TMP411)) {
			TMON_RD(client, REG_TEMP_LOW_LIMIT_LSB[i], &tmp);
			temp_low_limit[i] |= tmp;
		}

		TMON_RD(client, REG_TEMP_HIGH_LIMIT_MSB_RD[i], &tmp);
		temp_high_limit[i] = ((u16)tmp) << 8;
		if ((i == REMOTE) || (i == LOCAL && tmon_device == TMP411)) {
			TMON_RD(client,  REG_TEMP_HIGH_LIMIT_LSB[i],  &tmp);
			temp_high_limit[i] |= tmp;
		}

		TMON_RD(client,  REG_TEMP_CRIT_LIMIT[i], &temp_crit_limit[i]);
	}
	TMON_RD(client, REG_CON_RATE_READ, &conv_rate);
	TMON_RD(client, REG_CFG_READ, &config);

	return 0;
}

static int tmon_tmp411_resume(struct device *dev)
{
	int i;
	int err;
	u8 curr_cnfg;
	u8 limit_correction = 0;
	struct i2c_client *client = to_i2c_client(dev);

	TMON_RD(client, REG_CFG_READ, &curr_cnfg);

	/* Stop the temperature conversions */
	curr_cnfg = (curr_cnfg & (~CFG_STANDBY));
	TMON_WRT(client, REG_CFG_WRITE, curr_cnfg);

	/*  Restore temperature limits */
	for (i = 0; i < 2; i++) {
		TMON_WRT(client,
			REG_TEMP_HIGH_LIMIT_MSB_WRT[i],
			(temp_high_limit[i] >> 8) + limit_correction);

		if ((i == REMOTE) || (i == LOCAL && tmon_device == TMP411)) {
			TMON_WRT(client,
				REG_TEMP_HIGH_LIMIT_LSB[i],
				(temp_high_limit[i] & 0xFF));
		}

		TMON_WRT(client,
			 REG_TEMP_LOW_LIMIT_MSB_WRT[i],
			 (temp_low_limit[i] >> 8) + limit_correction);

		if ((i == REMOTE) || (i == LOCAL && tmon_device == TMP411)) {
			TMON_WRT(client,
				REG_TEMP_LOW_LIMIT_LSB[i],
				(temp_low_limit[i] & 0xFF));
		}

		TMON_WRT(client,
			REG_TEMP_CRIT_LIMIT[i],
			temp_crit_limit[i] + limit_correction);
	}
	TMON_WRT(client, REG_CONV_RATE_WRITE, conv_rate);

	/* Start the temperature conversion and restore config */
	TMON_WRT(client, REG_CFG_WRITE, config);

	/* FIXME: Is it required to wait for one temperature conversion? */

	return 0;
}

#endif

static const struct dev_pm_ops tegra_tmp411_dev_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = tmon_tmp411_suspend,
	.resume = tmon_tmp411_resume,
#endif
};

/* tmon-tmp411 driver struct */
static const struct i2c_device_id tmon_tmp411_id[] = {
	{"tmon-tmp411-sensor", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tmon_tmp411_id);

static struct i2c_driver tmon_tmp411_driver = {
	.driver = {
			.name = "tmon-tmp411-sensor",
#ifdef CONFIG_PM_SLEEP
			.pm = &tegra_tmp411_dev_pm_ops,
#endif
		   },
	.probe = tmon_tmp411_probe,
	.remove = tmon_tmp411_remove,
	.id_table = tmon_tmp411_id,
};

static int __init tmon_tmp411_module_init(void)
{
	return i2c_add_driver(&tmon_tmp411_driver);
}

static void __exit tmon_tmp411_module_exit(void)
{
	i2c_del_driver(&tmon_tmp411_driver);
}

module_init(tmon_tmp411_module_init);
module_exit(tmon_tmp411_module_exit);

MODULE_AUTHOR("Manoj Chourasia <mchourasia@nvidia.com>");
MODULE_DESCRIPTION("Temperature Monitor module");
MODULE_LICENSE("GPL");
