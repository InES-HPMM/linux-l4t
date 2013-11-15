/*
 * ina3221.c - driver for TI INA3221
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software. you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include "linux/ina3221.h"

#define DRIVER_NAME "ina3221"

/* Set non-zero to enable debug prints */
#define INA3221_DEBUG_PRINTS 0

#if INA3221_DEBUG_PRINTS
#define DEBUG_INA3221(x) (printk x)
#else
#define DEBUG_INA3221(x)
#endif

#define TRIGGERED 0
#define CONTINUOUS 1

#define busv_register_to_mv(x) ((x >> 3) * 8)
#define shuntv_register_to_uv(x) ((x >> 3) * 40)
#define uv_to_shuntv_register(x) (x/5) /*(x/40) << 3*/

#define CPU_THRESHOLD 2
#define CPU_FREQ_THRESHOLD 102000

/* Assume power can not exceed +-524W */
#define MAX_POWER_MW 0x000FFFFF
#define POWER_SHFT 12

struct ina3221_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
	struct ina3221_platform_data *plat_data;
	struct mutex mutex;
	u8 mode;
	struct notifier_block nb;
	struct notifier_block nb2;
	int shutdown_complete;
	int is_suspended;
};


static s32
__locked_set_crit_warn_register(struct i2c_client *client,
u32 index, u32 reg_addr)
{
	struct ina3221_data *data = i2c_get_clientdata(client);
	u32 shunt_volt_limit;
	s32 ret;
	shunt_volt_limit =
		data->plat_data->crit_conf_limits[index] *
			data->plat_data->shunt_resistor[index];
	shunt_volt_limit = uv_to_shuntv_register(shunt_volt_limit);
	DEBUG_INA3221(("Current = %d\n", shunt_volt_limit));
	ret = i2c_smbus_write_word_data(client, reg_addr,
				__constant_cpu_to_be16(shunt_volt_limit));
	return ret;
}

static s32 __locked_set_crit_warn_limits(struct i2c_client *client)
{
	struct ina3221_data *data = i2c_get_clientdata(client);
	u32 i;
	u32 crit_reg_addr;
	u32 warn_reg_addr;
	s32 ret = 0;
	if (!(data->plat_data->crit_conf_limits) ||
		!(data->plat_data->warn_conf_limits))
		return -EINVAL;

	for (i = 0; i < INA3221_NUMBER_OF_RAILS; i++) {
		crit_reg_addr = (INA3221_CRIT_CHAN1 + (i * 2));
		warn_reg_addr = (INA3221_WARN_CHAN1 + (i * 2));
		if (data->plat_data->crit_conf_limits[i] != -1) {
			ret = __locked_set_crit_warn_register(client,
							i, crit_reg_addr);
			if (ret < 0)
				break;
		}

		if (data->plat_data->warn_conf_limits[i] != -1) {
			ret = __locked_set_crit_warn_register(client,
							i, warn_reg_addr);
			if (ret < 0)
				break;
		}
	}
	return ret;
}

static s32 __locked_power_down_ina3221(struct i2c_client *client)
{
	s32 ret;
	struct ina3221_data *data = i2c_get_clientdata(client);
	if (data->shutdown_complete)
		return -ENODEV;
	ret = i2c_smbus_write_word_data(client, INA3221_CONFIG,
		INA3221_POWER_DOWN);
	if (ret < 0)
		dev_err(&client->dev, "Power down failure status: 0x%x", ret);
	return ret;
}

static s32 __locked_power_up_ina3221(struct i2c_client *client, int config)
{
	s32 ret;
	struct ina3221_data *data = i2c_get_clientdata(client);
	if (data->shutdown_complete)
		return -ENODEV;
	ret = i2c_smbus_write_word_data(client, INA3221_CONFIG,
					__constant_cpu_to_be16(config));
	if (ret < 0)
		dev_err(&client->dev,
			"Config data write failed, error: 0x%x", ret);
	return ret;
}

static s32 show_mode(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&data->mutex);
	ret = sprintf(buf, "%d\n", data->mode);
	mutex_unlock(&data->mutex);
	return ret;
}

static s32 show_rail_name(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index = attr->index;
	return sprintf(buf, "%s\n", data->plat_data->rail_name[index]);
}

static s32 show_voltage(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, bus_volt_reg_addr;
	s32 ret;
	s32 voltage_mv;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}
	index = attr->index;
	bus_volt_reg_addr = (INA3221_BUS_VOL_CHAN1 + (index * 2));

	if (data->mode == TRIGGERED) {
		ret = __locked_power_up_ina3221(client,
				data->plat_data->trig_conf_data);
		if (ret < 0) {
			dev_err(dev,
			"Power up failed, status: 0x%x\n", ret);
			goto error;
		}
	}

	/* getting voltage readings in milli volts*/
	ret = i2c_smbus_read_word_data(client,
			bus_volt_reg_addr);
	if (ret < 0)
		goto error;
	voltage_mv = be16_to_cpu(ret);
	voltage_mv = (voltage_mv << 16) >> 16;
	voltage_mv = busv_register_to_mv(voltage_mv);
	DEBUG_INA3221(("Ina3221 bus voltage in mv: %d\n", voltage_mv));

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		ret = __locked_power_down_ina3221(client);
		if (ret < 0)
			goto error;
	}

	DEBUG_INA3221(("%s volt = %d\n", __func__, voltage_mv));
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mV\n", voltage_mv);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static s32 show_current(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, shunt_volt_reg_addr;
	s32 ret;
	s32 voltage_uv;
	s32 current_ma;
	s32 inverse_shunt_resistor;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}
	index = attr->index;
	shunt_volt_reg_addr = (INA3221_SHUNT_VOL_CHAN1 + (index * 2));

	if (data->mode == TRIGGERED) {
		ret = __locked_power_up_ina3221(client,
				data->plat_data->trig_conf_data);
		if (ret < 0) {
			dev_err(dev,
				"power up failed sts: 0x%x\n", ret);
			goto error;
		}
	}

	/* getting voltage readings in micro volts*/
	ret = i2c_smbus_read_word_data(client,
			shunt_volt_reg_addr);
	if (ret < 0)
		goto error;
	voltage_uv = be16_to_cpu(ret);
	DEBUG_INA3221(("Ina3221 shunt voltage reg Value: 0x%x\n", voltage_uv));
	voltage_uv = (voltage_uv << 16) >> 16;
	voltage_uv = shuntv_register_to_uv(voltage_uv);
	DEBUG_INA3221(("Ina3221 shunt voltage in uv: %d\n", voltage_uv));

	/* shunt_resistor is received in mOhms */
	inverse_shunt_resistor = 1000 / data->plat_data->shunt_resistor[index];
	current_ma = (voltage_uv * inverse_shunt_resistor) / 1000;

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		ret = __locked_power_down_ina3221(client);
		if (ret < 0)
			goto error;
	}

	DEBUG_INA3221(("%s current = %d\n", __func__, current_ma));
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mA\n", current_ma);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static s32 show_current2(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, shunt_volt_reg_addr;
	s32 ret;
	s32 voltage_uv;
	s32 current_ma;
	s32 inverse_shunt_resistor;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}

	/*return 0 if INA is off*/
	if (data->mode == TRIGGERED) {
		mutex_unlock(&data->mutex);
		return sprintf(buf, "%d mA\n", 0);
	}

	index = attr->index;
	shunt_volt_reg_addr = (INA3221_SHUNT_VOL_CHAN1 + (index * 2));
	/* getting voltage readings in micro volts*/
	ret = i2c_smbus_read_word_data(client,
			shunt_volt_reg_addr);
	if (ret < 0)
		goto error;
	voltage_uv = be16_to_cpu(ret);
	DEBUG_INA3221(("Ina3221 shunt voltage reg Value: 0x%x\n", voltage_uv));
	voltage_uv = (voltage_uv << 16) >> 16;
	voltage_uv = shuntv_register_to_uv(voltage_uv);
	DEBUG_INA3221(("Ina3221 shunt voltage in uv: %d\n", voltage_uv));

	/* shunt_resistor is received in mOhms */
	inverse_shunt_resistor = 1000 / data->plat_data->shunt_resistor[index];
	current_ma = (voltage_uv * inverse_shunt_resistor) / 1000;

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		ret = __locked_power_down_ina3221(client);
		if (ret < 0)
			goto error;
	}

	DEBUG_INA3221(("%s current = %d\n", __func__, current_ma));
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mA\n", current_ma);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static s32 __locked_calculate_power(struct i2c_client *client,
					u8 shunt_volt_reg_addr,
					u8 bus_volt_reg_addr,
					int index)
{

	struct ina3221_data *data = i2c_get_clientdata(client);
	s32 voltage_mv;
	s32 voltage_uv;
	s32 inverse_shunt_resistor;
	s32 current_ma;
	s32 power_mw;
	s32 ret;
	/* getting voltage readings in micro volts*/
	ret = i2c_smbus_read_word_data(client,
			shunt_volt_reg_addr);
	if (ret < 0)
		goto error;
	voltage_uv = be16_to_cpu(ret);
	DEBUG_INA3221(("Ina3221 shunt voltage reg Value: 0x%x\n", voltage_uv));
	voltage_uv = (voltage_uv << 16) >> 16;
	voltage_uv = shuntv_register_to_uv(voltage_uv);
	DEBUG_INA3221(("Ina3221 shunt voltage in uv: %d\n", voltage_uv));

	/* getting voltage readings in milli volts*/
	ret = i2c_smbus_read_word_data(client,
			bus_volt_reg_addr);
	if (ret < 0)
		goto error;
	voltage_mv = be16_to_cpu(ret);
	DEBUG_INA3221(("Ina3221 bus voltage reg Value: 0x%x\n", voltage_mv));
	voltage_mv = (voltage_mv << 16) >> 16;
	voltage_mv = busv_register_to_mv(voltage_mv);
	DEBUG_INA3221(("Ina3221 bus voltage in mv: %d\n", voltage_mv));

	/* shunt_resistor is received in mOhms */
	inverse_shunt_resistor = 1000 / data->plat_data->shunt_resistor[index];
	current_ma = voltage_uv * inverse_shunt_resistor / 1000;
	power_mw = voltage_mv * current_ma / 1000;
	return power_mw & MAX_POWER_MW;
error:
	return -EIO;
}

static s32 show_power(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, bus_volt_reg_addr, shunt_volt_reg_addr;
	s32 ret;
	s32 power_mw;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}
	index = attr->index;
	bus_volt_reg_addr = (INA3221_BUS_VOL_CHAN1 + (index * 2));
	shunt_volt_reg_addr = (INA3221_SHUNT_VOL_CHAN1 + (index * 2));

	if (data->mode == TRIGGERED) {
		ret = __locked_power_up_ina3221(client,
				data->plat_data->trig_conf_data);
		if (ret < 0) {
			dev_err(dev,
			"power up failed sts: 0x%x\n", ret);
			goto error;
		}
	}
	/*Will get -EIO on error*/
	power_mw = __locked_calculate_power(client, shunt_volt_reg_addr,
						bus_volt_reg_addr, index);
	if (power_mw < 0) {
		ret = power_mw;
		goto error;
	}

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		ret = __locked_power_down_ina3221(client);
		if (ret < 0)
			goto error;
	}
	power_mw = (power_mw << POWER_SHFT) >> POWER_SHFT;
	DEBUG_INA3221(("%s power = %d\n", __func__, power_mw));
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mW\n", power_mw);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static s32 show_power2(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, bus_volt_reg_addr, shunt_volt_reg_addr;
	s32 power_mw;
	s32 ret;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}

	/*return 0 if INA is off*/
	if (data->mode == TRIGGERED) {
		mutex_unlock(&data->mutex);
		return sprintf(buf, "%d mW\n", 0);
	}
	index = attr->index;
	bus_volt_reg_addr = (INA3221_BUS_VOL_CHAN1 + (index * 2));
	shunt_volt_reg_addr = (INA3221_SHUNT_VOL_CHAN1 + (index * 2));

	power_mw = __locked_calculate_power(client, shunt_volt_reg_addr,
						bus_volt_reg_addr, index);
	if (power_mw < 0) {
		ret = power_mw;
		goto error;
	}
	power_mw = (power_mw << POWER_SHFT) >> POWER_SHFT;
	DEBUG_INA3221(("%s power = %d\n", __func__, power_mw));
	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mW\n", power_mw);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static int __locked_ina3221_switch(struct ina3221_data *data,
		struct i2c_client *client,
		int cpus, int cpufreq)
{
	int ret = 0;
	if ((data->mode == TRIGGERED) &&
		((cpus >= CPU_THRESHOLD) ||
		(cpufreq >= CPU_FREQ_THRESHOLD))) {
		/**
		 * Turn INA on when cpu frequency crosses threshold or number of cpus
		 * crosses threshold
		 */
		DEBUG_INA3221(("Turning on ina3221, cpus:%d, cpufreq:%d\n",
					cpus, cpufreq));
		ret = __locked_power_up_ina3221(client,
				data->plat_data->cont_conf_data);
		if (ret < 0) {
			dev_err(&client->dev,
					"INA can't be turned on: 0x%x\n", ret);
			return ret;
		}
		data->mode = CONTINUOUS;
		return ret;
	} else if ((data->mode == CONTINUOUS) &&
			 (cpus < CPU_THRESHOLD) &&
			(cpufreq < CPU_FREQ_THRESHOLD)) {
		/*
		 * Turn off ina when number of cpu cores on are below threshold
		 * and cpu frequency are below threshold
		 */
		DEBUG_INA3221(("Turning off ina3221, cpus:%d, cpufreq:%d\n",
				cpus, cpufreq));
		ret = __locked_power_down_ina3221(client);
		if (ret < 0) {
			dev_err(&client->dev,
					"INA can't be turned off: 0x%x\n", ret);
			return ret;
		}
		data->mode = TRIGGERED;
		return ret;
	} else {
		return ret;
	}
}

static int ina3221_cpufreq_notify(struct notifier_block *nb,
					unsigned long event,
					void *hcpu)
{
	int ret = 0;
	int cpufreq;
	int cpus;
	struct ina3221_data *data = container_of(nb, struct ina3221_data, nb2);
	struct i2c_client *client = data->client;
	if (event == CPUFREQ_POSTCHANGE) {
		mutex_lock(&data->mutex);
		if (data->is_suspended) {
			mutex_unlock(&data->mutex);
			return 0;
		}
		cpufreq = ((struct cpufreq_freqs *)hcpu)->new;
		cpus = num_online_cpus();
		DEBUG_INA3221(("***INA3221 CPUfreq notified freq:%d cpus:%d\n",
						cpufreq, cpus));
		ret = __locked_ina3221_switch(data, client, cpus, cpufreq);
		if (ret < 0)
			goto error;
		mutex_unlock(&data->mutex);
		return 0;
	} else
		return 0;
error:
	mutex_unlock(&data->mutex);
	dev_err(&client->dev, "INA can't be turned off/on: 0x%x\n", ret);
	return 0;
}

static int ina3221_hotplug_notify(struct notifier_block *nb,
					unsigned long event,
					void *hcpu)
{
	struct ina3221_data *data = container_of(nb, struct ina3221_data,
						nb);
	struct i2c_client *client = data->client;
	int cpus;
	int ret = 0;
	int cpufreq = 0;
	if (event == CPU_ONLINE || event == CPU_DEAD) {
		mutex_lock(&data->mutex);
		cpufreq = cpufreq_quick_get(0);
		cpus = num_online_cpus();
		DEBUG_INA3221(("INA3221 hotplug notified cpufreq:%d cpus:%d\n",
				cpufreq, cpus));
		ret = __locked_ina3221_switch(data, client, cpus, cpufreq);
		if (ret < 0)
			goto error;
		mutex_unlock(&data->mutex);
		return 0;
	} else
		return 0;
error:
	mutex_unlock(&data->mutex);
	dev_err(&client->dev, "INA can't be turned off/on: 0x%x\n", ret);
	return 0;
}

static s32 set_crit(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	s32 retval, crit_reg_addr;
	u32 index = 0;
	long int curr_limit;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		retval = -ENODEV;
		goto error;
	}
	index = attr->index;

	if (kstrtol(buf, 10, &curr_limit) < 0) {
		retval = -EINVAL;
		goto error;
	}

	if (data->mode == TRIGGERED) {
		retval = __locked_power_up_ina3221(client,
				data->plat_data->trig_conf_data);
		if (retval < 0) {
			dev_err(dev,
			"Power up failed, status: 0x%x\n", retval);
			goto error;
		}
	}
	data->plat_data->crit_conf_limits[index] = curr_limit;
	crit_reg_addr = (INA3221_CRIT_CHAN1 + (index * 2));
	retval = __locked_set_crit_warn_register(client, index, crit_reg_addr);

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		retval = __locked_power_down_ina3221(client);
		if (retval < 0)
			goto error;
	}

error:
	mutex_unlock(&data->mutex);
	if (retval >= 0)
		return count;
	return retval;
}

static s32 show_crit(struct device *dev,
		struct device_attribute *devattr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	u8 index, crit_reg_addr;
	s32 ret, current_ma;
	s32 voltage_uv;
	s32 inverse_shunt_resistor;

	mutex_lock(&data->mutex);
	if (data->shutdown_complete) {
		ret = -ENODEV;
		goto error;
	}
	index = attr->index;

	if (data->mode == TRIGGERED) {
		ret = __locked_power_up_ina3221(client,
				data->plat_data->trig_conf_data);
		if (ret < 0) {
			dev_err(dev,
			"Power up failed, status: 0x%x\n", ret);
			goto error;
		}
	}

	crit_reg_addr = (INA3221_CRIT_CHAN1 + (index * 2));

	/* getting voltage readings in micro volts*/
	ret = i2c_smbus_read_word_data(client,
			crit_reg_addr);
	if (ret < 0)
		goto error;
	voltage_uv = be16_to_cpu(ret);
	DEBUG_INA3221(("Ina3221 crit voltage reg Value: 0x%x\n", voltage_uv));
	voltage_uv = (voltage_uv << 16) >> 16;
	voltage_uv = shuntv_register_to_uv(voltage_uv);
	DEBUG_INA3221(("Ina3221 crit voltage in uv: %d\n", voltage_uv));

	/* shunt_resistor is received in mOhms */
	inverse_shunt_resistor = 1000 / data->plat_data->shunt_resistor[index];
	current_ma = (voltage_uv * inverse_shunt_resistor) / 1000;

	DEBUG_INA3221(("Ina3221 crit current in mA: %d\n", current_ma));

	if (data->mode == TRIGGERED) {
		/* set ina3221 to power down mode */
		ret = __locked_power_down_ina3221(client);
		if (ret < 0)
			goto error;
	}

	mutex_unlock(&data->mutex);
	return sprintf(buf, "%d mA\n", current_ma);
error:
	mutex_unlock(&data->mutex);
	dev_err(dev, "%s: failed\n", __func__);
	return ret;
}

static struct sensor_device_attribute ina3221[] = {
	SENSOR_ATTR(rail_name_0, S_IRUGO, show_rail_name, NULL, 0),
	SENSOR_ATTR(in1_input_0, S_IRUGO, show_voltage, NULL, 0),
	SENSOR_ATTR(curr1_input_0, S_IRUGO, show_current, NULL, 0),
	SENSOR_ATTR(curr2_input_0, S_IRUGO, show_current2, NULL, 0),
	SENSOR_ATTR(power1_input_0, S_IRUGO, show_power, NULL, 0),
	SENSOR_ATTR(power2_input_0, S_IRUGO, show_power2, NULL, 0),
	SENSOR_ATTR(rail_name_1, S_IRUGO, show_rail_name, NULL, 1),
	SENSOR_ATTR(in1_input_1, S_IRUGO, show_voltage, NULL, 1),
	SENSOR_ATTR(curr1_input_1, S_IRUGO, show_current, NULL, 1),
	SENSOR_ATTR(curr2_input_1, S_IRUGO, show_current2, NULL, 1),
	SENSOR_ATTR(power1_input_1, S_IRUGO, show_power, NULL, 1),
	SENSOR_ATTR(power2_input_1, S_IRUGO, show_power2, NULL, 1),
	SENSOR_ATTR(rail_name_2, S_IRUGO, show_rail_name, NULL, 2),
	SENSOR_ATTR(in1_input_2, S_IRUGO, show_voltage, NULL, 2),
	SENSOR_ATTR(curr1_input_2, S_IRUGO, show_current, NULL, 2),
	SENSOR_ATTR(curr2_input_2, S_IRUGO, show_current2, NULL, 2),
	SENSOR_ATTR(power1_input_2, S_IRUGO, show_power, NULL, 2),
	SENSOR_ATTR(power2_input_2, S_IRUGO, show_power2, NULL, 2),
	SENSOR_ATTR(running_mode, S_IRUGO, show_mode, NULL, 0),
	SENSOR_ATTR(crit_cur_limit_0, S_IWUSR|S_IRUGO, show_crit, set_crit, 0),
	SENSOR_ATTR(crit_cur_limit_1, S_IWUSR|S_IRUGO, show_crit, set_crit, 1),
	SENSOR_ATTR(crit_cur_limit_2, S_IWUSR|S_IRUGO, show_crit, set_crit, 2),
/* mode setting :
 * running_mode = 0 ---> Triggered mode
 * running_mode > 0 ---> Continuous mode
 */
};

static int ina3221_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ina3221_data *data;
	int ret, i;

	data = devm_kzalloc(&client->dev, sizeof(struct ina3221_data),
						GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto exit;
	}
	i2c_set_clientdata(client, data);
	data->plat_data = client->dev.platform_data;
	mutex_init(&data->mutex);

	data->mode = TRIGGERED;
	data->shutdown_complete = 0;
	data->is_suspended = 0;
	/* reset ina3221 */
	ret = i2c_smbus_write_word_data(client, INA3221_CONFIG,
		__constant_cpu_to_be16((INA3221_RESET)));
	if (ret < 0) {
		dev_err(&client->dev, "ina3221 reset failure status: 0x%x\n",
			ret);
		goto exit_free;
	}

	for (i = 0; i < ARRAY_SIZE(ina3221); i++) {
		ret = device_create_file(&client->dev, &ina3221[i].dev_attr);
		if (ret) {
			dev_err(&client->dev, "device_create_file failed.\n");
			goto exit_remove;
		}
	}
	data->client = client;
	data->nb.notifier_call = ina3221_hotplug_notify;
	data->nb2.notifier_call = ina3221_cpufreq_notify;
	register_hotcpu_notifier(&(data->nb));
	cpufreq_register_notifier(&(data->nb2), CPUFREQ_TRANSITION_NOTIFIER);
	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	ret = __locked_set_crit_warn_limits(client);
	if (ret < 0) {
		dev_info(&client->dev, "Not able to set warn and crit limits!\n");
		/*Not an error condition, could let the probe continue*/
	}

	/* set ina3221 to power down mode */
	ret = __locked_power_down_ina3221(client);
	if (ret < 0)
		goto exit_remove;

	return 0;

exit_remove:
	while (i--)
		device_remove_file(&client->dev, &ina3221[i].dev_attr);
exit_free:
	devm_kfree(&client->dev, data);
exit:
	return ret;
}

static int ina3221_remove(struct i2c_client *client)
{
	u8 i;
	struct ina3221_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->mutex);
	__locked_power_down_ina3221(client);
	hwmon_device_unregister(data->hwmon_dev);
	mutex_unlock(&data->mutex);
	unregister_hotcpu_notifier(&(data->nb));
	cpufreq_unregister_notifier(&(data->nb2), CPUFREQ_TRANSITION_NOTIFIER);
	for (i = 0; i < ARRAY_SIZE(ina3221); i++)
		device_remove_file(&client->dev, &ina3221[i].dev_attr);
	return 0;
}

static void ina3221_shutdown(struct i2c_client *client)
{
	struct ina3221_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->mutex);
	__locked_power_down_ina3221(client);
	data->shutdown_complete = 1;
	mutex_unlock(&data->mutex);
}

#ifdef CONFIG_PM_SLEEP
static int ina3221_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	s32 ret = 0;
	mutex_lock(&data->mutex);
	ret = __locked_power_down_ina3221(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"INA can't be turned off: 0x%x\n", ret);
		goto error;
	}
	data->mode = TRIGGERED;
	data->is_suspended = 1;
error:
	mutex_unlock(&data->mutex);
	return ret;
}

static int ina3221_resume(struct device *dev)
{
	int cpufreq, cpus, ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct ina3221_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->mutex);
	cpufreq = cpufreq_quick_get(0);
	cpus = num_online_cpus();
	ret = __locked_ina3221_switch(data, client, cpus, cpufreq);
	if (ret < 0)
		dev_err(&client->dev,
			"INA can't be turned off/on: 0x%x\n", ret);

	data->is_suspended = 0;
	mutex_unlock(&data->mutex);
	return ret;
}
#endif

static const struct i2c_device_id ina3221_id[] = {
	{DRIVER_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ina3221_id);

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ina3221_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ina3221_suspend,
				ina3221_resume)
};
#endif

static struct i2c_driver ina3221_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &ina3221_pm_ops,
#endif
	},
	.probe		= ina3221_probe,
	.remove		= ina3221_remove,
	.shutdown	= ina3221_shutdown,
	.id_table	= ina3221_id,
};

static int __init ina3221_init(void)
{
	return i2c_add_driver(&ina3221_driver);
}

static void __exit ina3221_exit(void)
{
	i2c_del_driver(&ina3221_driver);
}

module_init(ina3221_init);
module_exit(ina3221_exit);
MODULE_LICENSE("GPL");
