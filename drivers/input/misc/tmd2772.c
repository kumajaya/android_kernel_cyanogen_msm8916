/*
 * Description: Linux device driver for Taos ambient light and
 * proximity sensors.
 *
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Author:		John Koshi
 * History:		09/16/2009 - Initial creation
 *			10/09/2009 - Triton version
 *			12/21/2009 - Probe/remove mode
 *			02/07/2010 - Add proximity
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include "taos_common.h"

#define LOG_TAG "SENSOR_ALS_PROX"
//#define DEBUG_ON

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s: %d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

/* device */
static dev_t const tmd2772_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static dev_t const tmd2772_light_dev_t = MKDEV(MISC_MAJOR, 102);

/* gain table */
static u8 als_gain_table[] = {1, 8, 16, 120};

static struct lux_data lux_table[] = {
	{9830,  8320,  15360},
	{12452, 10554, 22797},
	{14746, 6234,  11430},
	{17695, 3968,  6400 },
	{0,	0,     0    }
};
static int lux_history[ALS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};

/* device reg init values */
u8 taos_triton_reg_init[16] = {0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
			0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "AMS",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "1.0",
	.sensor_power = "0.5",
	.min_delay = 500000, /* in microseconds (us) */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 500,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.flags = 2,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "AMS",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "10",
	.resolution = "1.0",
	.sensor_power = "0.5",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.flags = 3,
};

/* function implementation */
static ssize_t attr_chip_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);

	if (!strlen(taos_data->chip_name))
		return sprintf(buf, "%s", "No chip name has been given");
	return sprintf(buf, "%s", taos_data->chip_name);

}

static ssize_t attr_prox_check_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);

	if (!strlen(taos_data->chip_name))
		return sprintf(buf, "%s", "No chip name has been given");
	return sprintf(buf, "%s(0x%x)",
			taos_data->chip_name, TMD2772_DEVICE_ID);
}

static ssize_t attr_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_on is %s\n",
			taos_data->prox_on ? "true" : "false");
}

static ssize_t attr_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	bool value = false;

	SENSOR_LOG_INFO("enter\n");

	if (strtobool(buf, &value))
		return -EINVAL;

	mutex_lock(&taos_data->lock);

	if (value)
		taos_prox_on(taos_data);
	else
		taos_prox_off(taos_data);

	mutex_unlock(&taos_data->lock);

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 1;
	int read_value;
	int ret = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&taos_data->lock);

	if (value == 1) {
		ret = taos_read_cal_value(taos_data, PATH_PROX_OFFSET, &read_value);
		if (ret == 0)
			taos_cfg->prox_config_offset = read_value;

		ret = taos_read_cal_value(taos_data, PATH_PROX_UNCOVER_DATA, &read_value);
		if (ret == 0 && read_value > 0) {
			taos_data->prox_uncover_data = read_value;
			taos_data->prox_thres_hi_min = taos_data->prox_uncover_data +
						PROX_THRESHOLD_SAFE_DISTANCE;
			taos_data->prox_thres_hi_max = taos_data->prox_thres_hi_min +
						PROX_THRESHOLD_DISTANCE / 2;
			taos_data->prox_thres_hi_max = (taos_data->prox_thres_hi_max >
						PROX_THRESHOLD_HIGH_MAX) ?
						PROX_THRESHOLD_HIGH_MAX :
						taos_data->prox_thres_hi_max;
			taos_data->prox_thres_lo_min = taos_data->prox_uncover_data +
						PROX_THRESHOLD_DISTANCE;
			taos_data->prox_thres_lo_max = taos_data->prox_uncover_data +
						PROX_THRESHOLD_DISTANCE * 2;
			SENSOR_LOG_INFO("prox_uncover_data = %d\n",
						taos_data->prox_uncover_data);
			SENSOR_LOG_INFO("prox_thres_hi range is [%d--%d]\n",
						taos_data->prox_thres_hi_min,
						taos_data->prox_thres_hi_max);
			SENSOR_LOG_INFO("prox_thres_lo range is [%d--%d]\n",
						taos_data->prox_thres_lo_min,
						taos_data->prox_thres_lo_max);
		}

		ret = taos_read_cal_value(taos_data, PATH_PROX_CAL_THRESHOLD, &read_value);
		if (ret < 0) {
			SENSOR_LOG_ERROR("need to create %s\n",
					PATH_PROX_CAL_THRESHOLD);

			ret = taos_write_cal_file(taos_data, PATH_PROX_CAL_THRESHOLD, 0);
			if (ret < 0) {
				SENSOR_LOG_ERROR("failed to create %s\n",
						PATH_PROX_CAL_THRESHOLD);
				mutex_unlock(&taos_data->lock);
				return -EINVAL;
			}
			taos_data->prox_calibrate_flag = true;
		} else {
			if (read_value == 0) {
				taos_data->prox_calibrate_flag = true;
			} else if (read_value > (taos_data->prox_thres_hi_min)) {
				taos_data->prox_calibrate_flag = false;
				taos_data->prox_manual_calibrate_threshold = read_value;

				taos_cfg->prox_threshold_hi = read_value;
				taos_cfg->prox_threshold_hi = (taos_cfg->prox_threshold_hi <
						taos_data->prox_thres_hi_max) ?
						taos_cfg->prox_threshold_hi :
						taos_data->prox_thres_hi_max;
				taos_cfg->prox_threshold_hi = (taos_cfg->prox_threshold_hi >
						taos_data->prox_thres_hi_min) ?
						taos_cfg->prox_threshold_hi :
						taos_data->prox_thres_hi_min;

				taos_cfg->prox_threshold_lo = taos_cfg->prox_threshold_hi -
						PROX_THRESHOLD_DISTANCE;
				taos_cfg->prox_threshold_lo = (taos_cfg->prox_threshold_lo <
						taos_data->prox_thres_lo_max) ?
						taos_cfg->prox_threshold_lo :
						taos_data->prox_thres_lo_max;
				taos_cfg->prox_threshold_lo = (taos_cfg->prox_threshold_lo >
						taos_data->prox_thres_lo_min) ?
						taos_cfg->prox_threshold_lo :
						taos_data->prox_thres_lo_min;

				input_report_rel(taos_data->p_idev, REL_Y, taos_cfg->prox_threshold_hi);
				input_report_rel(taos_data->p_idev, REL_Z, taos_cfg->prox_threshold_lo);
				input_sync(taos_data->p_idev);

				SENSOR_LOG_INFO("get hi/lo = %d/%d from %s\n",
							taos_cfg->prox_threshold_hi,
				taos_cfg->prox_threshold_lo, PATH_PROX_CAL_THRESHOLD);
			}
		}
	} else {
		mutex_unlock(&taos_data->lock);
		SENSOR_LOG_ERROR("input error, valid: 1\n");
		return -EINVAL;
	}

	mutex_unlock(&taos_data->lock);
	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_led_pluse_cnt_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_led_pluse_cnt is %d\n",
			taos_data->taos_cfg->prox_pulse_cnt);
}

static ssize_t attr_prox_led_pluse_cnt_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	taos_cfg->prox_pulse_cnt = val;
	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_COUNT,
		taos_cfg->prox_pulse_cnt) < 0)
		SENSOR_LOG_ERROR("failed to write prox_pulse_cnt reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_adc_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_adc_time is 2.73 * %d ms\n",
			256 - taos_data->taos_cfg->prox_adc_time);
}

static ssize_t attr_prox_adc_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	taos_cfg->prox_adc_time = 256 - val;
	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_TIME,
		taos_cfg->prox_adc_time) < 0)
		SENSOR_LOG_ERROR("failed to write prox_adc_time reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_led_strength_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	char *p_led_sth[4] = {"100", "50", "25", "12.5"};
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_led_strength is %s mA\n",
			p_led_sth[(taos_data->taos_cfg->prox_gain) >> 6]);
}

static ssize_t attr_prox_led_strength_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (val > 4 || val <= 0) {
		SENSOR_LOG_ERROR("input error, valid: 1~4");
		return -EINVAL;
	}

	val = 4 - val;
	taos_cfg->prox_gain = (taos_cfg->prox_gain & 0x3F) | (val << 6);

	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN, taos_cfg->prox_gain) < 0)
		SENSOR_LOG_ERROR("failed to write prox_led_strength reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_debug_delay_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_debug_delay is %d\n",
			taos_data->prox_debug_delay_time);
}

static ssize_t attr_prox_debug_delay_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_data->prox_debug_delay_time = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_set_prox_calibrate(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (val > 1) {
		taos_data->prox_calibrate_times = val;
		taos_prox_calibrate(taos_data);
	} else {
		SENSOR_LOG_ERROR("input error, valid: > 1\n");
	}

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_threshold_high_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_threshold_hi is %d",
			taos_data->taos_cfg->prox_threshold_hi);
}

static ssize_t attr_prox_threshold_high_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_data->taos_cfg->prox_threshold_hi = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_threshold_low_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_threshold_lo is %d",
			taos_data->taos_cfg->prox_threshold_lo);
}

static ssize_t attr_prox_threshold_low_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_data->taos_cfg->prox_threshold_lo = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_result_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_calibrate_result is %d",
			taos_data->prox_calibrate_result);
}

static ssize_t attr_prox_cali_high_param_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_calibrate_hi_param is %d\n",
			taos_data->prox_calibrate_hi_param);
}

static ssize_t attr_prox_cali_high_param_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 0)
		taos_data->prox_calibrate_hi_param = val;
	else
		SENSOR_LOG_ERROR("input error, valid: > 0\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_cali_low_param_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_calibrate_lo_param is %d\n",
			taos_data->prox_calibrate_lo_param);
}

static ssize_t attr_prox_cali_low_param_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 0)
		taos_data->prox_calibrate_lo_param = val;
	else
		SENSOR_LOG_ERROR("input error, valid: > 0\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;
	return sprintf(buf, "prox_threshold_lo is %d, prox_threshold_hi is %d\n",
			taos_cfg->prox_threshold_lo, taos_cfg->prox_threshold_hi);
}

static ssize_t attr_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long value = 0;
	int read_value;
	int ret = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtol(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&taos_data->lock);

	if (value == 1)	{
		ret = taos_read_cal_value(taos_data, PATH_PROX_CAL_THRESHOLD, &read_value);
		if (ret < 0) {
			mutex_unlock(&taos_data->lock);
			return -EINVAL;
		}

		if (read_value > taos_data->prox_thres_hi_min) {
			taos_data->prox_calibrate_flag = false;
			taos_data->prox_manual_calibrate_threshold = read_value;

			taos_cfg->prox_threshold_hi = read_value;
			taos_cfg->prox_threshold_hi = (taos_cfg->prox_threshold_hi <
					taos_data->prox_thres_hi_max) ?
					taos_cfg->prox_threshold_hi :
					taos_data->prox_thres_hi_max;
			taos_cfg->prox_threshold_hi = (taos_cfg->prox_threshold_hi >
					taos_data->prox_thres_hi_min) ?
					taos_cfg->prox_threshold_hi :
					taos_data->prox_thres_hi_min;

			taos_cfg->prox_threshold_lo = taos_cfg->prox_threshold_hi -
					PROX_THRESHOLD_DISTANCE;
			taos_cfg->prox_threshold_lo = (taos_cfg->prox_threshold_lo <
					taos_data->prox_thres_lo_max) ?
					taos_cfg->prox_threshold_lo :
					taos_data->prox_thres_lo_max;
			taos_cfg->prox_threshold_lo = (taos_cfg->prox_threshold_lo >
					taos_data->prox_thres_lo_min) ?
					taos_cfg->prox_threshold_lo :
					taos_data->prox_thres_lo_min;

			input_report_rel(taos_data->p_idev, REL_Y, taos_cfg->prox_threshold_hi);
			input_report_rel(taos_data->p_idev, REL_Z, taos_cfg->prox_threshold_lo);
			input_sync(taos_data->p_idev);

			SENSOR_LOG_INFO("prox_th_high = %d\n", taos_cfg->prox_threshold_hi);
			SENSOR_LOG_INFO("prox_th_low = %d\n", taos_cfg->prox_threshold_lo);
		}
	} else {
		mutex_unlock(&taos_data->lock);
		return -EINVAL;
	}

	mutex_unlock(&taos_data->lock);
	SENSOR_LOG_INFO("exit\n");

	return size;
}

static ssize_t attr_prox_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "flag_prox_debug is %s\n",
			taos_data->flag_prox_debug ? "true" : "false");
}

static ssize_t attr_prox_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (val)
		taos_data->flag_prox_debug = true;
	else
		taos_data->flag_prox_debug = false;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_start_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "flag_prox_debug is %s\n",
			taos_data->flag_prox_debug ? "true" : "false");
}

static ssize_t attr_prox_thres_hi_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", PROX_THRESHOLD_HIGH_MAX);
}

static ssize_t attr_prox_thres_lo_min_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d", taos_data->prox_thres_hi_min);
}

static ssize_t attr_prox_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d", taos_data->prox_data_max);
}

static ssize_t attr_prox_manual_cali_threshold_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d", taos_data->prox_manual_calibrate_threshold);
}

static ssize_t attr_prox_prox_wakelock_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "proximity_wakelock is %s\n",
			taos_data->proximity_wakelock.locked ? "true" : "false");
}

static ssize_t attr_prox_prox_wakelock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int recv = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	if (kstrtouint(buf, 10, &recv))
		return -EINVAL;

	mutex_lock(&taos_data->lock);
	if (recv) {
		taos_wakelock_ops(&(taos_data->proximity_wakelock), true);
	} else {
		hrtimer_cancel(&taos_data->prox_unwakelock_timer);
		taos_wakelock_ops(&(taos_data->proximity_wakelock), false);
	}
	mutex_unlock(&taos_data->lock);
	return size;
}

static void taos_irq_ops(struct taos_data *taos_data, bool enable, bool flag_sync)
{
	if (enable == taos_data->irq_enabled) {
		SENSOR_LOG_INFO("doubule %s irq, retern here\n",
				enable ? "enable" : "disable");
		return;
	} else {
        taos_data->irq_enabled = enable;
    }

	if (enable) {
		enable_irq(taos_data->client->irq);
	} else {
		if (flag_sync)
			disable_irq(taos_data->client->irq);
		else
			disable_irq_nosync(taos_data->client->irq);
	}
}

static ssize_t attr_irq_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "flag_irq is %s\n",
			taos_data->irq_enabled ? "true" : "false");
}

static ssize_t attr_irq_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	if (val)
		taos_irq_ops(taos_data, true, true);
	else
		taos_irq_ops(taos_data, false, true);

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_wait_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "wait_time is 2.73 * %d ms\n",
			256 - taos_data->taos_cfg->wait_time);
}

static ssize_t attr_wait_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	taos_cfg->wait_time = 256 - val;
	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_WAIT_TIME,
		taos_cfg->wait_time) < 0)
		SENSOR_LOG_ERROR("failed to write wait_time reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_start_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "flag_prox_debug is %s\n",
			taos_data->flag_prox_debug ? "true" : "false");
}

static ssize_t attr_prox_offset_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 1;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&taos_data->lock);
	if (value == 1)
		schedule_delayed_work(&taos_data->prox_offset_cal_work,
						msecs_to_jiffies(0));
	else
		SENSOR_LOG_ERROR("input error, valid: 1\n");

	mutex_unlock(&taos_data->lock);
	SENSOR_LOG_INFO("exit\n");

	return size;
}

static ssize_t attr_prox_offset_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_config_offset is %d\n",
			taos_data->taos_cfg->prox_config_offset);
}

static ssize_t attr_prox_offset_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_cfg->prox_config_offset = val;
	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_cfg->prox_config_offset) < 0)
		SENSOR_LOG_ERROR("failed to write the prox_config_offset reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_result_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_offset_cal_result is %d",
			taos_data->prox_offset_cal_result);
}

static ssize_t attr_prox_data_safa_range_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MAX);
}

static ssize_t attr_prox_data_safa_range_min_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MIN);
}

static ssize_t attr_reg_addr_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "g_reg_addr is 0x%02x\n", taos_data->g_reg_addr);
}

static ssize_t attr_reg_addr_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	taos_data->g_reg_addr = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_reg_data_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	unsigned char i = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	if (100 == taos_data->g_reg_addr) {
		for (i = 0x00; i <= 0x0F; i++) {
			i2c_smbus_write_byte(taos_data->client, (TAOS_TRITON_CMD_REG | i));
			SENSOR_LOG_INFO("reg[0x%02x] = 0x%02x\n", i,
					i2c_smbus_read_byte(taos_data->client));
		}

		for (i = 0x12; i <= 0x19; i++) {
			i2c_smbus_write_byte(taos_data->client, (TAOS_TRITON_CMD_REG | i));
			SENSOR_LOG_INFO("reg[0x%02x] = 0x%02x\n", i,
					i2c_smbus_read_byte(taos_data->client));
		}

		i2c_smbus_write_byte(taos_data->client, (TAOS_TRITON_CMD_REG | 0x1E));
		SENSOR_LOG_INFO("reg[0x1E] = 0x%02x\n",
				i2c_smbus_read_byte(taos_data->client));
	} else {
		i2c_smbus_write_byte(taos_data->client,
				TAOS_TRITON_CMD_REG | taos_data->g_reg_addr);
		return sprintf(buf, "reg[0x%02x] = 0x%02x", taos_data->g_reg_addr,
					i2c_smbus_read_byte(taos_data->client));
	}

	return strlen(buf);
}

static ssize_t attr_reg_data_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	if (100 == taos_data->g_reg_addr) {
		SENSOR_LOG_ERROR("reg addr error\n");
	} else {
		if (i2c_smbus_write_byte_data(taos_data->client,
				TAOS_TRITON_CMD_REG | taos_data->g_reg_addr, val) < 0)
			SENSOR_LOG_ERROR("failed to write 0x%02x reg\n",
					taos_data->g_reg_addr);
	}

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_verify_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d", taos_data->prox_offset_cal_verify);
}

static ssize_t attr_prox_offset_cal_verify_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_data->prox_offset_cal_verify = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_verify_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%d", taos_data->prox_calibrate_verify);
}

static ssize_t attr_prox_calibrate_verify_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	taos_data->prox_calibrate_verify = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

/* als attributes */
static ssize_t attr_als_light_check_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "%s(0x%x)", taos_data->chip_name, TMD2772_DEVICE_ID);
}

static ssize_t attr_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "als_on is %s\n",
			taos_data->als_on ? "true" : "false");
}

static ssize_t attr_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	bool value = false;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (strtobool(buf, &value))
		return -EINVAL;

	mutex_lock(&taos_data->lock);
	if (value)
		taos_sensors_als_poll_on(taos_data);
	else
		taos_sensors_als_poll_off(taos_data);

	mutex_unlock(&taos_data->lock);
	SENSOR_LOG_INFO("exit\n");

	return size;
}

static ssize_t attr_als_gain_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "als gain is x%d\n",
			als_gain_table[taos_data->taos_cfg->als_gain]);
}

static ssize_t attr_als_gain_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (val > 4 || val <= 0) {
		SENSOR_LOG_ERROR("input error, valid: 1~4");
		return -EINVAL;
	}

	val = val - 1;
	taos_cfg->als_gain = val;
	taos_cfg->prox_gain = (taos_cfg->prox_gain & 0xFC) | val;

	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | 0x0F, taos_cfg->prox_gain) < 0)
		SENSOR_LOG_ERROR("failed to write the prox_led_strength reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_als_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "flag_als_debug is %s\n",
			taos_data->flag_als_debug ? "true" : "false");
}

static ssize_t attr_als_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	if (val)
		taos_data->flag_als_debug = true;
	else
		taos_data->flag_als_debug = false;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_als_adc_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "als_adc_time is 2.73 * %d ms\n",
			256 - taos_data->taos_cfg->als_adc_time);
}

static ssize_t attr_als_adc_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned int val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	taos_cfg->als_adc_time = 256 - val;
	if (i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
		taos_cfg->als_adc_time) < 0)
		SENSOR_LOG_ERROR("failed to write als_adc_time reg\n");

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_scale_factor_als_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "als_scale_factor is %d\n",
			taos_data->taos_cfg->als_scale_factor);
}

static ssize_t attr_scale_factor_als_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val <= 0) {
		SENSOR_LOG_ERROR("input error, valid: > 0\n");
		return -EINVAL;
	}

	taos_data->taos_cfg->als_scale_factor = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_scale_factor_prox_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "prox_scale_factor is %d\n",
			taos_data->taos_cfg->prox_scale_factor);
}

static ssize_t attr_scale_factor_prox_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val <= 0) {
		SENSOR_LOG_ERROR("input error, valid: > 0\n");
		return -EINVAL;
	}

	taos_data->taos_cfg->prox_scale_factor = val;

	SENSOR_LOG_INFO("exit\n");
	return size;
}

static ssize_t attr_als_poll_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos_data = dev_get_drvdata(dev);
	return sprintf(buf, "als_poll_time = %d\n",
			taos_data->taos_cfg->als_poll_delay);
}

static ssize_t attr_als_poll_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long time = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &time))
		return -EINVAL;

	taos_data->taos_cfg->als_poll_delay = time;
	return size;
}

static struct device_attribute attrs_prox[] = {
	__ATTR(chip_name, 0444,
			attr_chip_name_show,
			NULL),
	__ATTR(prox_check, 0444,
			attr_prox_check_show,
			NULL),
	__ATTR(enable, 0644,
			attr_prox_enable_show,
			attr_prox_enable_store),
	__ATTR(prox_init, 0200,
			NULL,
			attr_prox_init_store),
	__ATTR(prox_led_pluse_cnt, 0644,
			attr_prox_led_pluse_cnt_show,
			attr_prox_led_pluse_cnt_store),
	__ATTR(prox_adc_time, 0644,
			attr_prox_adc_time_show,
			attr_prox_adc_time_store),
	__ATTR(prox_led_strength_level, 0644,
			attr_prox_led_strength_show,
			attr_prox_led_strength_store),
	__ATTR(prox_debug_delay, 0644,
			attr_prox_debug_delay_show,
			attr_prox_debug_delay_store),
	__ATTR(prox_calibrate, 0200,
			NULL,
			attr_set_prox_calibrate),
	__ATTR(prox_threshold_high, 0644,
			attr_prox_threshold_high_show,
			attr_prox_threshold_high_store),
	__ATTR(prox_threshold_low, 0644,
			attr_prox_threshold_low_show,
			attr_prox_threshold_low_store),
	__ATTR(prox_offset, 0644,
			attr_prox_offset_show,
			attr_prox_offset_store),
	__ATTR(prox_calibrate_result, 0444,
			attr_prox_calibrate_result_show,
			NULL),
	__ATTR(prox_thres_param_high, 0644,
			attr_prox_cali_high_param_show,
			attr_prox_cali_high_param_store),
	__ATTR(prox_thres_param_low, 0644,
			attr_prox_cali_low_param_show,
			attr_prox_cali_low_param_store),
	__ATTR(prox_thres, 0644,
			attr_prox_thres_show,
			attr_prox_thres_store),
	__ATTR(prox_debug, 0644,
			attr_prox_debug_show,
			attr_prox_debug_store),
	__ATTR(prox_calibrate_start, 0644,
			attr_prox_calibrate_start_show,
			attr_prox_debug_store),
	__ATTR(prox_thres_max, 0444,
			attr_prox_thres_hi_max_show,
			NULL),
	__ATTR(prox_thres_min, 0444,
			attr_prox_thres_lo_min_show,
			NULL),
	__ATTR(prox_data_max, 0444,
			attr_prox_data_max_show,
			NULL),
	__ATTR(prox_manual_calibrate_threshold, 0444,
			attr_prox_manual_cali_threshold_show,
			NULL),
	__ATTR(prox_wakelock, 0644,
			attr_prox_prox_wakelock_show,
			attr_prox_prox_wakelock_store),
	__ATTR(irq_status, 0644,
			attr_irq_show,
			attr_irq_store),
	__ATTR(wait_time, 0644,
			attr_wait_time_show,
			attr_wait_time_store),
	__ATTR(prox_offset_cal_start, 0644,
			attr_prox_offset_cal_start_show,
			attr_prox_debug_store),
	__ATTR(prox_offset_cal,	0644,
			attr_prox_offset_show,
			attr_prox_offset_cal_store),
	__ATTR(prox_offset_cal_result, 0444,
			attr_prox_offset_cal_result_show,
			NULL),
	__ATTR(prox_data_safe_range_max, 0444,
			attr_prox_data_safa_range_max_show,
			NULL),
	__ATTR(prox_data_safe_range_min, 0444,
			attr_prox_data_safa_range_min_show,
			NULL),
	__ATTR(reg_addr, 0644,
			attr_reg_addr_show,
			attr_reg_addr_store),
	__ATTR(reg_data, 0644,
			attr_reg_data_show,
			attr_reg_data_store),
	__ATTR(prox_offset_cal_verify, 0644,
			attr_prox_offset_cal_verify_show,
			attr_prox_offset_cal_verify_store),
	__ATTR(prox_calibrate_verify, 0644,
			attr_prox_calibrate_verify_show,
			attr_prox_calibrate_verify_store),
};

static struct device_attribute attrs_light[] = {
	__ATTR(light_check, 0444,
			attr_als_light_check_show,
			NULL),
	__ATTR(enable, 0644,
			attr_als_enable_show,
			attr_als_enable_store),
	__ATTR(light_gain, 0644,
			attr_als_gain_show,
			attr_als_gain_store),
	__ATTR(light_debug, 0644,
			attr_als_debug_show,
			attr_als_debug_store),
	__ATTR(light_adc_time, 0644,
			attr_als_adc_time_show,
			attr_als_adc_time_store),
	__ATTR(scale_factor_als, 0644,
			attr_scale_factor_als_show,
			attr_scale_factor_als_store),
	__ATTR(scale_factor_prox, 0644,
			attr_scale_factor_prox_show,
			attr_scale_factor_prox_store),
	__ATTR(delay, 0644,
			attr_als_poll_time_show,
			attr_als_poll_time_store),
};

static int create_sysfs_interfaces_prox(struct device *dev)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(attrs_prox); i++) {
		ret = device_create_file(dev, attrs_prox + i);
		if (ret)
			goto error;
	}
	return ret;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attrs_prox + i);

	SENSOR_LOG_ERROR("Unable to create interface\n");
	return ret;
}

static void remove_sysfs_interfaces_prox(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_prox); i++)
		device_remove_file(dev, attrs_prox + i);
}

static int create_sysfs_interfaces_light(struct device *dev)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(attrs_light); i++) {
		ret = device_create_file(dev, attrs_light + i);
		if (ret)
			goto error;
	}
	return ret;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attrs_light + i);

	SENSOR_LOG_ERROR("Unable to create interface\n");
	return ret;
}

static void remove_sysfs_interfaces_light(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_light); i++)
		device_remove_file(dev, attrs_light + i);
}

static int taos_write_cal_file(struct taos_data *taos_data, char *file_path, unsigned int value)
{
	struct file *file_p = NULL;
	char write_buf[10] = {0};
	mm_segment_t old_fs;
	int ret = -1;

	if (!file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		goto error;
	}

	memset(write_buf, 0, sizeof(write_buf));
	sprintf(write_buf, "%d\n", value);

	file_p = filp_open(file_path, O_CREAT | O_RDWR , 0665);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n", file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_write(file_p, (char *)write_buf, sizeof(write_buf), &file_p->f_pos);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write file <%s>\n", file_path);
		goto error;
	}

	set_fs(old_fs);
	filp_close(file_p, NULL);
	return 0;

error:
	return ret;
}

static int taos_read_cal_value(struct taos_data *taos_data, char *file_path, int *value)
{
	struct file *file_p = NULL;
	int ret = -1;
	mm_segment_t old_fs;
	char read_buf[32] = { 0 };
	unsigned short read_value = 0;

	if (!file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		goto error;
	}

	memset(read_buf, 0, 32);

	file_p = filp_open(file_path, O_RDONLY , 0);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("failed to open file <%s>\n", file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_read(file_p, (char *)read_buf, 16, &file_p->f_pos);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to read file <%s>\n", file_path);
		goto error;
	}

	set_fs(old_fs);
	filp_close(file_p, NULL);

	ret = kstrtou16(read_buf, 10, &read_value);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to kstrtou16 %s\n", read_buf);
		goto error;
	}

	*value = read_value;

error:
	return ret;
}

static int taos_prox_on(struct taos_data *taos_data)
{
	int prox_sum = 0;
	int prox_mean = 0;
	int prox_max = 0;
	int ret = 0;
	u8 reg_cntrl = 0;
	int i = 0;
	int j = 0;
	struct taos_prox_info prox_info = {0};
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	taos_data->als_poll_time_mul = 2;

	if (taos_data->als_on) {
		ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
			ALS_ADC_TIME_PROX_ON);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
			return ret;
		}
	} else {
		ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
			0xFF);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
			return ret;
		}
	}

	taos_update_sat_als(taos_data);

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_TIME,
		taos_cfg->prox_adc_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_TIME reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_WAIT_TIME,
		taos_cfg->wait_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_WAIT_TIME reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_INTERRUPT,
		taos_cfg->prox_intr_filter);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_INTERRUPT reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_CFG,
		taos_cfg->prox_config);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_CFG reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_COUNT,
		taos_cfg->prox_pulse_cnt);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_COUNT reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN,
		taos_cfg->prox_gain);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_cfg->prox_config_offset);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");
		return ret;
	}

	reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON
		| TAOS_TRITON_CNTL_PROX_INT_ENBL | TAOS_TRITON_CNTL_ADC_ENBL
		| TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
	ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
			reg_cntrl);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
		return ret;
	}

	taos_data->prox_on = true;
	taos_data->pro_ft = true;
	if (taos_data->prox_calibrate_flag) {
		prox_sum = 0;
		prox_max = 0;

		msleep(20);
		for (i = 0, j = 0; i < 5; i++) {
			if (taos_prox_poll(taos_data, &prox_info) < 0) {
				SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
			} else {
				j++;
				prox_sum += prox_info.prox_data;
				if (prox_info.prox_data > prox_max)
					prox_max = prox_info.prox_data;
				msleep(20);
			}
		}

		if (j == 0) {
			ret = -1;
			goto error;
		}
		prox_mean = prox_sum / j;

		taos_cfg->prox_threshold_hi = ((((prox_max - prox_mean) *
						taos_data->prox_calibrate_hi_param) + 50) / 100)
						+ prox_mean + 110;
		taos_cfg->prox_threshold_lo = ((((prox_max - prox_mean) *
						taos_data->prox_calibrate_lo_param) + 50) / 100)
						+ prox_mean + 35;

		SENSOR_LOG_INFO("prox_threshold_hi = %d\n", taos_cfg->prox_threshold_hi);
		SENSOR_LOG_INFO("prox_threshold_lo = %d\n", taos_cfg->prox_threshold_lo);

		if (prox_mean > 800 || taos_cfg->prox_threshold_hi > 1000
				|| taos_cfg->prox_threshold_lo > 900) {
			taos_cfg->prox_threshold_hi = 800;
			taos_cfg->prox_threshold_lo = 750;
		}

		if (taos_cfg->prox_threshold_hi < 200) {
			taos_cfg->prox_threshold_hi = 200;
			taos_cfg->prox_threshold_lo = 100;
		}

		input_report_rel(taos_data->p_idev, REL_Y, taos_cfg->prox_threshold_hi);
		input_report_rel(taos_data->p_idev, REL_Z, taos_cfg->prox_threshold_lo);
		input_sync(taos_data->p_idev);
	}

error:
	taos_prox_threshold_set(taos_data);
	taos_irq_ops(taos_data, true, true);
	return ret;
}

static int taos_prox_off(struct taos_data *taos_data)
{
	int ret = 0;

	if (taos_data->proximity_wakelock.locked) {
		hrtimer_cancel(&taos_data->prox_unwakelock_timer);
		taos_wakelock_ops(&(taos_data->proximity_wakelock), false);
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
		0x00);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL\n");
		return ret;
	}

	taos_data->prox_on = false;
	taos_data->als_poll_time_mul = 1;

	if (taos_data->als_on)
		taos_sensors_als_poll_on(taos_data);

	if (taos_data->irq_enabled)
		taos_irq_ops(taos_data, false, true);

	return ret;
}

static int tmd2772_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct taos_data *taos_data = container_of(sensors_cdev,
			struct taos_data, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	if (enable)
		taos_prox_on(taos_data);
	else
		taos_prox_off(taos_data);

	return 0;
}

static int taos_prox_calibrate(struct taos_data *taos_data)
{
	int ret = 0;
	int prox_sum = 0;
	int prox_mean = 0;
	int prox_max = 0;
	int prox_min = 1024;
	u8 reg_cntrl = 0;
	int i = 0;
	int j = 0;
	struct taos_prox_info prox_info = {0};
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
		taos_cfg->als_adc_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_TIME,
		taos_cfg->prox_adc_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_TIME reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_WAIT_TIME,
		taos_cfg->wait_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_WAIT_TIME reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_CFG,
		taos_cfg->prox_config);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_CFG reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_COUNT,
		taos_cfg->prox_pulse_cnt);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_COUNT reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_cfg->prox_config_offset);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN,
		taos_cfg->prox_gain);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
		goto error;
	}

	reg_cntrl = 0 | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON
			| TAOS_TRITON_CNTL_ADC_ENBL);
	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
		reg_cntrl);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
		goto error;
	}

	prox_sum = 0;
	prox_max = 0;
	prox_min = 1024;
	msleep(30);

	for (i = 0, j = 0; i < taos_data->prox_calibrate_times; i++) {
		if (taos_prox_poll(taos_data, &prox_info) < 0) {
			SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
		} else {
			j++;
			prox_sum += prox_info.prox_data;
			if (prox_info.prox_data > prox_max)
				prox_max = prox_info.prox_data;
			if (prox_info.prox_data < prox_min)
				prox_min = prox_info.prox_data;

			msleep(30);
		}
	}

	if (j == 0)
		goto error;

	prox_mean = prox_sum / j;
	taos_cfg->prox_threshold_hi = ((((prox_max - prox_mean) *
					taos_data->prox_calibrate_hi_param) + 50) / 100)
					+ prox_mean + 120;
	taos_cfg->prox_threshold_lo = ((((prox_max - prox_mean) *
					taos_data->prox_calibrate_lo_param) + 50) / 100)
					+ prox_mean + 40;

	if ((prox_mean > 700) || (taos_cfg->prox_threshold_hi > 1000)
			|| (taos_cfg->prox_threshold_lo > 900)) {
		taos_cfg->prox_threshold_hi = PROX_DEFAULT_THRESHOLD_HIGH;
		taos_cfg->prox_threshold_lo = PROX_DEFAULT_THRESHOLD_LOW;
		taos_cfg->prox_config_offset = 0x0;
	}

	SENSOR_LOG_INFO("prox_threshold_hi = %d\n", taos_cfg->prox_threshold_hi);
	SENSOR_LOG_INFO("rox_threshold_lo = %d\n", taos_cfg->prox_threshold_lo);
	SENSOR_LOG_INFO("prox_mean = %d\n", prox_mean);
	SENSOR_LOG_INFO("prox_max = %d\n", prox_max);
	SENSOR_LOG_INFO("prox_min = %d\n", prox_min);

	for (i = 0; i < sizeof(taos_triton_reg_init); i++) {
		if (i != 11) {
			ret = i2c_smbus_write_byte_data(taos_data->client,
					TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + i),
					taos_triton_reg_init[i]);
			if (ret < 0) {
				SENSOR_LOG_ERROR("failed to write triton_init reg\n");
				goto error;
			}
		 }
	}

	input_report_rel(taos_data->p_idev, REL_Y, taos_cfg->prox_threshold_hi);
	input_report_rel(taos_data->p_idev, REL_Z, taos_cfg->prox_threshold_lo);
	input_sync(taos_data->p_idev);

	return 0;
error:
	return ret;
}

static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable)
{
	if (enable == wakelock->locked)
		return;

	if (enable)
		wake_lock(&wakelock->lock);
	else
		wake_unlock(&wakelock->lock);

	wakelock->locked = enable;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	struct taos_data *taos_data = dev_id;

	SENSOR_LOG_DEBUG("enter\n");

	taos_data->irq_work_status = true;
	taos_irq_ops(taos_data, false, false);
	taos_wakelock_ops(&(taos_data->proximity_wakelock), true);
	if (!queue_work(taos_data->irq_work_queue, &taos_data->irq_work))
		SENSOR_LOG_ERROR("failed to schedule_work\n");

	SENSOR_LOG_DEBUG("exit\n");
	return IRQ_HANDLED;
}

static void taos_irq_work_func(struct work_struct *work)
{
	int retry_times = 0;
	int ret = 0;
	struct taos_data *taos_data;

	taos_data = container_of(work, struct taos_data, irq_work);

	SENSOR_LOG_INFO("enter\n");

	mutex_lock(&taos_data->lock);

	if (taos_data->wakeup_from_sleep) {
		SENSOR_LOG_INFO("wakeup_from_sleep is true\n");
		msleep(50);
		taos_data->wakeup_from_sleep = false;
	}

	for (retry_times = 0; retry_times <= 50; retry_times++) {
		ret = taos_get_data(taos_data);
		if (ret >= 0)
			break;
		msleep(20);
	}
	taos_interrupts_clear(taos_data);

	hrtimer_cancel(&taos_data->prox_unwakelock_timer);
	taos_data->irq_work_status = false;
	hrtimer_start(&taos_data->prox_unwakelock_timer,
			ktime_set(3, 0), HRTIMER_MODE_REL);

	taos_irq_ops(taos_data, true, true);
	mutex_unlock(&taos_data->lock);
	SENSOR_LOG_INFO("exit\n");
}

static int taos_get_data(struct taos_data *taos_data)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to read TAOS_TRITON_STATUS\n");
		return ret;
	}
	return taos_prox_threshold_set(taos_data);
}

static int taos_interrupts_clear(struct taos_data *taos_data)
{
	int ret = 0;
	ret = i2c_smbus_write_byte(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | 0x07);
	if (ret < 0)
		SENSOR_LOG_ERROR("failed to clear interrupts\n");

	return ret;
}

static int taos_prox_poll(struct taos_data *taos_data, struct taos_prox_info *prxp)
{
	int i = 0, ret = 0;
	u8 chdata[6] = {0};

	for (i = 0; i < 6; i++) {
		chdata[i] = i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_AUTO |
			(TAOS_TRITON_ALS_CHAN0LO + i));
	}

	prxp->prox_clear = chdata[1];
	prxp->prox_clear <<= 8;
	prxp->prox_clear |= chdata[0];

	if (prxp->prox_clear > taos_data->als_saturation_value) {
		SENSOR_LOG_ERROR("prox_clear %u > als_saturation_value %u\n",
				prxp->prox_clear, taos_data->als_saturation_value);
		return -ENODATA;
	}
	prxp->prox_data = chdata[5];
	prxp->prox_data <<= 8;
	prxp->prox_data |= chdata[4];

	return ret;
}

static void taos_prox_calibrate_work_func(struct work_struct *work)
{
	struct taos_data *taos_data;
	taos_data = container_of((struct delayed_work *)work,
				struct taos_data, prox_calibrate_work);

	taos_prox_calibrate(taos_data);
}

static int taos_prox_offset_cal_prepare(struct taos_data *taos_data)
{
	int ret = 0;

	taos_data->taos_cfg->prox_config_offset = 0;
	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_data->taos_cfg->prox_config_offset);
	if (ret < 0)
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");

	return ret;
}

static int taos_prox_offset_calculate(struct taos_data *taos_data, int data, int target)
{
	int offset = 0;
	int prox_pulse_cnt = taos_data->taos_cfg->prox_pulse_cnt;

	if (data > target)
		offset = (data - target) * 8 / (5 * prox_pulse_cnt);
	else
		offset = (target - data) * 8 / (5 * prox_pulse_cnt) + 128;

	return offset;
}

static int tmd2772_prox_uncover_data_get(struct taos_data *taos_data)
{
	u8 i = 0;
	int j = 0;
	int prox_sum = 0;
	int ret = 0;
	static struct taos_prox_info prox_info_temp;

	msleep(100);

	for (i = 0, j = 0; i < PROX_OFFSET_CAL_BUFFER_SIZE / 3; i++) {
		if (taos_prox_poll(taos_data, &prox_info_temp) < 0) {
			SENSOR_LOG_ERROR("failed to tmd2772_prox_read_data\n");
		} else {
			j++;
			prox_sum += prox_info_temp.prox_data;
		}
		msleep(10);
	}

	if (j == 0) {
		ret = -1;
		goto error;
	}

	taos_data->prox_uncover_data = prox_sum / j;

	taos_data->prox_thres_hi_min = taos_data->prox_uncover_data +
				PROX_THRESHOLD_SAFE_DISTANCE;
	taos_data->prox_thres_hi_max = taos_data->prox_thres_hi_min +
				PROX_THRESHOLD_DISTANCE / 2;
	taos_data->prox_thres_hi_max = (taos_data->prox_thres_hi_max >
				PROX_THRESHOLD_HIGH_MAX) ?
				PROX_THRESHOLD_HIGH_MAX :
				taos_data->prox_thres_hi_max;
	taos_data->prox_thres_lo_min = taos_data->prox_uncover_data +
				PROX_THRESHOLD_DISTANCE;
	taos_data->prox_thres_lo_max = taos_data->prox_uncover_data +
				PROX_THRESHOLD_DISTANCE * 2;
	SENSOR_LOG_INFO("prox_uncover_data = %d\n",
				taos_data->prox_uncover_data);
	SENSOR_LOG_INFO("prox_thres_hi range is [%d--%d]\n",
				taos_data->prox_thres_hi_min,
				taos_data->prox_thres_hi_max);
	SENSOR_LOG_INFO("prox_thres_lo range is [%d--%d]\n",
				taos_data->prox_thres_lo_min,
				taos_data->prox_thres_lo_max);

	taos_write_cal_file(taos_data, PATH_PROX_UNCOVER_DATA, taos_data->prox_uncover_data);

	return 0;

error:
	return ret;
}

static int taos_prox_offset_cal_process(struct taos_data *taos_data)
{
	int ret = 0;
	int prox_sum = 0;
	int prox_mean = 0;
	int i = 0;
	int j = 0;
	u8 reg_cntrl = 0;
	struct taos_prox_info prox_info = {0};
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
		taos_cfg->als_adc_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write als_adc_time reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_TIME,
		taos_cfg->prox_adc_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_adc_time reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_WAIT_TIME,
		taos_cfg->wait_time);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write wait_time reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_CFG,
		taos_cfg->prox_config);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_config reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_COUNT,
		taos_cfg->prox_pulse_cnt);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_pulse_cnt reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_cfg->prox_config_offset);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_config_offset reg\n");
		goto error;
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN,
		taos_cfg->prox_gain);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_gain reg\n");
		goto error;
	}

	reg_cntrl = 0 | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON
			| TAOS_TRITON_CNTL_ADC_ENBL);
	ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
			reg_cntrl);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write cntrl reg\n");
		goto error;
	}

	msleep(30);

	for (i = 0, j = 0; i < (PROX_OFFSET_CAL_BUFFER_SIZE); i++) {
		ret = taos_prox_poll(taos_data, &prox_info);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
		} else {
			j++;
			prox_sum += prox_info.prox_data;
			msleep(30);
		}
	}

	if (j == 0)
		goto error;

	prox_mean = prox_sum / j;
	SENSOR_LOG_INFO("prox_mean = %d\n", prox_mean);

	taos_cfg->prox_config_offset =
		taos_prox_offset_calculate(taos_data, prox_mean, PROX_DATA_TARGET);

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET,
		taos_cfg->prox_config_offset);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write prox_config_offset reg\n");
		goto error;
	}

	ret = taos_write_cal_file(taos_data, PATH_PROX_OFFSET, taos_cfg->prox_config_offset);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to taos_write_cal_file\n");
		goto error;
	}

	/* read prox data register after update offset register */
	ret = tmd2772_prox_uncover_data_get(taos_data);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to tmd2772_prox_uncover_data_get\n");
		goto error;
	}

	for (i = 0; i < sizeof(taos_triton_reg_init); i++) {
		if (i != 11) {
			ret = i2c_smbus_write_byte_data(taos_data->client,
			    TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + i),
			    taos_triton_reg_init[i]);
			if (ret < 0) {
				SENSOR_LOG_ERROR("failed to write triton_init reg\n");
				goto error;
			}
		 }
	}
	return 0;
error:
	SENSOR_LOG_ERROR("ERROR\n");
	return ret;
}

static void taos_prox_offset_cal_finish(struct taos_data *taos_data)
{
	if (taos_data->prox_on)
		taos_prox_on(taos_data);
	else
		taos_prox_off(taos_data);
}

static int taos_prox_offset_cal(struct taos_data *taos_data)
{
	int ret = 0;
	taos_data->prox_offset_cal_result = false;

	ret = taos_prox_offset_cal_prepare(taos_data);
	if (ret < 0)
		goto error;

	msleep(50);

	ret = taos_prox_offset_cal_process(taos_data);
	if (ret == 0)
		taos_data->prox_offset_cal_result = true;

	taos_prox_offset_cal_finish(taos_data);

error:
	return ret;
}

static void taos_prox_offset_cal_work_func(struct work_struct *work)
{
	struct taos_data *taos_data;
	taos_data = container_of((struct delayed_work *)work,
				struct taos_data, prox_offset_cal_work);

	taos_prox_offset_cal(taos_data);
}

static enum hrtimer_restart taos_prox_unwakelock_work_func(struct hrtimer *timer)
{
	struct taos_data *taos_data =
		container_of(timer, struct taos_data, prox_unwakelock_timer);

	if (false == taos_data->irq_work_status)
		taos_wakelock_ops(&(taos_data->proximity_wakelock), false);

	return HRTIMER_NORESTART;
}

static int taos_prox_threshold_set(struct taos_data *taos_data)
{
	int i = 0;
	int ret = 0;
	u8 chdata[6] = {0};
	u16 proxdata = 0;
	int cleardata = 0;
	char pro_buf[4] = {0};
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	for (i = 0; i < 6; i++)
		chdata[i] = (i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW |
			(TAOS_TRITON_ALS_CHAN0LO + i)));

	cleardata = chdata[0] + (chdata[1] << 8);
	proxdata = chdata[4] + (chdata[5] << 8);

	if (taos_data->pro_ft || taos_data->flag_prox_debug) {
		pro_buf[0] = 0xFF;
		pro_buf[1] = 0xFF;
		pro_buf[2] = 0xFF;
		pro_buf[3] = 0xFF;

		for (i = 0; i < 4; i++) {
			ret = i2c_smbus_write_byte_data(taos_data->client,
					(TAOS_TRITON_CMD_REG | 0x08) + i,
					pro_buf[i]);
			if (ret < 0) {
				SENSOR_LOG_ERROR("failed to access reg\n");
				return ret;
			}
		}

		if (taos_data->pro_ft)
			SENSOR_LOG_INFO("init the prox threshold");

		if (taos_data->flag_prox_debug) {
			msleep(taos_data->prox_debug_delay_time);
			input_report_rel(taos_data->p_idev, REL_MISC,
					proxdata > 0 ? proxdata : 1);
		}
		taos_data->pro_ft = false;
	} else {
		if (proxdata < taos_cfg->prox_threshold_lo) {/* FAR */
			pro_buf[0] = 0x0;
			pro_buf[1] = 0x0;
			pro_buf[2] = taos_cfg->prox_threshold_hi & 0x0FF;
			pro_buf[3] = taos_cfg->prox_threshold_hi >> 8;

			SENSOR_LOG_INFO("Far!!! proxdata = %d\n", proxdata);
			input_report_rel(taos_data->p_idev, REL_X,
					proxdata > 0 ? proxdata : 1);
		} else {
			if (proxdata > taos_cfg->prox_threshold_hi) {/* NEAR */
				if (cleardata > taos_data->als_saturation_value) {
					msleep(100);
					return -ENODATA;
				}

				pro_buf[0] = taos_cfg->prox_threshold_lo & 0x0ff;
				pro_buf[1] = taos_cfg->prox_threshold_lo >> 8;
				pro_buf[2] = 0xFF;
				pro_buf[3] = 0xFF;

				SENSOR_LOG_INFO("Near!!! proxdata = %d\n", proxdata);
				input_report_rel(taos_data->p_idev, REL_X, proxdata);
			} else {
				if ((taos_cfg->prox_threshold_hi - proxdata) >
						(proxdata - taos_cfg->prox_threshold_lo)) {/* FAR */
					pro_buf[0] = 0x0;
					pro_buf[1] = 0x0;
					pro_buf[2] = taos_cfg->prox_threshold_hi & 0xFF;
					pro_buf[3] = taos_cfg->prox_threshold_hi >> 8;

					SENSOR_LOG_INFO("Far!!! proxdata = %d\n", proxdata);
					input_report_rel(taos_data->p_idev, REL_X,
							taos_cfg->prox_threshold_lo - 1);
				} else {/* NEAR */
					if (cleardata > taos_data->als_saturation_value) {
						msleep(100);
						return -ENODATA;
					}

					pro_buf[0] = taos_cfg->prox_threshold_lo & 0xFF;
					pro_buf[1] = taos_cfg->prox_threshold_lo >> 8;
					pro_buf[2] = 0xFF;
					pro_buf[3] = 0xFF;

					SENSOR_LOG_INFO("Near!!! proxdata = %d\n", proxdata);
					input_report_rel(taos_data->p_idev, REL_X,
							taos_cfg->prox_threshold_hi + 1);
				}
			}
		}
	}

	input_sync(taos_data->p_idev);

	for (i = 0; i < 4; i++) {
		ret = i2c_smbus_write_byte_data(taos_data->client,
				(TAOS_TRITON_CMD_REG | 0x08) + i,
				pro_buf[i]);
		if (ret < 0) {
			SENSOR_LOG_ERROR("write byte failed in taos prox threshold set\n");
			return ret;
		}
	}

	return ret;
}

/* als function implementation */
static void taos_update_sat_als(struct taos_data *taos_data)
{
	u8 reg_val = 0;
	int ret = 0;
	u16 sat_als = 0;

	ret = i2c_smbus_write_byte(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME\n");
		return;
	}

	reg_val = i2c_smbus_read_byte(taos_data->client);

	sat_als = (256 - reg_val) << 10;
	taos_data->als_saturation_value = sat_als * 80 / 100;
}

static int taos_sensors_als_poll_on(struct taos_data *taos_data)
{
	int ret = 0;
	int i = 0;
	u8 reg_val = 0;
	int reg_cntrl = 0;
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	for (i = 0; i < ALS_FILTER_DEPTH; i++)
		lux_history[i] = -ENODATA;

	if (taos_data->prox_on) {
		ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
			ALS_ADC_TIME_PROX_ON);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
			return ret;
		}
	} else {
		ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME,
			taos_cfg->als_adc_time);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
			return ret;
		}
	}

	reg_val = i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN);
	reg_val = reg_val & 0xFC;
	reg_val = reg_val | (taos_cfg->als_gain & 0x03);
	ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN,
			reg_val);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
		return ret;
	}

	reg_cntrl = i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);

	reg_cntrl |= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON);
	ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
			reg_cntrl);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
		return ret;
	}

	schedule_delayed_work(&taos_data->als_poll_work, msecs_to_jiffies(200));

	taos_data->als_on = true;

	taos_update_sat_als(taos_data);

	return ret;
}

static int taos_sensors_als_poll_off(struct taos_data *taos_data)
{
	int ret = 0, i = 0;
	u8 reg_val = 0;

	for (i = 0; i < ALS_FILTER_DEPTH; i++)
		lux_history[i] = -ENODATA;

	reg_val = i2c_smbus_read_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
	if ((reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL) == 0x00
			&& (0 == taos_data->prox_on)) {
		reg_val = 0x00;
		ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, reg_val);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
			return ret;
		}
	}

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME, 0XFF);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
		return ret;
	}

	taos_data->als_on = false;
	cancel_delayed_work_sync(&taos_data->als_poll_work);
	taos_update_sat_als(taos_data);

	return ret;
}

static int tmd2772_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct taos_data *taos_data = container_of(sensors_cdev,
			struct taos_data, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	if (enable)
		taos_sensors_als_poll_on(taos_data);
	else
		taos_sensors_als_poll_off(taos_data);

	return 0;
}

static int tmd2772_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct taos_data *taos_data = container_of(sensors_cdev,
			struct taos_data, als_cdev);

	taos_data->taos_cfg->als_poll_delay = delay_msec;
	return 0;
}

static int taos_get_lux(struct taos_data *taos_data, int *lux_value)
{
	int raw_clear = 0;
	int raw_ir = 0;
	int raw_lux = 0;
	u32 lux = 0, ratio = 0;
	u8 dev_gain = 0;
	u16 Tint = 0;
	struct lux_data *p = NULL;
	int ret = 0;
	u8 chdata[4] = {0};
	int tmp = 0;
	int i = 0;
	int tmp_gain = 1;
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;
	struct time_scale_factor tritontime = {1, 0, 0};

	for (i = 0; i < 4; i++) {
		ret = i2c_smbus_write_byte(taos_data->client,
				TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i));
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to read chan0/1/lo/hi reg\n");
			return ret;
		}
		chdata[i] = i2c_smbus_read_byte(taos_data->client);
	}

	tmp = (ALS_TIME_PARAM + 25) / 50;
	tritontime.numerator = 1;
	tritontime.denominator = tmp;
	tmp = 300 * ALS_TIME_PARAM;
	if (tmp > 65535)
		tmp = 65535;

	tritontime.saturation = tmp;

	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir = chdata[3];
	raw_ir <<= 8;
	raw_ir |= chdata[2];

	raw_clear *= ((taos_cfg->als_scale_factor) * tmp_gain);
	raw_ir *= (taos_cfg->prox_scale_factor);

	if (raw_ir > raw_clear) {
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}

	dev_gain = als_gain_table[taos_cfg->als_gain & 0x3];

	if (raw_clear >= tritontime.saturation) {
		*lux_value = ALS_MAX_LUX;
		return 0;
	}

	if (raw_ir >= tritontime.saturation) {
		*lux_value = ALS_MAX_LUX;
		return 0;
	}

	if (raw_clear == 0) {
		*lux_value = 0;
		return 0;
	}

	if (dev_gain == 0 || dev_gain > 127)
		return -EINVAL;

	if (tritontime.denominator == 0)
		return -EINVAL;

	ratio = (raw_ir << 15) / raw_clear;

	for (p = lux_table; p->ratio && p->ratio < ratio; p++)
		;

#ifdef WORK_UES_POLL_MODE
	if (!p->ratio) {
		if (lux_history[0] < 0) {
			*lux_value = 0;
			return 0;
		}
		*lux_value = lux_history[0];
		return 0;
	}
#endif

	Tint = ALS_TIME_PARAM;

	raw_clear = ((raw_clear * 400 + (dev_gain >> 1)) / dev_gain
						+ (Tint >> 1)) / Tint;
	raw_ir = ((raw_ir * 400 + (dev_gain >> 1)) / dev_gain
						+ (Tint >> 1)) / Tint;
	lux = ((raw_clear * (p->clear)) - (raw_ir * (p->ir)));
	lux = (lux + 32000) / 64000;
	if (lux > ALS_MAX_LUX)
		lux = ALS_MAX_LUX;

	*lux_value = lux;
	return 0;
}

static int taos_lux_filter(int lux)
{
	static u8 middle[] = {1, 0, 2, 0, 0, 2, 0, 1};
	int index = 0;

	lux_history[2] = lux_history[1];
	lux_history[1] = lux_history[0];
	lux_history[0] = lux;

	if (lux_history[2] < 0) {
		if (lux_history[1] > 0)
			return lux_history[1];
		return lux_history[0];
	}

	index = 0;
	if (lux_history[0] > lux_history[1])
		index += 4;
	if (lux_history[1] > lux_history[2])
		index += 2;
	if (lux_history[0] > lux_history[2])
		index++;

	return lux_history[middle[index]];
}

static int taos_als_gain_set(struct taos_data *taos_data, unsigned als_gain)
{
	int ret = 0;
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;

	taos_cfg->prox_gain = (taos_cfg->prox_gain & 0xFC) | als_gain;
	taos_cfg->als_gain = taos_cfg->prox_gain & 0x03;

	ret = i2c_smbus_write_byte_data(taos_data->client,
		TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN,
		taos_cfg->prox_gain);
	if (ret < 0)
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");

	return ret;
}

static int taos_als_get_data(struct taos_data *taos_data)
{
	int ret = 0;
	u8 reg_val = 0;
	int lux_val = 0;
	struct taos_cfg *taos_cfg = taos_data->taos_cfg;


	ret = i2c_smbus_write_byte(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to access TAOS_TRITON_CNTRL reg\n");
		return ret;
	}

	reg_val = i2c_smbus_read_byte(taos_data->client);
	if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
			!= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
		return -ENODATA;

	ret = i2c_smbus_write_byte(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to access TAOS_TRITON_STATUS reg\n");
		return ret;
	}

	reg_val = i2c_smbus_read_byte(taos_data->client);
	if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
		return -ENODATA;

	ret = taos_get_lux(taos_data, &lux_val);
	if (ret < 0)
		SENSOR_LOG_ERROR("failed to taos_get_lux, ret = %d\n", ret);

	if (lux_val < ALS_GAIN_DIVIDE && taos_cfg->als_gain != ALS_GAIN_8X) {
		taos_als_gain_set(taos_data, ALS_GAIN_8X);
	} else {
		if (lux_val > ALS_GAIN_DIVIDE && taos_cfg->als_gain != ALS_GAIN_1X)
			taos_als_gain_set(taos_data, ALS_GAIN_1X);
	}

	ret = i2c_smbus_write_byte(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to access TAOS_TRITON_ALS_TIME\n");
		return ret;
	}

	reg_val = i2c_smbus_read_byte(taos_data->client);
	if (reg_val != ALS_ADC_TIME_DEFAULT)
		lux_val = (lux_val * (101 - (0xFF - reg_val))) / 20;

	lux_val = taos_lux_filter(lux_val);

	lux_val = lux_val * taos_data->light_percent / 100;
	lux_val = lux_val > 10000 ? 10000 : lux_val;

	input_report_rel(taos_data->a_idev, REL_X, lux_val + 1);
	input_sync(taos_data->a_idev);

	return ret;
}

static void taos_als_poll_work_func(struct work_struct *work)
{
	struct taos_data *taos_data;
	taos_data = container_of((struct delayed_work *)work,
				struct taos_data, als_poll_work);

	taos_als_get_data(taos_data);
	schedule_delayed_work(&taos_data->als_poll_work,
		msecs_to_jiffies(taos_data->als_poll_time_mul
			* taos_data->taos_cfg->als_poll_delay));
}

static void tmd2772_data_init(struct taos_data *taos_data)
{
	taos_data->als_on = false;
	taos_data->prox_on = false;
	taos_data->als_poll_time_mul = 1;
	taos_data->prox_name = "proximity";
	taos_data->als_name = "light";
	taos_data->chip_name = "tmd2772";
	taos_data->prox_calibrate_result = false;
	taos_data->prox_offset_cal_result = false;
	taos_data->prox_offset_cal_verify = true;
	taos_data->prox_calibrate_verify = true;
	taos_data->prox_thres_hi_max = PROX_THRESHOLD_HIGH_MAX;
	taos_data->prox_thres_hi_min = PROX_THRESHOLD_HIGH_MIN;
	taos_data->prox_calibrate_hi_param = PROX_CAL_HIGH_PARAM;
	taos_data->prox_calibrate_lo_param = PROX_CAL_LOW_PARAM;
	taos_data->prox_data_max = PROX_DATA_MAX;
	taos_data->prox_calibrate_times = 10;
	taos_data->prox_calibrate_flag = true;
	taos_data->prox_manual_calibrate_threshold = 0;
	taos_data->prox_uncover_data = 0;
	taos_data->proximity_wakelock.name = "proximity-wakelock";
	taos_data->proximity_wakelock.locked = false;
	taos_data->irq_work_status = false;
	taos_data->irq_enabled = true;
	taos_data->pro_ft = false;
	taos_data->flag_als_debug = false;
	taos_data->flag_prox_debug = false;
}

static int tmd2772_parse_dt(struct taos_data *taos_data)
{
	int rc = 0;
	u32 temp_val = 0;
	struct device_node *np = taos_data->client->dev.of_node;

	/* irq gpio */
	rc = of_get_named_gpio_flags(np, "ams,irq-gpio", 0, NULL);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	taos_data->irq_pin_num = rc;

	rc = of_property_read_u32(np, "ams,pul_cnt", &temp_val);
	if (rc && (rc != -EINVAL)) {
		SENSOR_LOG_ERROR("Unable to read ams,pul_cnt\n");
		return rc;
	}
	taos_data->taos_cfg->prox_pulse_cnt = temp_val;
	SENSOR_LOG_INFO("prox_pulse_cnt is %d\n", taos_data->taos_cfg->prox_pulse_cnt);

	rc = of_property_read_u32(np, "ams,light-percent", &temp_val);
	taos_data->light_percent = (!rc ? temp_val : 100);
	SENSOR_LOG_INFO("light_percent is %d\n", taos_data->light_percent);

	return 0;
}

static int tmd2772_power_init(struct taos_data *taos_data, bool on)
{
	int rc = 0;

	if (!on) {
		if (regulator_count_voltages(taos_data->vdd) > 0)
			regulator_set_voltage(taos_data->vdd, 0,
				TMD2772_VDD_MAX_UV);
		regulator_put(taos_data->vdd);

		if (regulator_count_voltages(taos_data->vio) > 0)
			regulator_set_voltage(taos_data->vio, 0,
				TMD2772_VIO_MAX_UV);
		regulator_put(taos_data->vio);
	} else {
		taos_data->vdd = regulator_get(&taos_data->client->dev, "vdd");
		if (IS_ERR(taos_data->vdd)) {
			rc = PTR_ERR(taos_data->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(taos_data->vdd) > 0) {
			rc = regulator_set_voltage(taos_data->vdd,
				TMD2772_VDD_MIN_UV, TMD2772_VDD_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
					rc);
				goto err_vdd_set;
			}
		}

		taos_data->vio = regulator_get(&taos_data->client->dev, "vio");
		if (IS_ERR(taos_data->vio)) {
			rc = PTR_ERR(taos_data->vio);
				SENSOR_LOG_ERROR("Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(taos_data->vio) > 0) {
			rc = regulator_set_voltage(taos_data->vio,
				TMD2772_VIO_MIN_UV, TMD2772_VIO_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set;
			}
		}

		rc = regulator_enable(taos_data->vdd);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(taos_data->vio);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
	}
	return 0;

err_vio_enable:
	regulator_disable(taos_data->vdd);
err_vdd_enable:
	if (regulator_count_voltages(taos_data->vio) > 0)
		regulator_set_voltage(taos_data->vio, 0, TMD2772_VIO_MAX_UV);
err_vio_set:
	regulator_put(taos_data->vio);
err_vio_get:
	if (regulator_count_voltages(taos_data->vdd) > 0)
		regulator_set_voltage(taos_data->vdd, 0, TMD2772_VDD_MAX_UV);
err_vdd_set:
	regulator_put(taos_data->vdd);
	return rc;
}

static int tmd2772_pinctrl_init(struct taos_data *taos_data)
{
	struct i2c_client *client = taos_data->client;

	taos_data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(taos_data->pinctrl)) {
		SENSOR_LOG_ERROR("Failed to get pinctrl\n");
		return PTR_ERR(taos_data->pinctrl);
	}

	taos_data->pin_default = pinctrl_lookup_state(taos_data->pinctrl, "tmd2772_default");
	if (IS_ERR_OR_NULL(taos_data->pin_default)) {
		SENSOR_LOG_ERROR("Failed to look up default state\n");
		return PTR_ERR(taos_data->pin_default);
	}

	taos_data->pin_sleep = pinctrl_lookup_state(taos_data->pinctrl, "tmd2772_sleep");
	if (IS_ERR_OR_NULL(taos_data->pin_sleep)) {
		SENSOR_LOG_ERROR("Failed to look up sleep state\n");
		return PTR_ERR(taos_data->pin_sleep);
	}

	return 0;
}

static int tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp)
{
	int ret = 0;
	int chip_id = -1;
	u16 sat_als = 0;
	struct taos_data *taos_data;
	struct taos_cfg *taos_cfg;

	SENSOR_LOG_INFO("probe start\n");

	if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		SENSOR_LOG_ERROR("unsupport i2c smbus byte data functions\n");
		ret = -EOPNOTSUPP;
		goto failed;
	}

	taos_data = kzalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos_data) {
		SENSOR_LOG_ERROR("failed to kzalloc for struct taos_data\n");
		ret = -ENOMEM;
		goto failed;
	}

	taos_data->client = clientp;
	i2c_set_clientdata(clientp, taos_data);

	/* initialize pinctrl */
	ret = tmd2772_pinctrl_init(taos_data);
	if (ret) {
		SENSOR_LOG_ERROR("pinctrl init failed.\n");
		goto failed;
	}
	ret = pinctrl_select_state(taos_data->pinctrl, taos_data->pin_default);
	if (ret) {
		SENSOR_LOG_ERROR("Can't select pinctrl state\n");
		goto failed;
	}

	ret = tmd2772_power_init(taos_data, 1);
	if (ret < 0)
		goto power_init_failed;

	chip_id = i2c_smbus_read_byte_data(clientp,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CHIPID);
	SENSOR_LOG_INFO("chip_id = 0x%x TMD27713 = 0x30, TMD27723 = 0x39\n", chip_id);
	if (chip_id != TMD2772_DEVICE_ID) {
		SENSOR_LOG_ERROR("chip id does not match TMD27723(0x39)\n");
		ret = -ENODEV;
		goto read_chipid_failed;
	}

	INIT_WORK(&(taos_data->irq_work), taos_irq_work_func);

	mutex_init(&(taos_data->lock));
	wake_lock_init(&taos_data->proximity_wakelock.lock,
			WAKE_LOCK_SUSPEND, "proximity-wakelock");

	tmd2772_data_init(taos_data);

	strlcpy(clientp->name, TAOS_DEVICE_NAME, I2C_NAME_SIZE);

	taos_data->taos_cfg = kzalloc(sizeof(struct taos_cfg), GFP_KERNEL);
	if (!taos_data->taos_cfg) {
		SENSOR_LOG_ERROR("failed to kzalloc for struct taos_cfg\n");
		ret = -ENOMEM;
		goto read_chipid_failed;
	}

	ret = tmd2772_parse_dt(taos_data);
	if (ret) {
		SENSOR_LOG_ERROR("parse device tree failed\n");
		goto parse_dt_failed;
	}

	taos_cfg = taos_data->taos_cfg;
	taos_cfg->als_scale_factor = ALS_SCALE_FACTOR_DEFAULT;
	taos_cfg->als_gain = ALS_GAIN_DEFAULT;
	taos_cfg->als_poll_delay = ALS_POLL_DELAY_DEFAULT;

	taos_cfg->als_adc_time = ALS_ADC_TIME_DEFAULT;
	sat_als = (256 - taos_cfg->als_adc_time) << 10;
	taos_data->als_saturation_value = sat_als * 80 / 100;

	taos_cfg->prox_threshold_hi = PROX_DEFAULT_THRESHOLD_HIGH;
	taos_cfg->prox_threshold_lo = PROX_DEFAULT_THRESHOLD_LOW;
	taos_cfg->prox_scale_factor = PROX_SCALE_FACTOR_DEFAULT;
	taos_cfg->prox_adc_time = PROX_ADC_TIME_DEFAULT;
	taos_cfg->prox_intr_filter = PROX_INTR_FILTER_DEFAULT;
	taos_cfg->prox_config = PROX_CONFIG_DEFAULT;
	taos_cfg->prox_gain = PROX_GAIN_DEFAULT;
	taos_cfg->prox_config_offset = PROX_CONFIG_OFFSET_DEFAULT;
	taos_cfg->wait_time = WAIT_TIME_DEFAULT;

	ret = i2c_smbus_write_byte_data(taos_data->client,
			TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL,
			0x00);
	if (ret < 0) {
		SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
		goto parse_dt_failed;
	}

	taos_data->irq_work_queue =
		create_singlethread_workqueue("taos_work_queue");
	if (!taos_data->irq_work_queue) {
		ret = -ENOMEM;
		SENSOR_LOG_ERROR("failed to create irq_work_queue");
		goto parse_dt_failed;
	}

	ret = gpio_request(taos_data->irq_pin_num, "ALS_PS_INT");
	if (ret) {
		gpio_free(taos_data->irq_pin_num);
		ret = gpio_request(taos_data->irq_pin_num, "ALS_PS_INT");
		if (ret)
			goto gpio_request_failed;
	}

	taos_data->client->irq = gpio_to_irq(taos_data->irq_pin_num);
	ret = request_irq(taos_data->client->irq,
			taos_irq_handler,
			IRQF_TRIGGER_FALLING,
			"taos_irq", taos_data);
	if (ret)
		goto request_irq_failed;

	taos_irq_ops(taos_data, false, true);

	INIT_DELAYED_WORK(&taos_data->als_poll_work, taos_als_poll_work_func);
	INIT_DELAYED_WORK(&taos_data->prox_calibrate_work,
			taos_prox_calibrate_work_func);
	INIT_DELAYED_WORK(&taos_data->prox_offset_cal_work,
			taos_prox_offset_cal_work_func);

	hrtimer_init(&taos_data->prox_unwakelock_timer,
			CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos_data->prox_unwakelock_timer.function = taos_prox_unwakelock_work_func;

	taos_data->proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(taos_data->proximity_class)) {
		ret = PTR_ERR(taos_data->proximity_class);
		taos_data->proximity_class = NULL;
		goto request_irq_failed;
	}

	taos_data->light_class = class_create(THIS_MODULE, "light");
	if (IS_ERR(taos_data->light_class)) {
		ret = PTR_ERR(taos_data->light_class);
		taos_data->light_class = NULL;
		goto create_light_class_failed;
	}

	taos_data->proximity_dev = device_create(taos_data->proximity_class,
			NULL, tmd2772_proximity_dev_t, NULL, "proximity");
	if (IS_ERR(taos_data->proximity_dev)) {
		ret = PTR_ERR(taos_data->proximity_dev);
		SENSOR_LOG_ERROR("failed to create device proximity\n");
		goto create_proximity_dev_failed;
	}

	taos_data->light_dev = device_create(taos_data->light_class,
				NULL, tmd2772_light_dev_t, NULL, "light");
	if (IS_ERR(taos_data->light_dev)) {
		ret = PTR_ERR(taos_data->light_dev);
		SENSOR_LOG_ERROR("failed to create device light\n");
		goto create_light_dev_failed;
	}

	taos_data->p_idev = input_allocate_device();
	if (!taos_data->p_idev) {
		SENSOR_LOG_ERROR("failed to create input_dev '%s'\n",
						taos_data->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	taos_data->p_idev->name = taos_data->prox_name;
	taos_data->p_idev->id.bustype = BUS_I2C;
	dev_set_drvdata(&taos_data->p_idev->dev, taos_data);

	ret = input_register_device(taos_data->p_idev);
	if (ret) {
		SENSOR_LOG_ERROR("failed to register input '%s'\n",
						taos_data->prox_name);
		goto input_p_register_failed;
	}

	set_bit(EV_REL, taos_data->p_idev->evbit);
	set_bit(REL_X, taos_data->p_idev->relbit);
	set_bit(REL_Y, taos_data->p_idev->relbit);
	set_bit(REL_Z, taos_data->p_idev->relbit);
	set_bit(REL_MISC, taos_data->p_idev->relbit);

	taos_data->a_idev = input_allocate_device();
	if (!taos_data->a_idev) {
		SENSOR_LOG_ERROR("failed to create input_dev '%s'\n",
						taos_data->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	taos_data->a_idev->name = taos_data->als_name;
	taos_data->a_idev->id.bustype = BUS_I2C;

	set_bit(EV_REL, taos_data->a_idev->evbit);
	set_bit(REL_X, taos_data->a_idev->relbit);
	set_bit(REL_Y, taos_data->a_idev->relbit);

	dev_set_drvdata(&taos_data->a_idev->dev, taos_data);
	ret = input_register_device(taos_data->a_idev);
	if (ret) {
		SENSOR_LOG_ERROR("fail to register input '%s'\n",
						taos_data->prox_name);
		goto input_a_register_failed;
	}

	dev_set_drvdata(taos_data->proximity_dev, taos_data);
	dev_set_drvdata(taos_data->light_dev, taos_data);

	ret = create_sysfs_interfaces_prox(taos_data->proximity_dev);
	if (ret)
		goto create_proximity_sysfs_failed;

	ret = create_sysfs_interfaces_light(taos_data->light_dev);
	if (ret)
		goto create_light_sysfs_failed;

	/* Register to sensors class */
	taos_data->als_cdev = sensors_light_cdev;
	taos_data->als_cdev.sensors_enable = tmd2772_als_set_enable;
	taos_data->als_cdev.sensors_poll_delay = tmd2772_als_poll_delay;
	taos_data->ps_cdev = sensors_proximity_cdev;
	taos_data->ps_cdev.sensors_enable = tmd2772_ps_set_enable;
	taos_data->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&taos_data->a_idev->dev, &taos_data->als_cdev);
	if (ret) {
		SENSOR_LOG_ERROR("register to sensor class failed: %d\n", ret);
		goto als_sensor_register_failed;
	}

	ret = sensors_classdev_register(&taos_data->p_idev->dev, &taos_data->ps_cdev);
	if (ret) {
		SENSOR_LOG_ERROR("register to sensor class failed: %d\n", ret);
		goto classdev_register_failed;
	}

	SENSOR_LOG_INFO("probe OK\n");

	return 0;

classdev_register_failed:
	sensors_classdev_unregister(&taos_data->als_cdev);
als_sensor_register_failed:
	remove_sysfs_interfaces_light(taos_data->light_dev);
create_light_sysfs_failed:
	remove_sysfs_interfaces_prox(taos_data->proximity_dev);
create_proximity_sysfs_failed:
	input_unregister_device(taos_data->a_idev);
input_a_register_failed:
	input_free_device(taos_data->a_idev);
input_a_alloc_failed:
	input_unregister_device(taos_data->p_idev);
input_p_register_failed:
	input_free_device(taos_data->p_idev);
input_p_alloc_failed:
	device_destroy(taos_data->light_class, tmd2772_light_dev_t);
	taos_data->light_dev = NULL;
create_light_dev_failed:
	device_destroy(taos_data->proximity_class, tmd2772_proximity_dev_t);
	taos_data->proximity_dev = NULL;
create_proximity_dev_failed:
	class_destroy(taos_data->light_class);
create_light_class_failed:
	class_destroy(taos_data->proximity_class);
request_irq_failed:
	gpio_free(taos_data->irq_pin_num);
gpio_request_failed:
	destroy_workqueue(taos_data->irq_work_queue);
parse_dt_failed:
	kfree(taos_data->taos_cfg);
read_chipid_failed:
	tmd2772_power_init(taos_data, 0);
power_init_failed:
	kfree(taos_data);
failed:
	SENSOR_LOG_ERROR("probe failed\n");
	return ret;
}

static int tmd2772_remove(struct i2c_client *client)
{
	struct taos_data *taos_data = i2c_get_clientdata(client);

	sensors_classdev_unregister(&taos_data->ps_cdev);
	sensors_classdev_unregister(&taos_data->als_cdev);
	remove_sysfs_interfaces_light(taos_data->light_dev);
	remove_sysfs_interfaces_prox(taos_data->proximity_dev);
	input_unregister_device(taos_data->a_idev);
	input_unregister_device(taos_data->p_idev);
	input_free_device(taos_data->a_idev);
	input_free_device(taos_data->p_idev);
	device_destroy(taos_data->light_class, tmd2772_light_dev_t);
	device_destroy(taos_data->proximity_class, tmd2772_proximity_dev_t);
	class_destroy(taos_data->light_class);
	class_destroy(taos_data->proximity_class);
	gpio_free(taos_data->irq_pin_num);
	destroy_workqueue(taos_data->irq_work_queue);
	tmd2772_power_init(taos_data, 0);
	kfree(taos_data->taos_cfg);
	kfree(taos_data);

	return 0;
}

static int taos_resume(struct device *dev)
{
	int ret = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");

	if (taos_data->prox_on) {
		SENSOR_LOG_DEBUG("disable irq wakeup\n");
		ret = disable_irq_wake(taos_data->client->irq);
		if (ret < 0)
			SENSOR_LOG_ERROR("failed to disable_irq_wake\n");
	}
	else {
		ret = pinctrl_select_state(taos_data->pinctrl, taos_data->pin_default);
		if (ret)
			SENSOR_LOG_ERROR("Can't select pinctrl state\n");
	}

	if (taos_data->als_on) {
		SENSOR_LOG_DEBUG("recovery light sensor work\n");
		schedule_delayed_work(&taos_data->als_poll_work,
					msecs_to_jiffies(1000));
	}

	SENSOR_LOG_DEBUG("exit\n");
	return ret;
}

static int taos_suspend(struct device *dev)
{
	int ret = 0;
	struct taos_data *taos_data = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");

	if (taos_data->prox_on) {
		SENSOR_LOG_DEBUG("enable irq wakeup\n");
		ret = enable_irq_wake(taos_data->client->irq);
		if (ret < 0)
			SENSOR_LOG_ERROR("failed to enable_irq_wake\n");
	}
	else {
		ret = pinctrl_select_state(taos_data->pinctrl, taos_data->pin_sleep);
		if (ret)
			SENSOR_LOG_ERROR("Can't select pinctrl state\n");
	}

	if (taos_data->als_on) {
		SENSOR_LOG_DEBUG("cancel light sensor work\n");
		cancel_delayed_work(&(taos_data->als_poll_work));
		flush_delayed_work(&(taos_data->als_poll_work));
	}

	taos_data->wakeup_from_sleep = true;

	SENSOR_LOG_DEBUG("exit\n");
	return ret;
}

static const struct i2c_device_id tmd2772_idtable_id[] = {
	{"ams,tmd2772", 0},
	{},
};

static struct of_device_id of_tmd2772_idtable[] = {
	{.compatible = "ams,tmd2772",},
	{},
};

MODULE_DEVICE_TABLE(i2c, tmd2772_idtable);

static const struct dev_pm_ops tmd2772_pm_ops = {
	.suspend	= taos_suspend,
	.resume		= taos_resume,
};

static struct i2c_driver tmd2772_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tmd2772",
		.of_match_table = of_tmd2772_idtable,
		.pm = &tmd2772_pm_ops,
	},
	.id_table = tmd2772_idtable_id,
	.probe = tmd2772_probe,
	.remove = tmd2772_remove,
};

static int __init taos_init(void)
{
	return i2c_add_driver(&tmd2772_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&tmd2772_driver);
}

module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

