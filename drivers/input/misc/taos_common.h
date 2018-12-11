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
#ifndef __TAOS_COMMON_H__
#define __TAOS_COMMON_H__

#include <linux/wakelock.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>

/* device id/address */
#define TAOS_DEVICE_NAME		"tritonFN"
#define TAOS_ID_NAME_SIZE		10
#define TMD2772_DEVICE_ID		0x39

/* Triton register */
#define TAOS_TRITON_CNTRL		0x00
#define TAOS_TRITON_ALS_TIME		0x01
#define TAOS_TRITON_PRX_TIME		0x02
#define TAOS_TRITON_WAIT_TIME		0x03
#define TAOS_TRITON_INTERRUPT		0x0C
#define TAOS_TRITON_PRX_CFG		0x0D
#define TAOS_TRITON_PRX_COUNT		0x0E
#define TAOS_TRITON_GAIN		0x0F
#define TAOS_TRITON_CHIPID		0x12
#define TAOS_TRITON_STATUS		0x13
#define TAOS_TRITON_ALS_CHAN0LO		0x14
#define TAOS_TRITON_PRX_OFFSET		0x1E

/* Triton cmd reg masks */
#define TAOS_TRITON_CMD_REG		0x80
#define TAOS_TRITON_CMD_AUTO		0x20
#define TAOS_TRITON_CMD_WORD_BLK_RW	0x20
#define TAOS_TRITON_CMD_SPL_FN		0x60

/* Triton cntrl reg masks */
#define TAOS_TRITON_CNTL_PROX_INT_ENBL	0x20
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL	0x08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL	0x04
#define TAOS_TRITON_CNTL_ADC_ENBL	0x02
#define TAOS_TRITON_CNTL_PWRON		0x01

/* Triton status reg masks */
#define TAOS_TRITON_STATUS_ADCVALID	0x01

/* lux constants parameters */
#define ALS_MAX_LUX			10000
#define ALS_FILTER_DEPTH		3
#define	ALS_ADC_TIME_PROX_ON		0xF0
#define ALS_GAIN_DIVIDE			1000
#define ALS_GAIN_1X			0
#define ALS_GAIN_8X			1
#define ALS_SCALE_FACTOR_DEFAULT	6
#define ALS_GAIN_DEFAULT		0
#define ALS_ADC_TIME_DEFAULT		0xF0
#define ALS_TIME_PARAM			41
#define ALS_POLL_DELAY_DEFAULT		1000

/*prox constants parameters */
#define PROX_THRESHOLD_DISTANCE		100
#define PROX_DATA_TARGET		150
#define PROX_DATA_MAX			1023
#define PROX_OFFSET_CAL_BUFFER_SIZE	30
#define PROX_DATA_SAFE_RANGE_MIN	(PROX_DATA_TARGET - 50)
#define PROX_DATA_SAFE_RANGE_MAX	(PROX_DATA_TARGET + 250)

#define PROX_DEFAULT_THRESHOLD_HIGH	800
#define PROX_DEFAULT_THRESHOLD_LOW	700
#define PROX_THRESHOLD_HIGH_MAX		800
#define PROX_THRESHOLD_HIGH_MIN		500
#define PROX_CAL_HIGH_PARAM		500
#define PROX_CAL_LOW_PARAM		330
#define PROX_THRESHOLD_SAFE_DISTANCE	300

#define PROX_SCALE_FACTOR_DEFAULT	6
#define PROX_ADC_TIME_DEFAULT		0xFF
#define PROX_INTR_FILTER_DEFAULT	0x33
#define PROX_CONFIG_DEFAULT		0x00
#define PROX_GAIN_DEFAULT		0x20
#define PROX_CONFIG_OFFSET_DEFAULT	0x00

#define WAIT_TIME_DEFAULT		0xFF

/*file saved */
#define PATH_PROX_CAL_THRESHOLD		"/persist/proxdata/threshold"
#define PATH_PROX_OFFSET		"/persist/sensors/proximity/offset/proximity_offset"
#define PATH_PROX_UNCOVER_DATA		"/persist/sensors/proximity/uncover_data"

/* POWER SUPPLY VOLTAGE RANGE */
#define TMD2772_VDD_MIN_UV		2850000
#define TMD2772_VDD_MAX_UV		2850000
#define TMD2772_VIO_MIN_UV		1800000
#define TMD2772_VIO_MAX_UV		1800000

struct taos_wake_lock {
	struct wake_lock lock;
	bool locked;
	char *name;
};

/* lux time scale */
struct time_scale_factor {
	u16 numerator;
	u16 denominator;
	u16 saturation;
};

/* device configuration */
struct taos_cfg {
	u16 prox_threshold_hi;
	u16 prox_threshold_lo;
	u8 prox_adc_time;
	u16 prox_scale_factor;
	u8 prox_intr_filter;
	u8 prox_config;
	u8 prox_pulse_cnt;
	u8 prox_gain;
	u8 prox_config_offset;

	u8 wait_time;

	u16 als_adc_time;
	u16 als_scale_factor;
	u8 als_gain;
	unsigned int als_poll_delay;
};

/* proximity data */
struct taos_prox_info {
	u16 prox_clear;
	u16 prox_data;
	int prox_event;
};

/* lux data */
struct lux_data {
	u16 ratio;
	u16 clear;
	u16 ir;
};

struct taos_data {
	struct i2c_client *client;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct taos_wake_lock proximity_wakelock;
	struct mutex lock;
	struct delayed_work als_poll_work;
	struct delayed_work prox_calibrate_work;
	struct delayed_work prox_offset_cal_work;
	struct hrtimer prox_unwakelock_timer;
	struct input_dev *p_idev;
	struct input_dev *a_idev;

	struct taos_cfg *taos_cfg;

	struct class *proximity_class;
	struct class *light_class;

	struct device *proximity_dev;
	struct device *light_dev;

	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	struct regulator *vdd;
	struct regulator *vio;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	char *prox_name;
	char *als_name;
	bool prox_calibrate_flag;
	bool prox_calibrate_result;
	bool prox_offset_cal_result;

	bool prox_offset_cal_verify;
	bool prox_calibrate_verify;

	int light_percent;
	int prox_calibrate_times;
	int prox_thres_hi_max;
	int prox_thres_hi_min;
	int prox_thres_lo_max;
	int prox_thres_lo_min;
	int prox_data_max;
	int prox_manual_calibrate_threshold;
	int irq_pin_num;
	int prox_uncover_data;

	char *chip_name;

	bool prox_on;
	bool als_on;
	bool irq_enabled;
	bool irq_work_status;
	int als_poll_time_mul;

	/* debug flag */
	bool pro_ft;
	bool flag_als_debug;
	bool flag_prox_debug;
	unsigned int prox_debug_delay_time;

	/* prox threshold calculation coefficient */
	u16 prox_calibrate_hi_param;
	u16 prox_calibrate_lo_param;

	unsigned char g_reg_addr;
	u16 als_saturation_value;

	bool wakeup_from_sleep;
};

/* forward declarations */
static int tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
static int tmd2772_remove(struct i2c_client *client);
static int taos_suspend(struct device *dev);
static int taos_resume(struct device *dev);

static int taos_prox_on(struct taos_data *taos_data);
static int taos_prox_off(struct taos_data *taos_data);
static int taos_prox_poll(struct taos_data *taos_data, struct taos_prox_info *prxp);
static int taos_prox_threshold_set(struct taos_data *taos_data);
static int taos_interrupts_clear(struct taos_data *taos_data);
static int taos_prox_calibrate(struct taos_data *taos_data);
static void taos_prox_calibrate_work_func(struct work_struct *work);
static void taos_prox_offset_cal_work_func(struct work_struct *work);
static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable);
static int taos_write_cal_file(struct taos_data *taos_data, char *file_path, unsigned int value);
static int taos_read_cal_value(struct taos_data *taos_data, char *file_path, int *value);
static enum hrtimer_restart taos_prox_unwakelock_work_func(struct hrtimer *timer);
static int taos_get_data(struct taos_data *taos_data);

static int taos_sensors_als_poll_on(struct taos_data *taos_data);
static int taos_sensors_als_poll_off(struct taos_data *taos_data);
static void taos_als_poll_work_func(struct work_struct *work);
static int taos_als_gain_set(struct taos_data *taos_data, unsigned als_gain);
static void taos_update_sat_als(struct taos_data *taos_data);
static int taos_als_get_data(struct taos_data *taos_data);
static int taos_get_lux(struct taos_data *taos_data, int *lux_value);
static int taos_lux_filter(int raw_lux);

#endif
