/********************************************************************************
* Device driver for monitoring ambient light intensity (lux) and proximity
* detection for the TAOS TSL2x7x and TMD2x7x family of devices.
*
* Copyright (c) 2012, TAOS Corporation.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later vers ion.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA      02110-1301, USA.
********************************************************************************/

/*******************************************************************************
*                                                                              *
*       File Name:      taos_common.h                                          *
*       Description:    Common file for ioctl and configuration definitions.   *
*       		Used by kernel driver and driver access applications.  *
*       		Please include this file, and <sys/ioctl.h> in your    *
*                       driver access application program source.	       *
*       Author:         John Koshi                                             *
*       History:        09/16/2009 - Initial creation                          *
*       		02/07/2010 - Add proximity			       *
*                                                                              *
*******************************************************************************/
// ioctl numbers
#define TAOS_IOCTL_MAGIC		0XCF
#define TAOS_IOCTL_ALS_ON		_IO(TAOS_IOCTL_MAGIC, 1)
#define TAOS_IOCTL_ALS_OFF		_IO(TAOS_IOCTL_MAGIC, 2)
#define TAOS_IOCTL_ALS_DATA		_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 4)
#define TAOS_IOCTL_CONFIG_GET		_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_cfg)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_cfg)
#define TAOS_IOCTL_PROX_ON		_IO(TAOS_IOCTL_MAGIC, 7)
#define TAOS_IOCTL_PROX_OFF		_IO(TAOS_IOCTL_MAGIC, 8)
#define TAOS_IOCTL_PROX_DATA		_IOR(TAOS_IOCTL_MAGIC, 9, struct taos_prox_info)
#define TAOS_IOCTL_PROX_EVENT		_IO(TAOS_IOCTL_MAGIC, 10)
#define TAOS_IOCTL_PROX_CALIBRATE	_IO(TAOS_IOCTL_MAGIC, 11)

#define TAOS_IOCTL_SENSOR_ON		_IO(TAOS_IOCTL_MAGIC, 12)
#define TAOS_IOCTL_SENSOR_OFF		_IO(TAOS_IOCTL_MAGIC, 13)
#define TAOS_IOCTL_SENSOR_CONFIG	_IOW(TAOS_IOCTL_MAGIC, 14, struct taos_cfg)
#define TAOS_IOCTL_SENSOR_CHECK		_IO(TAOS_IOCTL_MAGIC, 15)
#define TAOS_IOCTL_SENSOR_test		_IO(TAOS_IOCTL_MAGIC, 16)


// device configuration
struct taos_cfg {
	u32	calibrate_target;
	u16	als_time;
	u16	scale_factor;
	u16	gain_trim;
	u16	prox_threshold_hi;
	u16	prox_threshold_lo;
	u16	als_threshold_hi;
	u16	als_threshold_lo;
	u8	filter_history;
	u8	filter_count;
	u8	gain;
	u8	prox_int_time;
	u8	prox_adc_time;
	u8	prox_wait_time;
	u8	prox_intr_filter;
	u8	prox_config;
	u8	prox_pulse_cnt;
	u8	prox_gain;
};

// proximity data
struct taos_prox_info {
	u16	prox_clear;
	u16	prox_data;
	int	prox_event;
};

struct tmd2771x_platform_data {
	u8	pdrive;
	u8	ppcount;
	int	irq_gpio;
//	int	(*setup_resources)(void);
//	int	(*release_resources)(void);
};

