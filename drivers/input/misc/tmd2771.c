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

/********************************************************************************
*										*
*   File Name:    tmd2771.c							*
*   Description:   Linux device driver for Taos ambient light and		*
*   proximity sensors.								*
   Author:         John Koshi							*
*   History:	09/16/2009 - Initial creation					*
*				10/09/2009 - Triton version			*
*				12/21/2009 - Probe/remove mode			*
*				02/07/2010 - Add proximity			*
*										*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/taos_common.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>

// device name/id/address/counts
#define TAOS_SENSOR_TMD2771_NAME	"tmd2771"
#define TAOS_DEVICE_NAME                TAOS_SENSOR_TMD2771_NAME
#define TAOS_DEVICE_ID                  TAOS_SENSOR_TMD2771_NAME
#define TAOS_INPUT_NAME			TAOS_SENSOR_TMD2771_NAME
#define TAOS_SENSOR_INFO		"1.0"

#define TAOS_ID_NAME_SIZE               10
#define TAOS_MAX_NUM_DEVICES            1
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                9

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG		0X80
#define TAOS_TRITON_CMD_AUTO		0x10
#define TAOS_TRITON_CMD_BYTE_RW		0x00
#define TAOS_TRITON_CMD_SPL_FN		0x60
#define TAOS_TRITON_CMD_PROX_INTCLR	0X05
#define TAOS_TRITON_CMD_ALS_INTCLR	0X06
#define TAOS_TRITON_CMD_INTCLR		0X07

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL	0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL	0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL	0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL	0X04
#define TAOS_TRITON_CNTL_SENS_ENBL	0x0F
#define TAOS_TRITON_CNTL_ADC_ENBL	0x02
#define TAOS_TRITON_CNTL_PWRON		0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID	0x01

// lux constants
#define TAOS_MAX_LUX			10000
#define TAOS_FILTER_DEPTH		3

// forward declarations
struct taos_data;

static int taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp);
static int taos_remove(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void taos_early_suspend(struct early_suspend *h);
static void taos_late_resume(struct early_suspend *h);
#endif
static int taos_open(struct inode *inode, struct file *file);
static int taos_release(struct inode *inode, struct file *file);
static long taos_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int taos_read(struct file *file, char *buf, size_t count, loff_t * ppos);
static int taos_write(struct file *file, const char *buf, size_t count,
		loff_t * ppos);
static loff_t taos_llseek(struct file *file, loff_t offset, int orig);
static int taos_get_lux(void);
static int taos_device_name(unsigned char *bufp, char **device_name);
static int taos_prox_poll(struct taos_prox_info *prxp);

static int taos_als_threshold_set(void);
static int taos_prox_threshold_set(void);
static int taos_als_get_data(void);

#define PS_INT_CLR	0
#define ALS_INT_CLR	1
static int taos_ps_als_int_clear(int type);
static int sensor_on(void);
static int sensor_off(void);
static int taos_als_calibrate(void);
static int taos_ps_calibrate(void);
static int taos_als_power_on(struct taos_data *taos_datap);
static int taos_als_power_off(struct taos_data *taos_datap);
static int taos_ps_power_on(struct taos_data *taos_datap);
static int taos_ps_power_off(struct taos_data *taos_datap);

DECLARE_WAIT_QUEUE_HEAD(waitqueue_read);

static unsigned int ReadEnable = 0;
struct ReadData {
	unsigned int data;
	unsigned int interrupt;
};
struct ReadData readdata[2];

// first device number
static dev_t taos_dev_number;

// class structure for this device
struct class *taos_class;

// module device table
static struct i2c_device_id taos_idtable[] = {
	{TAOS_DEVICE_ID, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, taos_idtable);

// client and device
struct i2c_client *my_clientp;
static char pro_buf[4];
static int mcount = 0;
static char als_buf[4];
u16 status = 0;
static int ALS_ON = 0;
static int PS_ON = 0;

#ifdef CONFIG_OF
static struct of_device_id goodix_match_table[] = {
                { .compatible = "taos,tmd2771",},
                { },
        };
#else
#define goodix_match_table NULL
#endif
// driver definition
static struct i2c_driver taos_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = TAOS_DEVICE_NAME,
		.of_match_table = goodix_match_table,
	},
	.id_table = taos_idtable,
	.probe = taos_probe,
	.remove = /*__devexit_p(taos_remove)*/taos_remove,
};

// per-device data
struct taos_data {
	struct i2c_client *client;
	struct cdev cdev;
	struct input_dev *input_dev;
	struct work_struct work;
	struct mutex date_lock;
	char taos_name[TAOS_ID_NAME_SIZE];
	struct semaphore update_lock;
	struct wake_lock taos_wake_lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int working;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
};

static struct taos_data *taos_datap;

// file operations
static struct file_operations taos_fops = {
	.owner = THIS_MODULE,
	.open = taos_open,
	.release = taos_release,
	.read = taos_read,
	.write = taos_write,
	.llseek = taos_llseek,
	.unlocked_ioctl = taos_ioctl,
};

// device configuration
struct taos_cfg *taos_cfgp;
static u32 calibrate_target_param = 300000;
static u16 als_time_param = 200;
static u16 scale_factor_param = 1;
static u16 gain_trim_param = 512;
static u8 filter_history_param = 3;
static u8 gain_param = 0;

//static u16 prox_threshold_hi_param = 820;
//static u16 prox_threshold_lo_param = 550;

static u16 prox_threshold_hi_param = 500;
static u16 prox_threshold_lo_param = 260;

static u16 als_threshold_hi_param = 3000;
static u16 als_threshold_lo_param = 10;
static u8 prox_int_time_param = 0xEE;		//50ms
static u8 prox_adc_time_param = 0xFF;
static u8 prox_wait_time_param = 0xEE;
static u8 prox_intr_filter_param = 0x13;
static u8 prox_config_param = 0x00;
static u8 prox_pulse_cnt_param = 0x02;	//set led pulse count
static u8 prox_gain_param = 0x20;

// prox info
struct taos_prox_info prox_cal_info[20];
struct taos_prox_info prox_cur_info;
struct taos_prox_info *prox_cur_infop = &prox_cur_info;
static u8 prox_history_hi = 0;
static u8 prox_history_lo = 0;
static int prox_on = 0;
static int device_released = 0;
static u16 sat_als = 0;
static u16 sat_prox = 0;

// device reg init values
u8 taos_triton_reg_init[16] = { 0x00, 0xFF, 0XFF, 0XFF, 0X00, 0X00, 0XFF, 0XFF,
	0X00, 0X00, 0XFF, 0XFF, 0X00, 0X00, 0X00, 0X00
};

// lux time scale
struct time_scale_factor {
	u16 numerator;
	u16 denominator;
	u16 saturation;
};
struct time_scale_factor TritonTime = { 1, 0, 0 };

struct time_scale_factor *lux_timep = &TritonTime;

// gain table
u8 taos_triton_gain_table[] = { 1, 8, 16, 120 };

// lux data
struct lux_data {
	u16 ratio;
	u16 clear;
	u16 ir;
};

struct lux_data TritonFN_lux_data[] = {
	{9830, 8320, 15360},
	{12452, 10554, 22797},
	{14746, 6234, 11430},
	{17695, 3968, 6400},
	{0, 0, 0}
};

struct lux_data *lux_tablep = TritonFN_lux_data;

static int lux_history[TAOS_FILTER_DEPTH] = { -ENODATA, -ENODATA, -ENODATA };


struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator taos_vreg[] = {
	{NULL, "vdd", 1700000, 3600000},
	{NULL, "vddio", 1700000, 3600000},
};

static int taos_config_regulator(struct taos_data *datap, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(taos_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			taos_vreg[i].vreg =
				regulator_get(&taos_datap->client->dev,
				taos_vreg[i].name);
			if (IS_ERR(taos_vreg[i].vreg)) {
				rc = PTR_ERR(taos_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				taos_vreg[i].vreg = NULL;
				goto error_vdd;
			}

			if (regulator_count_voltages(
				taos_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					taos_vreg[i].vreg,
					taos_vreg[i].min_uV,
					taos_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(taos_vreg[i].vreg);
					taos_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			rc = regulator_enable(taos_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					taos_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						taos_vreg[i].vreg, 0,
						taos_vreg[i].max_uV);
				}
				regulator_put(taos_vreg[i].vreg);
				taos_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(taos_vreg[i].vreg)) {
			if (regulator_count_voltages(
			taos_vreg[i].vreg) > 0) {
				regulator_set_voltage(taos_vreg[i].vreg,
						0, taos_vreg[i].max_uV);
			}
			regulator_disable(taos_vreg[i].vreg);
			regulator_put(taos_vreg[i].vreg);
			taos_vreg[i].vreg = NULL;
		}
	}
	return rc;
}


static int taos_read_byte(struct i2c_client *client, u8 reg)
{
	s32 ret;

	reg &= ~TAOS_TRITON_CMD_SPL_FN;
	reg |= TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_BYTE_RW;

	ret = i2c_smbus_read_byte_data(client, reg);
	return ret;
}

static int taos_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	s32 ret;

	reg &= ~TAOS_TRITON_CMD_SPL_FN;
	reg |= TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_BYTE_RW;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	return (int)ret;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	schedule_work(&taos_datap->work);

	return IRQ_HANDLED;
}

static int taos_ps_als_int_clear(int type)
{
	int ret = 0;
	u8 cl_irq = 0;

	if (type == PS_INT_CLR)
		cl_irq = 0x05;
	else
		cl_irq = 0x06;

	if ((ret = (i2c_smbus_write_byte(taos_datap->client,
			(TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | cl_irq)))) < 0) {
				printk(KERN_ERR "TAOS: clear interrupt failed in %s\n",
		       __func__);
		return (ret);
	}

	return ret;
}

static int taos_get_data(void)
{
	int ret = 0;

	if ((status = taos_read_byte(taos_datap->client, TAOS_TRITON_STATUS)) < 0) {
		printk(KERN_ERR "%s: read the chip status is failed\n",
		       __func__);
		return ret;
	}

	if ((status & 0x01) == 0x01) {
		ReadEnable = 1;
		taos_als_threshold_set();
		taos_als_get_data();
		taos_ps_als_int_clear(ALS_INT_CLR);
	}
	if ((status & 0x20) == 0x20) {
		ret = taos_prox_threshold_set();
		if (ret >= 0)
			ReadEnable = 1;
		taos_ps_als_int_clear(PS_INT_CLR);
	}

	return ret;
}

static void taos_work_func(struct work_struct *work)
{
	mutex_lock(&taos_datap->date_lock);
	taos_get_data();
	mutex_unlock(&taos_datap->date_lock);
}

static int taos_als_get_data(void)
{
	int ret = 0;
	u8 reg_val;
	int lux_val = 0;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR "%s: read TAOS_TRITON_CNTRL is failed\n",
		       __func__);
		return reg_val;
	}
	if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) !=
	    (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) {
		printk("%s:adc and power is disenable\n", __func__);
		return ret;
	}

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_STATUS)) < 0) {
		printk(KERN_ERR "%s: read TAOS_TRITON_STATUS is failed\n",
		       __func__);
		return reg_val;
	}
	if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) !=
	    TAOS_TRITON_STATUS_ADCVALID) {
		printk("%s:read the chip status value is failed\n", __func__);
		return ret;
	}

	if ((lux_val = taos_get_lux()) < 0)
		printk(KERN_ERR
		       "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n",
		       lux_val);
	input_report_abs(taos_datap->input_dev, ABS_MISC, lux_val);
	input_sync(taos_datap->input_dev);

	return ret;
}

static int taos_als_threshold_set(void)
{
	int i, ret = 0;
	u8 chdata[2];
	u16 ch0;

	for (i = 0; i < 2; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_ALS_CHAN0LO + i);
	}

	ch0 = chdata[0] + chdata[1] * 256;
	als_threshold_hi_param = (12 * ch0) / 10;
	if (als_threshold_hi_param >= 65535)
		als_threshold_hi_param = 65535;
	als_threshold_lo_param = (8 * ch0) / 10;
	als_buf[0] = als_threshold_lo_param & 0x0ff;
	als_buf[1] = als_threshold_lo_param >> 8;
	als_buf[2] = als_threshold_hi_param & 0x0ff;
	als_buf[3] = als_threshold_hi_param >> 8;

	for (mcount = 0; mcount < 4; mcount++) {
		if ((ret = taos_write_byte(taos_datap->client,
				     TAOS_TRITON_ALS_MINTHRESHLO + mcount, als_buf[mcount])) < 0) {
			printk(KERN_ERR
			       "TAOS: write als failed in taos als threshold set\n");
			return (ret);
		}
	}
	return ret;
}

static int taos_prox_threshold_set(void)
{
	int i, ret = 0;
	u8 chdata[6];
	u16 proxdata = 0;
	int data = 0;
	for (i = 0; i < 6; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_ALS_CHAN0LO + i);
	}
	proxdata = chdata[4] | (chdata[5] << 8);
	if (prox_on || proxdata < taos_cfgp->prox_threshold_lo) {
/*set prox_threshold_hi = 500*/
		pro_buf[0] = 0x00;
		pro_buf[1] = 0x00;
		pro_buf[2] = 0xF4;
		pro_buf[3] = 0x01;
		data = 1;
		input_report_abs(taos_datap->input_dev, ABS_DISTANCE, data);
		input_sync(taos_datap->input_dev);
	} else if (proxdata > taos_cfgp->prox_threshold_hi) {
/*set prox_threshold_lo = 400*/
		pro_buf[0] = 0x90;
		pro_buf[1] = 0x01;
		pro_buf[2] = 0xff;
		pro_buf[3] = 0xff;
		data = 0;
		input_report_abs(taos_datap->input_dev, ABS_DISTANCE, data);
		input_sync(taos_datap->input_dev);
	}

	for (mcount = 0; mcount < 4; mcount++) {
		if ((ret = taos_write_byte(taos_datap->client,
				     TAOS_TRITON_PRX_MINTHRESHLO + mcount, pro_buf[mcount])) < 0) {
			printk(KERN_ERR"%s: wirte the proximity threshold is faild\n",
			       __func__);
			return (ret);
		}
	}

	prox_on = 0;
	return ret;
}

/* ATTR  */
static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Chip: TAOS %s\nVersion: %s\n",
		       TAOS_SENSOR_TMD2771_NAME, TAOS_SENSOR_INFO);
}

static DEVICE_ATTR(info, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   info_show, NULL);

static int fl = 0;

static ssize_t enable_als_store(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	if (strncmp(buf, "1", 1) == 0) {
		if (fl == 0) {
			enable_irq(taos_datap->client->irq);
			fl = 1;
		}
		taos_als_power_on(taos_datap);
	} else {
		if (fl == 1) {
			disable_irq(taos_datap->client->irq);
			fl = 0;
		}
		taos_als_power_off(taos_datap);
	}

	return count;
}

static ssize_t enable_als_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ! !(ALS_ON));
}

static DEVICE_ATTR(enable_als, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   enable_als_show, enable_als_store);

static ssize_t enable_ps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (strncmp(buf, "1", 1) == 0) {
		if (fl == 0) {
			enable_irq(taos_datap->client->irq);
			fl = 1;
		}
		sensor_on();
		taos_ps_calibrate();
		taos_ps_power_on(taos_datap);
	} else {
		if (fl == 1) {
			disable_irq(taos_datap->client->irq);
			fl = 0;
		}
		taos_ps_power_off(taos_datap);
		sensor_off();
	}
	return count;
}

static ssize_t enable_ps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ! !(PS_ON));
}

static DEVICE_ATTR(enable_ps, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   enable_ps_show, enable_ps_store);

static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct taos_prox_info prxp;

	memset(&prxp, 0, sizeof(prxp));
	taos_prox_poll(&prxp);

	return sprintf(buf, "%d\n", prxp.prox_data);
}

static DEVICE_ATTR(raw_adc, S_IRUGO, ps_adc_show, NULL);

static ssize_t als_lux_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int lux;

	lux = taos_get_lux();

	return sprintf(buf, "%d\n", lux);
}

static DEVICE_ATTR(lux_adc, S_IRUGO, als_lux_show, NULL);

static struct attribute *tmd2771_attributes[] = {
	&dev_attr_info.attr,
	&dev_attr_enable_als.attr,
	&dev_attr_enable_ps.attr,
	&dev_attr_raw_adc.attr,
	&dev_attr_lux_adc.attr,
	NULL
};

static struct attribute_group tmd2771_attribute_group = {
	.attrs = tmd2771_attributes,
};

/* ATTR end */
// driver init
static int __init taos_init(void)
{
	int ret = 0;

	if ((ret = (alloc_chrdev_region(&taos_dev_number, 0,
			TAOS_MAX_NUM_DEVICES, TAOS_DEVICE_NAME))) < 0) {
		printk(KERN_ERR
		       "TAOS: alloc_chrdev_region() failed in taos_init()\n");
		goto exit_init;
	}
	taos_class = class_create(THIS_MODULE, TAOS_DEVICE_NAME);
	taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos_datap) {
		printk(KERN_ERR
		       "TAOS: kmalloc for struct taos_data failed in taos_init()\n");
		ret = -ENOMEM;
		goto exit_init;
	}
	memset(taos_datap, 0, sizeof(struct taos_data));
	cdev_init(&taos_datap->cdev, &taos_fops);
	taos_datap->cdev.owner = THIS_MODULE;
	if ((ret = (cdev_add(&taos_datap->cdev, taos_dev_number, 1))) < 0) {
		printk(KERN_ERR "TAOS: cdev_add() failed in taos_init()\n");
		goto err_taos_datap_kfree;
	}
	device_create(taos_class, NULL, MKDEV(MAJOR(taos_dev_number), 0),
		      &taos_driver, "tmd2771");
	if ((ret = (i2c_add_driver(&taos_driver))) < 0) {
		printk(KERN_ERR
		       "TAOS: i2c_add_driver() failed in taos_init()\n");
		goto err_taos_datap_kfree;
	}
	wake_lock_init(&taos_datap->taos_wake_lock, WAKE_LOCK_SUSPEND,
		       "taos_wake_lock");

	goto exit_init;

 err_taos_datap_kfree:
	kfree(taos_datap);
 exit_init:
	return ret;
}

// driver exit
static void __exit taos_exit(void)
{
	if (my_clientp)
		i2c_unregister_device(my_clientp);
	i2c_del_driver(&taos_driver);
	unregister_chrdev_region(taos_dev_number, TAOS_MAX_NUM_DEVICES);
	device_destroy(taos_class, MKDEV(MAJOR(taos_dev_number), 0));
	cdev_del(&taos_datap->cdev);
	class_destroy(taos_class);
	disable_irq(taos_datap->client->irq);
	kfree(taos_datap);
}

static int tmd_parse_dt(struct device *dev,
				struct tmd2771x_platform_data *tmd_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	rc = of_property_read_u32(np, "taos,pdrive", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		tmd_pdata->pdrive = (u8)temp_val;
        printk("wxl pdrive = %d\n",tmd_pdata->pdrive);
	}

	rc = of_property_read_u32(np, "taos,ppcount", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		tmd_pdata->ppcount = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "taos,irq_gpio", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		tmd_pdata->irq_gpio = temp_val;
	}

	return 0;
}
/*
struct tmd2771x_platform_data tmd_pdata = {
	.pdrive = 0x03,
	.ppcount = 0x08,
	.irq_gpio = 80,
	.setup_resources = NULL,
	.release_resources = NULL,
};
*/
// client probe

static int taos_pinctrl_init(struct taos_data *s_tmd)
{
	struct i2c_client *client = s_tmd->client;

	s_tmd->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(s_tmd->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(s_tmd->pinctrl);
	}

	s_tmd->pin_default = pinctrl_lookup_state(s_tmd->pinctrl,
			"default");
	if (IS_ERR_OR_NULL(s_tmd->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(s_tmd->pin_default);
	}

	return 0;
}

static int taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp)
{
	int ret = 0;
	int i = 0;
	unsigned char buf[TAOS_MAX_DEVICE_REGS];
	char *device_name;
	struct tmd2771x_platform_data *pdata;
		printk("==Jensen %s %d\n", __FUNCTION__, __LINE__);

	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus byte data functions unsupported\n");
		return -EOPNOTSUPP;
	}
	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus word data functions unsupported\n");
	}
	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_BLOCK_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus block data functions unsupported\n");
	}
	taos_datap->client = clientp;
	i2c_set_clientdata(clientp, taos_datap);

    pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
                    dev_err(&clientp->dev,
                                        "failed to allocate memory for module data\n");
                            return -ENOMEM;
                                }

    if (clientp->dev.of_node) {
		memset(pdata, 0 , sizeof(*pdata));
		ret = tmd_parse_dt(&clientp->dev, pdata);
		if (ret) {
			dev_err(&clientp->dev,
				"Unable to parse platfrom data err=%d\n", ret);
			return ret;
		}
    }

	ret = taos_config_regulator(taos_datap, true);
	if (ret < 0) {
		dev_err(&taos_datap->client->dev,
				"power_on failed: %d\n", ret);
		printk("==Jensen %s %d\n", __FUNCTION__, __LINE__);
		return ret;
	}

	INIT_WORK(&(taos_datap->work), taos_work_func);
	sema_init(&taos_datap->update_lock, 1);
	mutex_init(&taos_datap->date_lock);
	taos_datap->input_dev = input_allocate_device();
	if (taos_datap->input_dev == NULL) {
		printk("[tmd2771] probe error\n");
		return -ENOMEM;
	}

	taos_datap->input_dev->name = TAOS_INPUT_NAME;
	taos_datap->input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, taos_datap->input_dev->evbit);
	input_set_capability(taos_datap->input_dev, EV_ABS, ABS_DISTANCE);
	input_set_capability(taos_datap->input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(taos_datap->input_dev, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(taos_datap->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	ret = input_register_device(taos_datap->input_dev);

	for (i = 0; i < TAOS_MAX_DEVICE_REGS; i++) {
		if ((buf[i] = taos_read_byte(clientp, TAOS_TRITON_CNTRL + i)) < 0) {
			printk("%s: read the every reg is faild\n", __func__);
			goto err_unregister_input_dev;
		}
	}

/*compare chip-id*/
	if ((ret = taos_device_name(buf, &device_name)) == 0) {
		printk(KERN_ERR
		       "TAOS: chip id that was read found mismatched by taos_device_name(), in taos_probe()\n");
		ret = -ENODEV;
		goto err_unregister_input_dev;
	}

	if (strcmp(device_name, TAOS_DEVICE_ID)) {
		printk(KERN_ERR
		       "TAOS: chip id that was read does not match expected id in taos_probe()\n");
		ret = -ENODEV;
		goto err_unregister_input_dev;
	} else {
		pr_debug("TAOS: chip id of %s that was read matches expected id in taos_probe()\n",
		     device_name);
	}

	strlcpy(clientp->name, TAOS_DEVICE_ID, I2C_NAME_SIZE);
	strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);

	if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL))) {
		printk(KERN_ERR
		       "TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
		ret = -ENOMEM;
		goto err_unregister_input_dev;
	}

	taos_cfgp->calibrate_target = calibrate_target_param;
	taos_cfgp->als_time = als_time_param;
	taos_cfgp->scale_factor = scale_factor_param;
	taos_cfgp->gain_trim = gain_trim_param;
	taos_cfgp->filter_history = filter_history_param;
	taos_cfgp->gain = gain_param;
	taos_cfgp->als_threshold_hi = als_threshold_hi_param;
	taos_cfgp->als_threshold_lo = als_threshold_lo_param;
	taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
	taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
	taos_cfgp->prox_int_time = prox_int_time_param;
	taos_cfgp->prox_adc_time = prox_adc_time_param;
	taos_cfgp->prox_wait_time = prox_wait_time_param;
	taos_cfgp->prox_intr_filter = prox_intr_filter_param;
	taos_cfgp->prox_config = prox_config_param;
	taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
	taos_cfgp->prox_gain = prox_gain_param;
	sat_als = (256 - taos_cfgp->prox_int_time) << 10;
	sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;

	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL,
			     0x00)) < 0) {
		printk(KERN_ERR "%s: write the chip power down is failed\n",
		       __func__);
		goto err_taos_cfgp_kfree;
	}

	/* initialize pinctrl */
	if (!taos_pinctrl_init(taos_datap)) {
		ret = pinctrl_select_state(taos_datap->pinctrl, taos_datap->pin_default);
		if (ret) {
			dev_err(&clientp->dev, "Can't select pinctrl state\n");
			goto err_unregister_input_dev;
		}
	}

	printk("[ tmd2771]clientp->irq = %d\n", clientp->irq);

	ret = gpio_request(pdata->irq_gpio, "taos_irq");
	if (ret) {
		printk(KERN_ALERT "%s: gp2ap request gpio failed.\n",
		       __func__);
		return ret;
	}
	/*gpio_tlmm_config(GPIO_CFG(pdata->irq_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
			GPIO_CFG_ENABLE);*/
	gpio_direction_input(pdata->irq_gpio);
	ret = request_irq(clientp->irq, taos_irq_handler, IRQ_TYPE_EDGE_FALLING,
			"taos_irq", taos_datap);
	printk("[ tmd2771]request_irq ret = %d\n", ret);

	if (ret != 0) {
		printk("\nrequest tmd2771 irq : %d failed\n", clientp->irq);
		goto err_taos_cfgp_kfree;
	}

	printk("request tmd2771 irq : %d succeed\n", clientp->irq);
	disable_irq(clientp->irq);

	if (taos_datap == NULL) {
		printk("[tmd2771] taos_datap == NULL\n");
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	taos_datap->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	taos_datap->early_suspend.suspend = taos_early_suspend;
	taos_datap->early_suspend.resume = taos_late_resume;
	register_early_suspend(&taos_datap->early_suspend);
#endif

	ret = sysfs_create_group(&taos_datap->input_dev->dev.kobj,
			       &tmd2771_attribute_group);
	if (!ret)
		goto exit;

 err_taos_cfgp_kfree:
	kfree(taos_cfgp);
 err_unregister_input_dev:
	input_unregister_device(taos_datap->input_dev);
 exit:
	return ret;
}

// client remove
static int /*__devexit*/ taos_remove(struct i2c_client *client)
{
	int ret = 0;

	kfree(taos_cfgp);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&taos_datap->early_suspend);
#endif
	input_unregister_device(taos_datap->input_dev);
	sysfs_remove_group(&taos_datap->input_dev->dev.kobj,
			   &tmd2771_attribute_group);
	ret = taos_config_regulator(taos_datap, false);
	if (ret < 0) {
		dev_err(&taos_datap->client->dev,
				"power_on failed: %d\n", ret);
		printk("==Jensen %s %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	return 0;
}

//resume
#ifdef CONFIG_HAS_EARLYSUSPEND
static u8 reg_store = 0;
static void taos_late_resume(struct early_suspend *h)
{
	u8 reg_val = 0, reg_cntrl = 0;
	int ret = -1;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk("TAOS: read byte is failed in resume\n");
		return;
	}
	if (taos_datap->working == 1) {
		taos_datap->working = 0;
		reg_cntrl = reg_val | TAOS_TRITON_CNTL_PWRON;
		if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL, reg_store)) < 0) {	//reg_cntrl)) < 0) {
			printk(KERN_ERR
			       "TAOS: write byte_data failed in ioctl als_off\n");
			return;
		}
	}
	enable_irq(taos_datap->client->irq);
}

//suspend
static void taos_early_suspend(struct early_suspend *h)
{
	u8 reg_val = 0;
	int ret = -1;

	disable_irq(taos_datap->client->irq);
	taos_datap->working = 1;
	if ((reg_store = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk("TAOS: read byte is failed in suspend\n");
		return;
	}
	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL,
			     reg_val)) < 0) {
		printk(KERN_ERR "TAOS: write byte failed in taos_suspend\n");
		return;
	}
}
#endif
// open
static int taos_open(struct inode *inode, struct file *file)
{
	struct taos_data *taos_datap;
	int ret = 0;

	device_released = 0;
	taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
	if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) {
		printk(KERN_ERR
		       "TAOS: device name incorrect during taos_open(), get %s\n",
		       taos_datap->taos_name);
		ret = -ENODEV;
	}
	memset(readdata, 0, sizeof(struct ReadData) * 2);
	enable_irq(taos_datap->client->irq);
	return (ret);
}

// release
static int taos_release(struct inode *inode, struct file *file)
{
	struct taos_data *taos_datap;
	int ret = 0;

	device_released = 1;
	prox_on = 0;
	prox_history_hi = 0;
	prox_history_lo = 0;
	taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
	if (strcmp(taos_datap->taos_name, TAOS_DEVICE_ID) != 0) {
		printk(KERN_ERR
		       "TAOS: device name incorrect during taos_release(), get %s\n",
		       taos_datap->taos_name);
		ret = -ENODEV;
	}
	return (ret);
}

// read
static int taos_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
	unsigned long flags;
	int realmax;
	int err;
	if ((!ReadEnable) && (file->f_flags & O_NONBLOCK))
		return -EAGAIN;
	local_save_flags(flags);
	local_irq_disable();

	realmax = 0;
	if (down_interruptible(&taos_datap->update_lock))
		return -ERESTARTSYS;
	if (ReadEnable > 0) {
		if (sizeof(struct ReadData) * 2 < count)
			realmax = sizeof(struct ReadData) * 2;
		else
			realmax = count;
		err = copy_to_user(buf, readdata, realmax);
		if (err)
			return -EAGAIN;
		ReadEnable = 0;
	}
	up(&taos_datap->update_lock);
	memset(readdata, 0, sizeof(struct ReadData) * 2);
	local_irq_restore(flags);
	return realmax;
}

// write
static int taos_write(struct file *file, const char *buf, size_t count,
		      loff_t * ppos)
{
	struct taos_data *taos_datap;
	u8 i = 0, xfrd = 0, reg = 0;
	u8 my_buf[TAOS_MAX_DEVICE_REGS];
	int ret = 0;

	if ((*ppos < 0) || (*ppos >= TAOS_MAX_DEVICE_REGS)
	    || ((*ppos + count) > TAOS_MAX_DEVICE_REGS)) {
		printk(KERN_ERR
		       "TAOS: reg limit check failed in taos_write()\n");
		return -EINVAL;
	}
	reg = (u8) * ppos;
	if ((ret = copy_from_user(my_buf, buf, count))) {
		printk(KERN_ERR "TAOS: copy_to_user failed in taos_write()\n");
		return -ENODATA;
	}
	taos_datap =  container_of(file->f_dentry->d_inode->i_cdev,
					struct taos_data, cdev);
	while (xfrd < count) {
		if ((ret = taos_write_byte(taos_datap->client, reg,
				     my_buf[i++])) < 0) {
			printk(KERN_ERR
			       "TAOS: write reg failed in taos_write()\n");
			return (ret);
		}
		reg++;
		xfrd++;
	}
	return ((int)xfrd);
}

// llseek
static loff_t taos_llseek(struct file *file, loff_t offset, int orig)
{
	int ret = 0;
	loff_t new_pos = 0;

	if ((offset >= TAOS_MAX_DEVICE_REGS) || (orig < 0) || (orig > 1)) {
		printk(KERN_ERR
		       "TAOS: offset param limit or origin limit check failed in taos_llseek()\n");
		return -EINVAL;
	}
	switch (orig) {
	case 0:
		new_pos = offset;
		break;
	case 1:
		new_pos = file->f_pos + offset;
		break;
	default:
		return -EINVAL;
		break;
	}
	if ((new_pos < 0) || (new_pos >= TAOS_MAX_DEVICE_REGS) || (ret < 0)) {
		printk(KERN_ERR
		       "TAOS: new offset limit or origin limit check failed in taos_llseek()\n");
		return -EINVAL;
	}
	file->f_pos = new_pos;
	return new_pos;
}

static int sensor_on(void)
{
	int ret = 0;
	int i = 0;

	printk("[ tmd2771] < %s >\n", __func__);
	for (i = 0; i < TAOS_FILTER_DEPTH; i++) {
		lux_history[i] = -ENODATA;
	}
	/*ALS interrupt clear */
	if ((ret = (i2c_smbus_write_byte
	      (taos_datap->client,
	       (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN |
		TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
		printk(KERN_ERR
		       "TAOS: i2c_smbus_write_byte failed in ioctl als_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_ALS_TIME,
			     taos_cfgp->prox_int_time)) < 0) {
		printk(KERN_ERR
		       "TAOS: write als_time failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_TIME,
			     taos_cfgp->prox_adc_time)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_time failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_WAIT_TIME,
			     taos_cfgp->prox_wait_time)) < 0) {
		printk(KERN_ERR
		       "TAOS: write wait_time failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_INTERRUPT,
			     taos_cfgp->prox_intr_filter)) < 0) {
		printk(KERN_ERR
		       "TAOS: write interrupt failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_CFG,
			     taos_cfgp->prox_config)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_config failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_CFG,
			     taos_cfgp->prox_config)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_config failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_COUNT,
			     taos_cfgp->prox_pulse_cnt)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_config failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_GAIN, taos_cfgp->prox_gain)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_config failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_CNTRL,
			     TAOS_TRITON_CNTL_SENS_ENBL)) < 0) {
		printk(KERN_ERR
		       "TAOS: write prox_config failed in ioctl prox_on\n");
		return (ret);
	}
	return 0;
}

static int sensor_off(void)
{
	int ret = 0;

	/*turn off */
	printk(KERN_ERR "TAOS: TAOS_IOCTL_SENSOR_OFF\n");
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_CNTRL, 0x00)) < 0) {
		printk(KERN_ERR
		       "TAOS: write sensor_off failed in ioctl prox_on\n");
		return (ret);
	}
	return ret;
}

static int taos_als_power_on(struct taos_data *taos_datap)
{
	int ret = 0, i = 0;
	u8 itime = 0, reg_val = 0, reg_cntrl = 0;

	for (i = 0; i < TAOS_FILTER_DEPTH; i++)
		lux_history[i] = -ENODATA;

	/*ALS_INTERUPTER IS CLEAR */
	if ((ret = (i2c_smbus_write_byte
	      (taos_datap->client,
	       (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN |
		TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
		printk(KERN_ERR
		       "TAOS: i2c_smbus_write_byte failed in ioctl als_on\n");
		return (ret);
	}

	itime = (((taos_cfgp->als_time / 50) * 18) - 1);
	itime = (~itime);

	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_ALS_TIME,
			     itime)) < -1) {
		printk(KERN_ERR "%s: write the als time is failed\n", __func__);
		return ret;
	}

	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_INTERRUPT,
			     taos_cfgp->prox_intr_filter)) < 0) {
		printk(KERN_ERR "%s: write the als time is failed\n", __func__);
		return ret;
	}

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_GAIN)) < 0) {
		printk(KERN_ERR "%s: read the als gain is failed\n", __func__);
		return ret;
	}

	reg_val = reg_val & 0xFC;
	reg_val = reg_val | (taos_cfgp->gain & 0x03);

	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_GAIN,
			     reg_val)) < 0) {
		printk(KERN_ERR "%s: write the als gain is failed\n", __func__);
		return ret;
	}

	if ((reg_cntrl = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR
		       "%s:  taos_read_byte TAOS_TRITON_CNTRL is failed\n",
		       __func__);
		return ret;
	}

	reg_cntrl |= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON |
							TAOS_TRITON_CNTL_ALS_INT_ENBL);
	if ((ret = taos_write_byte(taos_datap->client,
				TAOS_TRITON_CNTRL, reg_cntrl)) < 0) {
		printk(KERN_ERR "%s: write the als data is failed\n", __func__);
		return ret;
	}

	taos_als_threshold_set();

	ALS_ON = 1;
	return ret;
}

static int taos_als_power_off(struct taos_data *taos_datap)
{
	int ret = 0;
	u8 reg_val = 0;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR "TAOS: read CNTRL failed in ioctl prox_on\n");
		return ret;
	}
	reg_val = reg_val & ~(1 << 1);
	reg_val = reg_val & ~(1 << 4);

	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_CNTRL, reg_val)) < 0) {
		printk(KERN_ERR "TAOS: write CNTRL failed in %s\n", __func__);
		return (ret);
	}
	ALS_ON = 0;
	return (ret);
}

static int taos_ps_power_on(struct taos_data *taos_datap)
{
	int ret = 0;
	u8 reg_cntrl = 0;

		taos_write_byte(taos_datap->client, 0x08, taos_cfgp->prox_threshold_lo & 0x0f) ;
		taos_write_byte(taos_datap->client, 0x09, taos_cfgp->prox_threshold_lo >> 8) ;
		taos_write_byte(taos_datap->client, 0x0a, taos_cfgp->prox_threshold_hi & 0x0f) ;
		taos_write_byte(taos_datap->client, 0x0b, taos_cfgp->prox_threshold_hi >> 8) ;
	prox_on = 1;
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_TIME,
			     taos_cfgp->prox_adc_time)) < -1) {
		printk(KERN_ERR
		       "%s: write the prox time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_WAIT_TIME,
			     taos_cfgp->prox_wait_time)) < -1) {
		printk(KERN_ERR
		       "%s: write the wait time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_INTERRUPT,
			     taos_cfgp->prox_intr_filter)) < -1) {
		printk(KERN_ERR
		       "%s: write the interrupt time is failed\n", __func__);
		return ret;
	}

	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_CFG,
			     taos_cfgp->prox_config)) < -1) {
		printk(KERN_ERR
		       "%s: write the prox_config is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_COUNT,
			     taos_cfgp->prox_pulse_cnt)) < -1) {
		printk(KERN_ERR
		       "%s: write the pulse count time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_GAIN, taos_cfgp->prox_gain)) < -1) {
		printk(KERN_ERR
		       "%s: write the gain time is failed\n", __func__);
		return ret;
	}
	if ((reg_cntrl = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR "TAOS: read CNTRL failed in ioctl prox_on\n");
		return ret;
	}
	reg_cntrl |=
	    TAOS_TRITON_CNTL_PROX_DET_ENBL |
	    TAOS_TRITON_CNTL_PWRON |
	    TAOS_TRITON_CNTL_PROX_INT_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
	if ((ret = (i2c_smbus_write_byte_data
	      (taos_datap->client,
	       (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR
		       "TAOS: write reg_cntrl failed in ioctl prox_on\n");
		return (ret);
	}
	taos_prox_threshold_set();
	PS_ON = 1;

	return ret;
}

static int taos_ps_power_off(struct taos_data *taos_datap)
{
	int ret = 0;
	u8 reg_val = 0;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR "TAOS: read CNTRL failed in ioctl prox_on\n");
		return ret;
	}
	reg_val = reg_val & ~(1 << 2);
	reg_val = reg_val & ~(1 << 5);
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_CNTRL, reg_val)) < 0) {
		printk(KERN_ERR "TAOS: write CNTRL failed in %s\n", __func__);
		return (ret);
	}
	PS_ON = 0;
	if(ALS_ON) {
		taos_als_power_off(taos_datap);
		taos_als_power_on(taos_datap);
	}
	return ret;
}

/*ignore the automatic calculation of threshold and set by manual*/
#define IGNORE_THRESHOLD_CALC
static int taos_ps_calibrate(void)
{
	int ret = 0, i = 0;
	u8 reg_val = 0, reg_cntrl = 0;
#ifndef IGNORE_THRESHOLD_CALC
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
#endif

	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_ALS_TIME,
			     taos_cfgp->prox_int_time)) < -1) {
		printk(KERN_ERR "%s: write the als time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_TIME,
			     taos_cfgp->prox_adc_time)) < -1) {
		printk(KERN_ERR
		       "%s: write the prox time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_WAIT_TIME,
			     taos_cfgp->prox_wait_time)) < -1) {
		printk(KERN_ERR
		       "%s: write the wait time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_INTERRUPT,
			     taos_cfgp->prox_intr_filter)) < -1) {
		printk(KERN_ERR
		       "%s: write the interrupt time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_CFG,
			     taos_cfgp->prox_config)) < -1) {
		printk(KERN_ERR
		       "%s: write the prox_config is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_PRX_COUNT,
			     taos_cfgp->prox_pulse_cnt)) < -1) {
		printk(KERN_ERR
		       "%s: write the pulse count time is failed\n", __func__);
		return ret;
	}
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_GAIN, taos_cfgp->prox_gain)) < -1) {
		printk(KERN_ERR
		       "%s: write the gain time is failed\n", __func__);
		return ret;
	}
	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR "TAOS: read CNTRL failed in ioctl prox_on\n");
		return ret;
	}
	reg_cntrl = reg_val | (TAOS_TRITON_CNTL_PROX_DET_ENBL |
		       TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
	if ((ret = taos_write_byte(taos_datap->client,
			     TAOS_TRITON_CNTRL, reg_cntrl)) < 0) {
		printk(KERN_ERR
		       "TAOS: write reg_cntrl failed in ioctl prox_on\n");
		return (ret);
	}

#ifndef IGNORE_THRESHOLD_CALC
	prox_sum = 0;
	prox_max = 0;
	for (i = 0; i < 20; i++) {
		if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
			printk(KERN_ERR
			       "TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
			return (ret);
		}
		prox_sum += prox_cal_info[i].prox_data;
		if (prox_cal_info[i].prox_data > prox_max)
			prox_max = prox_cal_info[i].prox_data;
		mdelay(100);
	}
	prox_mean = prox_sum / 20;
	taos_cfgp->prox_threshold_hi =
	    ((((prox_max - prox_mean) * 200) + 50) / 100) + prox_mean;
	taos_cfgp->prox_threshold_lo =
	    ((((prox_max - prox_mean) * 170) + 50) / 100) + prox_mean;
	if (taos_cfgp->prox_threshold_hi < prox_threshold_hi_param)
		taos_cfgp->prox_threshold_hi =
		    (taos_cfgp->prox_threshold_hi +
		     prox_threshold_hi_param) / 2;
	if (taos_cfgp->prox_threshold_lo > prox_threshold_lo_param)
		taos_cfgp->prox_threshold_lo =
		    (taos_cfgp->prox_threshold_lo +
		     prox_threshold_lo_param) / 2;
	prox_mean = taos_cfgp->prox_threshold_hi - taos_cfgp->prox_threshold_lo;
	if (prox_mean < 30) {
		taos_cfgp->prox_threshold_hi += prox_mean / 2;
		if (taos_cfgp->prox_threshold_lo > prox_mean)
			taos_cfgp->prox_threshold_lo -= prox_mean / 2;
	}
	if (taos_cfgp->prox_threshold_lo > prox_threshold_lo_param * 3) {
		taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
		taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
	}
#else
	taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
	taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
#endif

	for (i = 0; i < sizeof(taos_triton_reg_init); i++) {
		if (i != 11) {
			if ((ret = taos_write_byte(taos_datap->client,
					     TAOS_TRITON_CNTRL +
					     i, taos_triton_reg_init[i])) < 0) {
				printk(KERN_ERR
				       "TAOS: write reg_init failed in ioctl\n");
				return (ret);
			}
		}
	}
	/*printk("taos_cfgp->prox_threshold_hi = %d\n", taos_cfgp->prox_threshold_hi);*/
	/*printk("taos_cfgp->prox_threshold_lo = %d\n", taos_cfgp->prox_threshold_lo);*/
	return ret;
}

static int taos_als_calibrate(void)
{
	int ret = 0;
	int lux_val = 0;
	u16 gain_trim_val = 0;
	u8 reg_val = 0;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk(KERN_ERR
		       "TAOS: read TAO_TRITON_CNTRL failed in ioctl als_off\n");
		return (ret);
	}
	if ((reg_val & 0x07) != 0x07) {
		return -ENODATA;
	}
	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_STATUS)) < 0) {
		printk(KERN_ERR
		       "TAOS: read TAO_TRITON_CNTRL failed in ioctl als_off\n");
		return (ret);
	}
	if ((reg_val & 0x01) != 0x01) {
		return -ENODATA;
	}
	if ((lux_val = taos_get_lux()) < 0) {
		printk(KERN_ERR
		       "TAOS: call to lux_val() returned error %d in ioctl als_data\n",
		       lux_val);
		return (lux_val);
	}
	gain_trim_val = (u16) (((taos_cfgp->calibrate_target) * 512) / lux_val);
	taos_cfgp->gain_trim = (int)gain_trim_val;
	return ((int)gain_trim_val);
}

// ioctls
static long taos_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int lux_val = 0, ret = 0, tmp = 0;
	u8 reg_val = 0;		//, reg_cntrl = 0;
	u8 reg_val_temp = 0;

	switch (cmd) {
	case TAOS_IOCTL_SENSOR_CHECK:
		reg_val_temp = 0;
		if ((reg_val_temp = taos_read_byte(taos_datap->client,
				    TAOS_TRITON_CNTRL)) < 0) {
			printk(KERN_ERR
			       "TAOS: TAOS_IOCTL_SENSOR_CHECK failed\n");
			return (ret);
		}
		printk
		    ("TAOS: TAOS_IOCTL_SENSOR_CHECK,prox_adc_time,%d~\n",
		     reg_val_temp);
		if ((reg_val_temp & 0xFF) == 0xF)
			return -ENODATA;

		break;
	case TAOS_IOCTL_SENSOR_CONFIG:
		ret = copy_from_user(taos_cfgp, (struct taos_cfg *)arg,
				     sizeof(struct taos_cfg));
		if (ret) {
			printk(KERN_ERR
			       "TAOS: copy_from_user failed in ioctl config_set\n");
			return -ENODATA;
		}
		break;
	case TAOS_IOCTL_SENSOR_ON:
		return sensor_on();
		break;
	case TAOS_IOCTL_SENSOR_OFF:
		return sensor_off();
		break;
	case TAOS_IOCTL_ALS_ON:
		return taos_als_power_on(taos_datap);
		break;
	case TAOS_IOCTL_ALS_OFF:
		return taos_als_power_off(taos_datap);
		break;
	case TAOS_IOCTL_ALS_DATA:
		if ((reg_val = taos_read_byte(taos_datap->client,
				    TAOS_TRITON_CNTRL)) < 0) {
			printk(KERN_ERR
			       "TAOS: read TAO_TRITON_CNTRL failed in ioctl als_off\n");
			return (ret);
		}
		if ((reg_val &
		     (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
		    != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
			return -ENODATA;
		if ((reg_val = taos_read_byte(taos_datap->client,
				    TAOS_TRITON_STATUS)) < 0) {
			printk(KERN_ERR
			       "TAOS: read TAO_TRITON_CNTRL failed in ioctl als_off\n");
			return (ret);
		}
		if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) !=
		    TAOS_TRITON_STATUS_ADCVALID)
			return -ENODATA;
		if ((lux_val = taos_get_lux()) < 0)
			printk(KERN_ERR
			       "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n",
			       lux_val);
		return (lux_val);
		break;
	case TAOS_IOCTL_ALS_CALIBRATE:
		return taos_als_calibrate();
		break;
	case TAOS_IOCTL_CONFIG_GET:
		ret = copy_to_user((struct taos_cfg *)arg, taos_cfgp,
				 sizeof(struct taos_cfg));
		if (ret) {
			printk(KERN_ERR
			       "TAOS: copy_to_user failed in ioctl config_get\n");
			return -ENODATA;
		}
		return (ret);
		break;
	case TAOS_IOCTL_CONFIG_SET:
		ret = copy_from_user(taos_cfgp, (struct taos_cfg *)arg,
				   sizeof(struct taos_cfg));
		if (ret) {
			printk(KERN_ERR
			       "TAOS: copy_from_user failed in ioctl config_set\n");
			return -ENODATA;
		}
		if (taos_cfgp->als_time < 50)
			taos_cfgp->als_time = 50;
		if (taos_cfgp->als_time > 650)
			taos_cfgp->als_time = 650;
		tmp = (taos_cfgp->als_time + 25) / 50;
		taos_cfgp->als_time = tmp * 50;
		sat_als = (256 - taos_cfgp->prox_int_time) << 10;
		sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;
		break;
	case TAOS_IOCTL_PROX_ON:
		sensor_on();
		taos_ps_calibrate();
		return taos_ps_power_on(taos_datap);
		break;
	case TAOS_IOCTL_PROX_OFF:
		return taos_ps_power_off(taos_datap);
		break;
	case TAOS_IOCTL_PROX_DATA:
		if ((ret = taos_prox_poll(prox_cur_infop)) < 0) {
			printk(KERN_ERR
			       "TAOS: call to prox_poll failed in prox_data\n");
			return ret;
		}
		ret = copy_to_user((struct taos_prox_info *)arg,
				 prox_cur_infop, sizeof(struct taos_prox_info));
		if (ret) {
			printk(KERN_ERR
			       "TAOS: copy_to_user failed in ioctl prox_data\n");
			return -ENODATA;
		}
		return (ret);
		break;
	case TAOS_IOCTL_PROX_CALIBRATE:
		return taos_ps_calibrate();
		break;
	default:
		return -EINVAL;
		break;
	}
	return (ret);
}

// read/calculate lux value
static int taos_get_lux(void)
{
	u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0;
	u32 ratio = 0;
	u8 dev_gain = 0;
	u16 Tint = 0;
	struct lux_data *p;
	int ret = 0;
	u8 chdata[4];
	int tmp = 0, i = 0;

	for (i = 0; i < 4; i++) {
		if ((chdata[i] = (taos_read_byte
		      (taos_datap->client, TAOS_TRITON_ALS_CHAN0LO + i))) < 0) {
			printk(KERN_ERR
			       "TAOS: read chan0lo/li failed in taos_get_lux()\n");
			return (ret);
		}
		printk("--GGG--chdata is %d\n",chdata[i]);
	}

	/*if atime =100  tmp = (atime+25)/50=2.5   tine = 2.7*(256-atime)=  412.5*/
	tmp = (taos_cfgp->als_time + 25) / 50;
	TritonTime.numerator = 1;
	TritonTime.denominator = tmp;

	//tmp = 300*atime  400
	tmp = 300 * taos_cfgp->als_time;
	if (tmp > 65535)
		tmp = 65535;
	TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir = chdata[3];
	raw_ir <<= 8;
	raw_ir |= chdata[2];

	raw_clear *= (taos_cfgp->scale_factor);
	raw_ir *= (taos_cfgp->scale_factor);

	if (raw_ir > raw_clear) {
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}
	dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
	if (raw_clear >= lux_timep->saturation)
		return (TAOS_MAX_LUX);
	if (raw_ir >= lux_timep->saturation)
		return (TAOS_MAX_LUX);
	if (raw_clear == 0)
		return (0);
	if (dev_gain == 0 || dev_gain > 127) {
		printk(KERN_ERR
		       "TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
		return -1;
	}
	if (lux_timep->denominator == 0) {
		printk(KERN_ERR
		       "TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
		return -1;
	}
	ratio = (raw_ir << 15) / raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++) ;
	if (!p->ratio) {
		if (lux_history[0] < 0)
			return 0;
		else
			return lux_history[0];
	}
	Tint = taos_cfgp->als_time;
	raw_clear = ((raw_clear * 400 + (dev_gain >> 1)) / dev_gain +
			(Tint >> 1)) / Tint;
	raw_ir = ((raw_ir * 400 + (dev_gain >> 1)) / dev_gain + (Tint >> 1)) / Tint;
	lux = ((raw_clear * (p->clear)) - (raw_ir * (p->ir)));
	lux = (lux + 32000) / 64000;
	if (lux > TAOS_MAX_LUX) {
		lux = TAOS_MAX_LUX;
	}
	return (lux);
}

// verify device
static int taos_device_name(unsigned char *bufp, char **device_name)
{
	if ((bufp[0x12] & 0xf0) == 0x00)
		return (0);
	if (bufp[0x10] | bufp[0x1a] | bufp[0x1b] | bufp[0x1c] | bufp[0x1d] |
	    bufp[0x1e])
		return (0);
	if (bufp[0x13] & 0x0c)
		return (0);
	*device_name = "tmd2771";
	return (1);
}

// proximity poll
static int taos_prox_poll(struct taos_prox_info *prxp)
{
	int i = 0, ret = 0;
	u8 chdata[6];
	for (i = 0; i < 6; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_CMD_AUTO |
				   (TAOS_TRITON_ALS_CHAN0LO + i));
		if (chdata[i] < 0) {
			printk("ERROR : taos_read_byte chdata[%d]\n", i);
		}
	}
	prxp->prox_clear = chdata[1];
	prxp->prox_clear <<= 8;
	prxp->prox_clear |= chdata[0];
	prxp->prox_data = chdata[5];
	prxp->prox_data <<= 8;
	prxp->prox_data |= chdata[4];

	return (ret);
}

MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(taos_init);
module_exit(taos_exit);
