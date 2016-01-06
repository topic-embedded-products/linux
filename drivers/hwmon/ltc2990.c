/*
 * driver for Linear Technology LTC2990 power monitor
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 *
 * License: GPLv2
 *
 * This driver assumes the chip is wired as a dual current monitor, and
 * reports the voltage drop across two series resistors.
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define LTC2990_STATUS	0x00
#define LTC2990_CONTROL	0x01
#define LTC2990_TRIGGER	0x02
#define LTC2990_TINT_MSB	0x04
#define LTC2990_TINT_LSB	0x05
#define LTC2990_V1_MSB	0x06
#define LTC2990_V1_LSB	0x07
#define LTC2990_V2_MSB	0x08
#define LTC2990_V2_LSB	0x09
#define LTC2990_V3_MSB	0x0A
#define LTC2990_V3_LSB	0x0B
#define LTC2990_V4_MSB	0x0C
#define LTC2990_V4_LSB	0x0D
#define LTC2990_VCC_MSB	0x0E
#define LTC2990_VCC_LSB	0x0F

#define LTC2990_STATUS_BUSY	BIT(0)
#define LTC2990_STATUS_TINT	BIT(1)
#define LTC2990_STATUS_V1	BIT(2)
#define LTC2990_STATUS_V2	BIT(3)
#define LTC2990_STATUS_V3	BIT(4)
#define LTC2990_STATUS_V4	BIT(5)
#define LTC2990_STATUS_VCC	BIT(6)

/* Only define control settings we actually use */
#define LTC2990_CONTROL_KELVIN		BIT(7)
#define LTC2990_CONTROL_SINGLE		BIT(6)
#define LTC2990_CONTROL_MEASURE_ALL	(0x3 << 3)
#define LTC2990_CONTROL_MODE_CURRENT	0x06
#define LTC2990_CONTROL_MODE_VOLTAGE	0x07

struct ltc2990_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	unsigned long last_updated;
	short values[6];
	bool valid;
	u8 update_counter;
};

static int ltc2990_write(struct i2c_client *i2c, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(i2c, reg, value);
}

static int ltc2990_read_byte(struct i2c_client *i2c, u8 reg)
{
	return i2c_smbus_read_byte_data(i2c, reg);
}

static int ltc2990_read_word(struct i2c_client *i2c, u8 reg)
{
	int result = i2c_smbus_read_word_data(i2c, reg);
	/* Result is MSB first, but smbus specs say LSB first, so swap the
	 * result */
	return result < 0 ? result : swab16(result);
}

static struct ltc2990_data *ltc2990_update_device(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ltc2990_data *data = i2c_get_clientdata(i2c);
	struct ltc2990_data *ret = data;
	unsigned int timeout;

	mutex_lock(&data->update_lock);

	/* Update about 4 times per second max */
	if (time_after(jiffies, data->last_updated + HZ / 4) || !data->valid) {
		int val;
		int i;

		/* Trigger ADC, any value will do */
		val = ltc2990_write(i2c, LTC2990_TRIGGER, 1);
		if (unlikely(val < 0)) {
			ret = ERR_PTR(val);
			goto abort;
		}

		/* Wait for conversion complete */
		timeout = 200;
		for (;;) {
			usleep_range(2000, 4000);
			val = ltc2990_read_byte(i2c, LTC2990_STATUS);
			if (unlikely(val < 0)) {
				ret = ERR_PTR(val);
				goto abort;
			}
			/* Single-shot mode, wait for conversion to complete */
			if ((val & LTC2990_STATUS_BUSY) == 0)
				break;
			if (--timeout == 0) {
				ret = ERR_PTR(-ETIMEDOUT);
				goto abort;
			}
		}

		/* Read all registers */
		for (i = 0; i < ARRAY_SIZE(data->values); ++i) {
			val = ltc2990_read_word(i2c, (i<<1) + LTC2990_TINT_MSB);
			if (unlikely(val < 0)) {
				dev_dbg(dev,
					"Failed to read ADC value: error %d\n",
					val);
				ret = ERR_PTR(val);
				goto abort;
			}
			data->values[i] = val & 0x7FFF; /* Strip 'new' bit */
		}
		data->last_updated = jiffies;
		data->valid = 1;

		/*
		 *  Quirk: Second trigger is ignored? After this, the BUSY will
		 * still be set to "0" and no conversion performed.
		 */
		val = ltc2990_write(i2c, LTC2990_TRIGGER, 0);
	}
abort:
	mutex_unlock(&data->update_lock);
	return ret;
}

/* Return the converted value from the given register in uV or mC */
static int ltc2990_get_value(struct ltc2990_data *data, u8 index)
{
	s32 result;
	s16 v;

	if (index == 0) { /* internal temp, 0.0625 degrees/LSB, 12-bit  */
		v = data->values[index] << 3;
		result = (s32)v * 1000 >> 7;
	} else if (index < 5) { /* Vx-Vy, 19.42uV/LSB, 14-bit */
		v = data->values[index] << 2;
		result = (s32)v * 1942 / (4 * 100);
	} else { /* Vcc, 305.18μV/LSB, 2.5V offset, 14-bit */
		v = data->values[index] << 2;
		result = (s32)v * 30518 / (4 * 100);
		result += 2500000;
	}
	return result;
}

static ssize_t ltc2990_show_value(struct device *dev,
				  struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ltc2990_data *data = ltc2990_update_device(dev);
	int value;

	if (IS_ERR(data))
		return PTR_ERR(data);

	value = ltc2990_get_value(data, attr->index);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static SENSOR_DEVICE_ATTR(temp_int, S_IRUGO, ltc2990_show_value, NULL, 0);
static SENSOR_DEVICE_ATTR(v1v2_diff, S_IRUGO, ltc2990_show_value, NULL, 1);
static SENSOR_DEVICE_ATTR(v3v4_diff, S_IRUGO, ltc2990_show_value, NULL, 3);
static SENSOR_DEVICE_ATTR(vcc, S_IRUGO, ltc2990_show_value, NULL, 5);

static struct attribute *ltc2990_attributes[] = {
	&sensor_dev_attr_temp_int.dev_attr.attr,
	&sensor_dev_attr_v1v2_diff.dev_attr.attr,
	&sensor_dev_attr_v3v4_diff.dev_attr.attr,
	&sensor_dev_attr_vcc.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltc2990_group = {
	.attrs = ltc2990_attributes,
};

static int ltc2990_i2c_probe(
	struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	struct ltc2990_data *ltc2990;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	ltc2990 = devm_kzalloc(&i2c->dev,
		sizeof(struct ltc2990_data), GFP_KERNEL);
	if (ltc2990 == NULL)
		return -ENOMEM;

	ret = ltc2990_read_byte(i2c, 0);
	if (ret < 0) {
		dev_err(&i2c->dev, "Could not read LTC2990 on i2c bus.\n");
		return ret;
	}
	ret = ltc2990_write(i2c, LTC2990_CONTROL,
		LTC2990_CONTROL_SINGLE | LTC2990_CONTROL_MEASURE_ALL |
		LTC2990_CONTROL_MODE_CURRENT);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set control mode.\n");
		return ret;
	}

	mutex_init(&ltc2990->update_lock);
	i2c_set_clientdata(i2c, ltc2990);

	/* Register sysfs hooks */
	ret = sysfs_create_group(&i2c->dev.kobj, &ltc2990_group);
	if (ret)
		return ret;

	ltc2990->hwmon_dev = hwmon_device_register(&i2c->dev);
	if (IS_ERR(ltc2990->hwmon_dev)) {
		ret = PTR_ERR(ltc2990->hwmon_dev);
		goto out_hwmon_device_register;
	}

	return 0;

out_hwmon_device_register:
	sysfs_remove_group(&i2c->dev.kobj, &ltc2990_group);
	return ret;
}

static int ltc2990_i2c_remove(struct i2c_client *i2c)
{
	struct ltc2990_data *ltc2990 = i2c_get_clientdata(i2c);

	hwmon_device_unregister(ltc2990->hwmon_dev);
	sysfs_remove_group(&i2c->dev.kobj, &ltc2990_group);
	return 0;
}

static const struct i2c_device_id ltc2990_i2c_id[] = {
	{ "ltc2990", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2990_i2c_id);

static struct i2c_driver ltc2990_i2c_driver = {
	.driver = {
		.name = "ltc2990",
	},
	.probe    = ltc2990_i2c_probe,
	.remove   = ltc2990_i2c_remove,
	.id_table = ltc2990_i2c_id,
};

module_i2c_driver(ltc2990_i2c_driver);

MODULE_DESCRIPTION("LTC2990 Sensor Driver");
MODULE_AUTHOR("Topic Embedded Products");
MODULE_LICENSE("GPL v2");
