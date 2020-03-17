// SPDX-License-Identifier: GPL-2.0
/*
 * 3-axis accelerometer driver supporting following Bosch-Sensortec chips:
 *  - BMI088
 *
 * Copyright (c) 2018-2020, Topic Embedded Products
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regmap.h>
#include <asm/unaligned.h>

#include "bmi088-accel.h"

#define BMI088_ACCEL_DRV_NAME	"bmi088_accel"
#define BMI088_ACCEL_IRQ_NAME	"bmi088_accel_event"

#define BMI088_ACCEL_REG_CHIP_ID			0x00

#define BMI088_ACCEL_REG_INT_STATUS			0x1D
#define BMI088_ACCEL_INT_STATUS_BIT_DRDY		BIT(7)

#define BMI088_ACCEL_REG_RESET				0x7E
#define BMI088_ACCEL_RESET_VAL				0xB6

#define BMI088_ACCEL_REG_PWR_CTRL			0x7D
#define BMI088_ACCEL_REG_PWR_CONF			0x7C

#define BMI088_ACCEL_REG_INT_MAP_DATA			0x58
#define BMI088_ACCEL_INT_MAP_DATA_BIT_INT1_DRDY		BIT(2)
#define BMI088_ACCEL_INT_MAP_DATA_BIT_INT2_FWM		BIT(5)

#define BMI088_ACCEL_REG_INT1_IO_CONF			0x53
#define BMI088_ACCEL_INT1_IO_CONF_BIT_ENABLE_OUT	BIT(3)
#define BMI088_ACCEL_INT1_IO_CONF_BIT_LVL		BIT(1)

#define BMI088_ACCEL_REG_INT2_IO_CONF			0x54
#define BMI088_ACCEL_INT2_IO_CONF_BIT_ENABLE_OUT	BIT(3)
#define BMI088_ACCEL_INT2_IO_CONF_BIT_LVL		BIT(1)

#define BMI088_ACCEL_REG_ACC_CONF			0x40
#define BMI088_ACCEL_REG_ACC_RANGE			0x41
#define BMI088_ACCEL_RANGE_3G				0x00
#define BMI088_ACCEL_RANGE_6G				0x01
#define BMI088_ACCEL_RANGE_12G				0x02
#define BMI088_ACCEL_RANGE_24G				0x03

#define BMI088_ACCEL_REG_TEMP				0x22
#define BMI088_ACCEL_TEMP_CENTER_VAL			23
#define BMI088_ACCEL_TEMP_UNIT				125

#define BMI088_ACCEL_REG_XOUT_L				0x12
#define BMI088_ACCEL_AXIS_TO_REG(axis) \
	(BMI088_ACCEL_REG_XOUT_L + (axis * 2))

#define BMI088_ACCEL_MAX_STARTUP_TIME_MS		1
#define BMI088_AUTO_SUSPEND_DELAY_MS			2000

#define BMI088_ACCEL_REG_FIFO_STATUS			0x0E
#define BMI088_ACCEL_REG_FIFO_CONFIG0			0x48
#define BMI088_ACCEL_REG_FIFO_CONFIG1			0x49
#define BMI088_ACCEL_REG_FIFO_DATA			0x3F
#define BMI088_ACCEL_FIFO_LENGTH			100

#define BMI088_ACCEL_FIFO_MODE_FIFO			0x40
#define BMI088_ACCEL_FIFO_MODE_STREAM			0x80

enum bmi088_accel_axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_MAX,
};

enum bmi088_power_modes {
	BMI088_ACCEL_MODE_ACTIVE,
	BMI088_ACCEL_MODE_SUSPEND,
};

/* Available OSR (over sampling rate) sets the 3dB cut-off frequency */
enum bmi088_osr_modes {
	BMI088_ACCEL_MODE_OSR_NORMAL = 0xA,
	BMI088_ACCEL_MODE_OSR_2 = 0x9,
	BMI088_ACCEL_MODE_OSR_4 = 0x8,
};

/* Available ODR (output data rates) in Hz */
enum bmi088_odr_modes {
	BMI088_ACCEL_MODE_ODR_12_5 = 0x5,
	BMI088_ACCEL_MODE_ODR_25 = 0x6,
	BMI088_ACCEL_MODE_ODR_50 = 0x7,
	BMI088_ACCEL_MODE_ODR_100 = 0x8,
	BMI088_ACCEL_MODE_ODR_200 = 0x9,
	BMI088_ACCEL_MODE_ODR_400 = 0xa,
	BMI088_ACCEL_MODE_ODR_800 = 0xb,
	BMI088_ACCEL_MODE_ODR_1600 = 0xc,
};

struct bmi088_scale_info {
	int scale;
	u8 reg_range;
};

struct bmi088_accel_chip_info {
	const char *name;
	u8 chip_id;
	const struct iio_chan_spec *channels;
	int num_channels;
};

struct bmi088_accel_data {
	struct regmap *regmap;
	struct mutex mutex;
	const struct bmi088_accel_chip_info *chip_info;
	u8 buffer[2] ____cacheline_aligned;
};

const struct regmap_config bmi088_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7E,
};
EXPORT_SYMBOL_GPL(bmi088_regmap_conf);


#ifdef CONFIG_PM
static int bmi088_accel_set_power_state(struct bmi088_accel_data *data,
	bool on)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	if (on) {
		ret = pm_runtime_get_sync(dev);
	} else {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_autosuspend(dev);
	}

	if (ret < 0) {
		dev_err(dev, "Failed: %s(%d)\n", __func__, on);
		if (on)
			pm_runtime_put_noidle(dev);

		return ret;
	}

	return 0;
}
#else
static int bmi088_accel_set_power_state(struct bmi088_accel_data *data,
	bool on)
{
	return 0;
}
#endif

static int bmi088_accel_enable(struct bmi088_accel_data *data,
				bool on_off)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = regmap_write(data->regmap, BMI088_ACCEL_REG_PWR_CTRL,
				on_off ? 0x4 : 0x0);
	if (ret < 0) {
		dev_err(dev, "Error writing ACC_PWR_CTRL reg\n");
		return ret;
	}

	return 0;
}

static int bmi088_accel_set_mode(struct bmi088_accel_data *data,
				enum bmi088_power_modes mode)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = regmap_write(data->regmap, BMI088_ACCEL_REG_PWR_CONF,
			   mode == BMI088_ACCEL_MODE_SUSPEND ? 0x3 : 0x0);
	if (ret < 0) {
		dev_err(dev, "Error writing ACCEL_PWR_CONF reg\n");
		return ret;
	}

	return 0;
}

static int bmi088_accel_set_bw(struct bmi088_accel_data *data,
				enum bmi088_odr_modes odr_mode,
				enum bmi088_osr_modes osr_mode)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;
	u8 value = (osr_mode << 4) | (odr_mode & 0xF);

	ret = regmap_write(data->regmap, BMI088_ACCEL_REG_ACC_CONF, value);
	if (ret < 0) {
		dev_err(dev, "Error writing ACCEL_PWR_CONF reg\n");
		return ret;
	}

	return 0;
}

static int bmi088_accel_get_sample_freq(struct bmi088_accel_data *data,
					int* val, int *val2)
{
	unsigned int value;
	int ret;

	ret = regmap_read(data->regmap, BMI088_ACCEL_REG_ACC_CONF,
			  &value);
	if (ret < 0)
		return ret;

	value &= 0xf; /* ODR in lower 4 bits */
	if (value == BMI088_ACCEL_MODE_ODR_12_5) {
		*val = 12;
		*val2 = 500000;
	} else {
		*val = 25 << (value - BMI088_ACCEL_MODE_ODR_25);
		*val2 = 0;
	}

	return IIO_VAL_INT_PLUS_MICRO;
}

static int bmi088_accel_set_sample_freq(struct bmi088_accel_data *data, int val)
{
	unsigned int value = BMI088_ACCEL_MODE_ODR_1600;
	unsigned int freq = 1600;
	int ret;

	if (val < 12 || val > 1600)
		return -EINVAL;

	while (freq >= val && value > BMI088_ACCEL_MODE_ODR_12_5) {
		--value;
		freq >>= 1;
	}

	ret = regmap_update_bits(data->regmap, BMI088_ACCEL_REG_ACC_CONF,
				 0x0f, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int bmi088_accel_get_temp(struct bmi088_accel_data *data, int *val)
{
	int ret;
	__u16 temp;

	mutex_lock(&data->mutex);

	ret = regmap_bulk_read(data->regmap, BMI088_ACCEL_REG_TEMP,
			       &data->buffer, 2);
	temp = get_unaligned_be16(data->buffer);

	mutex_unlock(&data->mutex);

	if (ret < 0)
		return ret;

	*val = sign_extend32(temp, 11);

	return IIO_VAL_INT;
}

static int bmi088_accel_get_axis(struct bmi088_accel_data *data,
				 struct iio_chan_spec const *chan,
				 int *val)
{
	int ret;
	__u16 raw_val;

	mutex_lock(&data->mutex);

	ret = bmi088_accel_set_power_state(data, true);
	if (ret < 0)
		return ret;

	ret = regmap_bulk_read(data->regmap,
			       BMI088_ACCEL_AXIS_TO_REG(chan->scan_index),
			       data->buffer, 2);
	raw_val = get_unaligned_be16(data->buffer);

	bmi088_accel_set_power_state(data, false);

	mutex_unlock(&data->mutex);

	if (ret < 0)
		return ret;

	*val = sign_extend32(raw_val, 16);

	return IIO_VAL_INT;
}

static int bmi088_accel_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct bmi088_accel_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_TEMP:
			return bmi088_accel_get_temp(data, val);
		case IIO_ACCEL:
			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret < 0)
				return ret;

			ret = bmi088_accel_get_axis(data, chan, val);
			iio_device_release_direct_mode(indio_dev);
			if (ret < 0)
				return ret;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = BMI088_ACCEL_TEMP_CENTER_VAL;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		switch (chan->type) {
		case IIO_TEMP:
			*val = BMI088_ACCEL_TEMP_UNIT;
			return IIO_VAL_INT;
		case IIO_ACCEL:
		{
			ret = regmap_read(data->regmap,
					BMI088_ACCEL_REG_ACC_RANGE, val);
			if (ret < 0)
				return ret;

			*val2 =  15 - (*val & 0x3);
			*val = 3 * 980;

			return IIO_VAL_FRACTIONAL_LOG2;
		}
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmi088_accel_get_sample_freq(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
}

static int bmi088_accel_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int val, int val2, long mask)
{
	struct bmi088_accel_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmi088_accel_set_sample_freq(data, val);
		mutex_unlock(&data->mutex);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("12.5 25 50 100 200 400 800 1600");

static struct attribute *bmi088_accel_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group bmi088_accel_attrs_group = {
	.attrs = bmi088_accel_attributes,
};

#define BMI088_ACCEL_CHANNEL(_axis) {					\
	.type = IIO_ACCEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.scan_index = AXIS_##_axis,					\
}

static const struct iio_chan_spec bmi088_accel_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = -1,
	},
	BMI088_ACCEL_CHANNEL(X),
	BMI088_ACCEL_CHANNEL(Y),
	BMI088_ACCEL_CHANNEL(Z),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct bmi088_accel_chip_info bmi088_accel_chip_info_tbl[] = {
	[0] = {
		.name = "bmi088a",
		.chip_id = 0x1E,
		.channels = bmi088_accel_channels,
		.num_channels = ARRAY_SIZE(bmi088_accel_channels),
	},
};

static const struct iio_info bmi088_accel_info = {
	.attrs		= &bmi088_accel_attrs_group,
	.read_raw	= bmi088_accel_read_raw,
	.write_raw	= bmi088_accel_write_raw,
};

static const unsigned long bmi088_accel_scan_masks[] = {
				BIT(AXIS_X) | BIT(AXIS_Y) | BIT(AXIS_Z),
				0};


static int bmi088_accel_chip_init(struct bmi088_accel_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret, i;
	unsigned int val;

	/* Do a dummy read (of chip ID), to enable SPI interface */
	regmap_read(data->regmap, BMI088_ACCEL_REG_CHIP_ID, &val);

	/*
	 * Reset chip to get it in a known good state. A delay of 1ms after
	 * reset is required according to the data sheet
	 */
	regmap_write(data->regmap, BMI088_ACCEL_REG_RESET,
		     BMI088_ACCEL_RESET_VAL);
	usleep_range(1000, 2000);

	/*
	 * Do a dummy read (of chip ID), to enable SPI interface after reset
	 * and does no harm if using I2C.
	 */
	regmap_read(data->regmap, BMI088_ACCEL_REG_CHIP_ID, &val);

	/* Read chip ID */
	ret = regmap_read(data->regmap, BMI088_ACCEL_REG_CHIP_ID, &val);
	if (ret < 0) {
		dev_err(dev, "Error: Reading chip id\n");
		return ret;
	}

	/* Validate chip ID */
	dev_dbg(dev, "Chip Id %x\n", val);
	for (i = 0; i < ARRAY_SIZE(bmi088_accel_chip_info_tbl); i++) {
		if (bmi088_accel_chip_info_tbl[i].chip_id == val) {
			data->chip_info = &bmi088_accel_chip_info_tbl[i];
			break;
		}
	}

	if (!data->chip_info) {
		dev_err(dev, "Invalid chip %x\n", val);
		return -ENODEV;
	}

	/* Set Active mode (and wait for 5ms) */
	ret = bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_ACTIVE);
	if (ret < 0)
		return ret;

	usleep_range(5000, 10000);

	/* Enable accelerometer */
	ret = bmi088_accel_enable(data, true);
	if (ret < 0)
		return ret;

	/* Set Bandwidth */
	ret = bmi088_accel_set_bw(data, BMI088_ACCEL_MODE_ODR_100,
				  BMI088_ACCEL_MODE_OSR_NORMAL);
	if (ret < 0)
		return ret;

	/* Set Default Range */
	ret = regmap_write(data->regmap, BMI088_ACCEL_REG_ACC_RANGE,
			   BMI088_ACCEL_RANGE_6G);
	if (ret < 0)
		return ret;

	return 0;
}

int bmi088_accel_core_probe(struct device *dev, struct regmap *regmap,
	int irq, const char *name, bool block_supported)
{
	struct bmi088_accel_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	data->regmap = regmap;

	ret = bmi088_accel_chip_init(data);
	if (ret < 0)
		return ret;

	mutex_init(&data->mutex);

	indio_dev->dev.parent = dev;
	indio_dev->channels = data->chip_info->channels;
	indio_dev->num_channels = data->chip_info->num_channels;
	indio_dev->name = name ? name : data->chip_info->name;
	indio_dev->available_scan_masks = bmi088_accel_scan_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmi088_accel_info;

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, BMI088_AUTO_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "Unable to register iio device\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bmi088_accel_core_probe);

int bmi088_accel_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmi088_accel_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	mutex_lock(&data->mutex);
	bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_SUSPEND);
	mutex_unlock(&data->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(bmi088_accel_core_remove);

#ifdef CONFIG_PM_SLEEP
static int bmi088_accel_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmi088_accel_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_SUSPEND);
	mutex_unlock(&data->mutex);

	return 0;
}

static int bmi088_accel_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmi088_accel_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_ACTIVE);
	mutex_unlock(&data->mutex);

	return 0;
}
#endif

static int __maybe_unused bmi088_accel_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmi088_accel_data *data = iio_priv(indio_dev);
	int ret;

	ret = bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_SUSPEND);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}

static int __maybe_unused bmi088_accel_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmi088_accel_data *data = iio_priv(indio_dev);
	int ret;

	dev_dbg(dev,  __func__);

	ret = bmi088_accel_set_mode(data, BMI088_ACCEL_MODE_ACTIVE);
	if (ret < 0)
		return ret;

	usleep_range(BMI088_ACCEL_MAX_STARTUP_TIME_MS * 1000,
		BMI088_ACCEL_MAX_STARTUP_TIME_MS * 1000 * 2);

	return 0;
}

const struct dev_pm_ops bmi088_accel_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bmi088_accel_suspend, bmi088_accel_resume)
	SET_RUNTIME_PM_OPS(bmi088_accel_runtime_suspend,
			   bmi088_accel_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(bmi088_accel_pm_ops);

MODULE_AUTHOR("Niek van Agt <niek.van.agt@topicproducts.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BMI088 accelerometer driver (core)");
