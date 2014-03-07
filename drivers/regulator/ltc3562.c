/*
 * Regulator driver for Linear Technology LCT3562 
 *
 *  Copyright (C) 2014 Topic Embedded Products
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/regulator/of_regulator.h>


#define LTC3562_NUM_REGULATORS	4 

#define REGULATOR_TYPE_A_NUM_VOLTAGES	    16
#define REGULATOR_TYPE_A_MIN_UV		425000
#define REGULATOR_TYPE_A_UV_STEP	 25000

#define REGULATOR_TYPE_B_NUM_VOLTAGES	   128
#define REGULATOR_TYPE_B_MIN_UV		600000
#define REGULATOR_TYPE_B_UV_STEP	 25000


struct ltc3562_status {
	u8 shadow_addr;		/* Shadow register of address byte */
	u8 shadow_data;		/* Shadow register of data byte */
	u8 voltage_set;		/* Check if voltage has been set before */
};


struct ltc3562 {
	struct device *dev;
	struct i2c_client *i2c;
	struct mutex mutex;
	struct regulator_dev *rdev[LTC3562_NUM_REGULATORS];
	struct ltc3562_status rstatus[LTC3562_NUM_REGULATORS];
	
};

static int ltc3562_write(struct i2c_client *i2c, u8 reg_a, u8 reg_b);
static int ltc3562_dummy_write(struct i2c_client *i2c);

#define LTC3562_R400B_ID	0x00
#define LTC3562_R600B_ID	0x01
#define LTC3562_R400A_ID	0x02
#define LTC3562_R600A_ID	0x03

#define OPERATING_MODE_MASK	0x03

#define PROGRAM_R400B_MASK	0x10
#define PROGRAM_R600B_MASK	0x20
#define PROGRAM_R400A_MASK	0x40
#define PROGRAM_R600A_MASK	0x80

#define OPERATING_MODE_PULSE_SKIP	0x00
#define OPERATING_MODE_LDO		0x01
#define OPERATING_MODE_FOCED_BURST	0x02
#define OPERATING_MODE_BURST		0x03

#define REGULATOR_ENABLE_BIT	0x80


static int ltc3562_regulator_enable(struct regulator_dev *dev){
	int ret, v_index;
	unsigned int v_default;
	struct ltc3562 *ltc3562 = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	struct ltc3562_status *status = &ltc3562->rstatus[id];

	mutex_lock(&ltc3562->mutex);

	if(!status->voltage_set){
		if(of_property_read_u32(dev->dev.of_node, "ltc3562-default-voltage", &v_default) == 0){
			v_index = dev->desc->ops->map_voltage(dev,v_default , v_default);
			if(v_index > 0){
				status->shadow_data = v_index & (~REGULATOR_ENABLE_BIT);
			}
			status->voltage_set = 1;
		}

	}

	status->shadow_data |= REGULATOR_ENABLE_BIT;

	ret = ltc3562_write(ltc3562->i2c, status->shadow_addr | PROGRAM_R400B_MASK << id, status->shadow_data);

	mutex_unlock(&ltc3562->mutex); 

	return ret;
}

static int ltc3562_regulator_disable(struct regulator_dev *dev){
	int ret;
	struct ltc3562 *ltc3562 = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	struct ltc3562_status *status = &ltc3562->rstatus[id];

	mutex_lock(&ltc3562->mutex);
	
	status->shadow_data &= ~(REGULATOR_ENABLE_BIT);

	ret = ltc3562_write(ltc3562->i2c, status->shadow_addr, status->shadow_data);

	mutex_unlock(&ltc3562->mutex); 

	return ret;
}

static int ltc3562_regulator_is_enabled(struct regulator_dev *dev){
	int ret;
	struct ltc3562 *ltc3562 = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	struct ltc3562_status *status = &ltc3562->rstatus[id];

	mutex_lock(&ltc3562->mutex);

	ret = (status->shadow_data & REGULATOR_ENABLE_BIT) ? 1 : 0;

	mutex_unlock(&ltc3562->mutex);

	return ret; 
}

static int ltc3562_set_voltage_sel(struct regulator_dev *dev, unsigned int selector){
	int ret;
	struct ltc3562 *ltc3562 = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	struct ltc3562_status *status = &ltc3562->rstatus[id];

	mutex_lock(&ltc3562->mutex);
	
	status->shadow_data = (status->shadow_data & REGULATOR_ENABLE_BIT) | selector;
	status->voltage_set = 1;

	ret = ltc3562_write(ltc3562->i2c, status->shadow_addr, status->shadow_data);

	mutex_unlock(&ltc3562->mutex); 

	return ret;
}

static int ltc3562_get_voltage_sel(struct regulator_dev *dev){
	int ret;
	struct ltc3562 *ltc3562 = rdev_get_drvdata(dev);
	int id = rdev_get_id(dev);
	struct ltc3562_status *status = &ltc3562->rstatus[id];

	mutex_lock(&ltc3562->mutex);
	
	ret = status->shadow_data & ~(REGULATOR_ENABLE_BIT);

	mutex_unlock(&ltc3562->mutex); 

	return ret;
}

static struct regulator_ops ltc3562_regulator_ops = {
	.is_enabled = ltc3562_regulator_is_enabled,
	.enable = ltc3562_regulator_enable,
	.disable = ltc3562_regulator_disable,
	.set_voltage_sel = ltc3562_set_voltage_sel,
	.get_voltage_sel = ltc3562_get_voltage_sel,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
};


static struct regulator_desc ltc3562_regulators[] = {
	{
		.name = "R400B",
		.id = LTC3562_R400B_ID,
		.ops = &ltc3562_regulator_ops,
		.min_uV = REGULATOR_TYPE_B_MIN_UV,
		.uV_step = REGULATOR_TYPE_B_UV_STEP, 
		.n_voltages = REGULATOR_TYPE_B_NUM_VOLTAGES,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "R600B",
		.id = LTC3562_R600B_ID,
		.ops = &ltc3562_regulator_ops,
		.min_uV = REGULATOR_TYPE_B_MIN_UV,
		.uV_step = REGULATOR_TYPE_B_UV_STEP, 
		.n_voltages = REGULATOR_TYPE_B_NUM_VOLTAGES,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "R400A",
		.id = LTC3562_R400A_ID,
		.ops = &ltc3562_regulator_ops,
		.min_uV = REGULATOR_TYPE_A_MIN_UV,
		.uV_step = REGULATOR_TYPE_A_UV_STEP, 
		.n_voltages = REGULATOR_TYPE_A_NUM_VOLTAGES,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "R600A",
		.id = LTC3562_R600A_ID,
		.ops = &ltc3562_regulator_ops,
		.min_uV = REGULATOR_TYPE_A_MIN_UV,
		.uV_step = REGULATOR_TYPE_A_UV_STEP, 
		.n_voltages = REGULATOR_TYPE_A_NUM_VOLTAGES,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};


static int ltc3562_write(struct i2c_client *i2c, u8 reg_a, u8 reg_b){
	return i2c_smbus_write_byte_data(i2c, reg_a, reg_b);
}

/* Write dummy data to detect presence of physical device */
static int ltc3562_dummy_write(struct i2c_client *i2c){
	return ltc3562_write(i2c, 0, 0);
}


static int ltc3562_regulators_unregister(struct ltc3562 *ltc3562, int num_regulators){
	while(--num_regulators >= 0){
		pr_debug("Unregistering regulator: %s\n", ltc3562->rdev[num_regulators]->desc->name);
		regulator_unregister(ltc3562->rdev[num_regulators]);
	}
	return 0;
}

static int ltc3562_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id){
	int i, error;
	unsigned int r1, r2;
	unsigned long uV;
	struct ltc3562 *ltc3562;
	struct regulator_config rconfig = { };
	struct device_node *np_regulators, *np;
	struct device_node *np_child;


	ltc3562 = devm_kzalloc(&i2c->dev, sizeof(struct ltc3562), GFP_KERNEL);
	if (ltc3562 == NULL)
		return -ENOMEM;

	ltc3562->i2c = i2c;

	mutex_init(&ltc3562->mutex);

	if(ltc3562_dummy_write(i2c) < 0){
		dev_err(&i2c->dev, "Could not find device LTC3562 on i2c bus!\n");
		error = -ENODEV;
		goto lbl_exit;
	}

	np = of_node_get(i2c->dev.of_node);
	np_regulators = of_get_child_by_name(np, "regulators");

	if(np_regulators == NULL){
		dev_err(&i2c->dev, "Could not find regulators node!\n");
		error = -EINVAL;
		goto lbl_exit;
	}

	for(i = 0; i < LTC3562_NUM_REGULATORS; ++i){
		np_child = of_get_child_by_name(np_regulators, ltc3562_regulators[i].name);
		if(np_child == NULL){
			dev_err(&i2c->dev, "Could not find regulator data for %s in devicetree!\n", ltc3562_regulators[i].name);
			error = -EINVAL;
			goto lbl_cleanup;
		}

		if((ltc3562_regulators[i].id == LTC3562_R400A_ID) || (ltc3562_regulators[i].id == LTC3562_R600A_ID )){
			/* If regulator is A type get resistor values */
			error = of_property_read_u32(np_child, "ltc3562-A-r1", &r1);
			if(error){
				dev_err(&i2c->dev, "Resistor value ltc3562-A-r1 for A-Type regulator %s not defined!", ltc3562_regulators[i].name);
				goto lbl_cleanup;
			}
			
			error = of_property_read_u32(np_child, "ltc3562-A-r2", &r2);
			if(error){
				dev_err(&i2c->dev, "Resistor value ltc3562-A-r2 for A-Type regulator %s not defined!", ltc3562_regulators[i].name);
				goto lbl_cleanup;
			}
			uV = ltc3562_regulators[i].min_uV / 1000;
			ltc3562_regulators[i].min_uV = (uV + mult_frac(uV, r1, r2)) * 1000;

			uV = ltc3562_regulators[i].uV_step / 1000;
			ltc3562_regulators[i].uV_step = (uV + mult_frac(uV, r1, r2)) * 1000;
		}

		rconfig.dev = &i2c->dev;
		rconfig.init_data = of_get_regulator_init_data(&i2c->dev, np_child);
		rconfig.driver_data = ltc3562;	
		rconfig.of_node = np_child;
		
		ltc3562->rstatus[i].voltage_set = 0;

		ltc3562->rdev[i] = regulator_register(&ltc3562_regulators[i], &rconfig);

		if(IS_ERR(ltc3562->rdev[i])){
			error = PTR_ERR(ltc3562->rdev[i]);
			dev_err(&i2c->dev, "LTC3562 could not register regulator! Error # %d\n", error);			
			goto lbl_cleanup;
		}
	}	

	i2c_set_clientdata(i2c, ltc3562);
	dev_dbg(&i2c->dev, "LTC3562 Driver loaded.\n");
	return 0;

lbl_cleanup:
	ltc3562_regulators_unregister(ltc3562, i);
lbl_exit:
	return error;
}

static int ltc3562_i2c_remove(struct i2c_client *i2c)
{
	struct ltc3562 *ltc3562 = i2c_get_clientdata(i2c);

	ltc3562_regulators_unregister(ltc3562, LTC3562_NUM_REGULATORS);

	return 0;
}

static const struct i2c_device_id ltc3562_i2c_id[] = {
       { "ltc3562", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, ltc3562_i2c_id);

static struct i2c_driver ltc3562_i2c_driver = {
	.driver = {
		.name = "LTC3562",
		.owner = THIS_MODULE,
	},
	.probe    = ltc3562_i2c_probe,
	.remove   = ltc3562_i2c_remove,
	.id_table = ltc3562_i2c_id,
};

module_i2c_driver(ltc3562_i2c_driver);

MODULE_DESCRIPTION("LTC3562 Regulator Driver");
MODULE_AUTHOR("auryn.verwegen@topic.nl");
MODULE_AUTHOR("mike.looijmans@topic.nl");
MODULE_LICENSE("GPL v2");
