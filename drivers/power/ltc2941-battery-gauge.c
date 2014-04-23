/*
 * I2C client/driver for the Linear Technology LTC2941 Battery Gas Gauge IC 
 *
 * Copyright (C) 2014 Topic Embedded Systems
 *
 * Author: Auryn Verwegen
 *
 * 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/swab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/power_supply.h>
#include <linux/slab.h>


#define I16_MSB(x)			((x >> 8) & 0xFF)
#define I16_LSB(x)			(x & 0xFF)

#define LTC2941_WORK_DELAY		10	// Update delay in seconds

#define LTC2941_MAX_VALUE		0xFFFF
#define LTC2941_MID_SUPPLY		0x7FFF

#define LTC2941_DEFAULT_ALERT_LOW	0x0000
#define LTC2941_DEFAULT_ALERT_HIGH	0xFFFF

#define LTC2941_MAGIC_MASK		(BIT(7) | BIT(6))
#define LTC2941_MAGIC			(BIT(7))
#define LTC2941_GET_MAGIC(x) 		(x & LTC2941_MAGIC_MASK)

#define LTC2941_MAX_PRESCALER_EXP	7



typedef enum{
	LTC2941_REG_STATUS		= 0x00,
	LTC2941_REG_CONTROL		= 0x01,
	LTC2941_REG_ACC_CHARGE_MSB	= 0x02,
	LTC2941_REG_ACC_CHARGE_LSB	= 0x03,
	LTC2941_REG_THRESH_HIGH_MSB	= 0x04,
	LTC2941_REG_THRESH_HIGH_LSB	= 0x05,
	LTC2941_REG_THRESH_LOW_MSB	= 0x06,
	LTC2941_REG_THRESH_LOW_LSB	= 0x07,
} LTC2941_REG;

#define LTC2941_REG_CONTROL_PRESCALER_MASK	(BIT(5) | BIT(4) | BIT(3))
#define LTC2941_REG_CONTROL_SHUTDOWN_MASK	(BIT(0))

#define LTC2941_SET_PRESCALER(x)		((x << 3) & LTC2941_REG_CONTROL_PRESCALER_MASK)



//struct ltc2941_info;

struct ltc2941_info {
	struct i2c_client	*client;	// I2C Client pointer
	struct power_supply	supply;		// Supply pointer
	struct delayed_work	work;		// Work scheduler
	int			id;		// Identifier of ltc2941 chip
	int 			charge;		// Last measures charge value (uAh)
	int 			PrescalerM;	// M, 2^(0..7)
	int			rSense;		// mOhm
	int 			Qlsb;		// nAh
	int			alert_low;	// uAh
	int			alert_high;	// uAh
};

static DEFINE_IDR(ltc2941_id);
static DEFINE_MUTEX(ltc2941_lock);


static inline int convert_bin_to_uAh(const struct ltc2941_info *info, int Q){
	int uAh;
	uAh = ((Q * (info->Qlsb / 10))) / 100;

	return uAh;
}

static inline int convert_uAh_to_bin(const struct ltc2941_info *info, int uAh){
	int Q;
	Q = (uAh * 100) / (info->Qlsb/10);
	Q = (Q < LTC2941_MAX_VALUE) ? Q : LTC2941_MAX_VALUE;
	
	return Q;
}


static int ltc2941_read_regs(struct i2c_client *client,
				    LTC2941_REG reg, 
				    u8 *buf,
				    int num_regs){
        int ret;
        struct i2c_msg msgs[2] = { };
	u8 reg_start = reg;

        if(((reg_start + num_regs) > 8) || (num_regs < 1)){
                return -EINVAL;
        }

        msgs[0].addr 	= client->addr;
        msgs[0].len 	= 1;
        msgs[0].buf 	= &reg_start;

	msgs[1].addr 	= client->addr;
	msgs[1].len 	= num_regs;
	msgs[1].buf 	= buf;
	msgs[1].flags	= I2C_M_RD;	// Read

        ret = i2c_transfer(client->adapter, &msgs[0], 2);
        if (ret < 0) {
                dev_err(&client->dev, "ltc2941 read_reg failed!\n");
                return ret;
        }

        return 0;
}

/*
static int ltc2941_read_regs(struct i2c_client *client,
				    LTC2941_REG reg, 
				    u8 *buf,
				    int num_regs){
        int ret;
	u8 reg_start = reg;

        if(((reg_start + num_regs) > 8) || (num_regs < 1)){
                return -EINVAL;
        }

        ret = i2c_smbus_write_i2c_block_data(client, reg_start, num_regs, buf);
        if (ret < 0) {
                dev_err(&client->dev, "ltc2941 read_reg failed!\n");
                return ret;
        }

        return 0;
}
*/

static int ltc2941_write_regs(struct i2c_client *client, 
				     LTC2941_REG reg,
				     const u8* const buf,
				     int num_regs){
	int ret;
	u8 reg_start = reg;

	if(((reg_start + num_regs) > 8) || (num_regs < 1)){
		printk("LTC2941: Invalid write action, start: 0x%02x, num: 0x%02x", reg_start, num_regs);
		return -EINVAL;
	}

	ret = i2c_smbus_write_i2c_block_data(client, reg_start, num_regs, buf);
	if (ret < 0) {
		dev_err(&client->dev, "ltc2941 write_reg failed!\n");
		return ret;
	}

	return 0;
}

static int ltc2941_reset(const struct ltc2941_info *info){
	int ret;
	u8 buf[8] = { };

	// Read status and control registers
	ret = ltc2941_read_regs(info->client, LTC2941_REG_STATUS, &buf[LTC2941_REG_STATUS], 8);
	if(ret < 0){
		printk("LTC2941: Could not read registers from device!\n");
		goto error_exit;
	}

	// Check magic bits for chip id
	if(LTC2941_GET_MAGIC(buf[LTC2941_REG_STATUS]) != LTC2941_MAGIC){
		printk("LTC2941: Magic numbers do not match, status_reg: 0x%02x, control: 0x%02x\n", buf[LTC2941_REG_STATUS], buf[LTC2941_REG_CONTROL]);
		ret = -ENODEV;
		goto error_exit;
	}

	// Set prescaler and disable analog section
	buf[LTC2941_REG_CONTROL] &= ~LTC2941_REG_CONTROL_PRESCALER_MASK;
	buf[LTC2941_REG_CONTROL] |= LTC2941_SET_PRESCALER(info->PrescalerM) | LTC2941_REG_CONTROL_SHUTDOWN_MASK;
	
	ltc2941_write_regs(info->client, LTC2941_REG_CONTROL, &buf[LTC2941_REG_CONTROL], 1);
	if(ret < 0){
		printk("LTC2941: Could not write registers!\n");
		goto error_exit;
	}

	// Set and write all registers
//	buf[LTC2941_REG_ACC_CHARGE_MSB]  = I16_MSB(LTC2941_MID_SUPPLY);
//	buf[LTC2941_REG_ACC_CHARGE_LSB]  = I16_LSB(LTC2941_MID_SUPPLY);
	buf[LTC2941_REG_THRESH_HIGH_MSB] = I16_MSB(convert_uAh_to_bin(info, info->alert_high));
	buf[LTC2941_REG_THRESH_HIGH_LSB] = I16_LSB(convert_uAh_to_bin(info, info->alert_high));
	buf[LTC2941_REG_THRESH_LOW_MSB]  = I16_MSB(convert_uAh_to_bin(info, info->alert_low));
	buf[LTC2941_REG_THRESH_LOW_LSB]  = I16_LSB(convert_uAh_to_bin(info, info->alert_low));

	ltc2941_write_regs(info->client, LTC2941_REG_ACC_CHARGE_MSB, &buf[LTC2941_REG_ACC_CHARGE_MSB], 6);

	// Enable analog section
	buf[LTC2941_REG_CONTROL] &= ~LTC2941_REG_CONTROL_SHUTDOWN_MASK;
	ltc2941_write_regs(info->client, LTC2941_REG_CONTROL, &buf[LTC2941_REG_CONTROL], 1);
	if(ret < 0){
		goto error_exit;
	}

	// Read status and control registers
	ret = ltc2941_read_regs(info->client, LTC2941_REG_STATUS, &buf[LTC2941_REG_STATUS], 8);
	if(ret < 0){
		goto error_exit;
	}
	printk("ltc21941_status: 0x%02x\n", buf[LTC2941_REG_STATUS]);
	printk("ltc21941_control: 0x%02x\n", buf[LTC2941_REG_CONTROL]);
	printk("ltc21941_charge: 0x%02x, 0x%02x\n", buf[LTC2941_REG_ACC_CHARGE_MSB], buf[LTC2941_REG_ACC_CHARGE_LSB]);
	printk("ltc21941_thresh_high: 0x%02x, 0x%02x\n", buf[LTC2941_REG_THRESH_HIGH_MSB], buf[LTC2941_REG_THRESH_HIGH_LSB]);
	printk("ltc21941_thresh_low: 0x%02x, 0x%02x\n", buf[LTC2941_REG_THRESH_LOW_MSB], buf[LTC2941_REG_THRESH_LOW_LSB]);

	return 0;

error_exit:
	return ret;
}

static int ltc2941_get_charge_now(const struct ltc2941_info *info, int *val){
	int ret;
	u8 datar[2];
	u32 value;

	ret = ltc2941_read_regs(info->client, LTC2941_REG_ACC_CHARGE_MSB, &datar[0], 2);

	value = (datar[0] << 8) + datar[1];

	*val = convert_bin_to_uAh(info, value);

	return ret;
}

static int ltc2941_set_charge_now(const struct ltc2941_info *info, int val){
	int ret;
	u8 dataw[2], ctrl_reg;
	u32 value = convert_uAh_to_bin(info, val);

	// Read control register
        ret = ltc2941_read_regs(info->client, LTC2941_REG_CONTROL, &ctrl_reg, 1);
        if(ret < 0){
                goto error_exit;
        }

	// Disable analog section
	ctrl_reg |= LTC2941_REG_CONTROL_SHUTDOWN_MASK;
        ret = ltc2941_write_regs(info->client, LTC2941_REG_CONTROL, &ctrl_reg, 1);
        if(ret < 0){
                goto error_exit;
        }


	// Set new charge value
	dataw[0] = I16_MSB(value);
	dataw[1] = I16_LSB(value);
	ret = ltc2941_write_regs(info->client, LTC2941_REG_ACC_CHARGE_MSB, &dataw[0], 2);
	if(ret < 0){
		goto error_exit;
	}

	// Enable analog section
        ctrl_reg &= ~LTC2941_REG_CONTROL_SHUTDOWN_MASK;
        ret = ltc2941_write_regs(info->client, LTC2941_REG_CONTROL, &ctrl_reg, 1);
        if(ret < 0){
                goto error_exit;
        }

	return 0;

error_exit:
	return ret;
}

static int ltc2941_get_charge_counter(const struct ltc2941_info *info, int *val){
	int ret;

	ret = ltc2941_get_charge_now(info, val);
	
	*val -= convert_bin_to_uAh(info, LTC2941_MID_SUPPLY);

	return ret;
}

static int ltc2941_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	struct ltc2941_info *info = container_of(psy, struct ltc2941_info, supply);
	int ret;

	switch (prop) {
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			ret = ltc2941_get_charge_now(info, &val->intval);
			break;

		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			ret = ltc2941_get_charge_counter(info, &val->intval);
			break;

		default:
			ret = -EINVAL;
	}

	return ret;
}

static int ltc2941_set_property(struct power_supply *psy,
                                enum power_supply_property psp,
                                const union power_supply_propval *val)
{
        struct ltc2941_info *info = container_of(psy, struct ltc2941_info, supply);
        int ret;

        switch (psp) {
                case POWER_SUPPLY_PROP_CHARGE_NOW:
                        ret = ltc2941_set_charge_now(info, val->intval);
                        break;

                default:
                        ret = -EPERM;
        }

        return ret;
}

static int ltc2941_property_is_writeable(struct power_supply *psy, enum power_supply_property psp){
        switch (psp) {
	        case POWER_SUPPLY_PROP_CHARGE_NOW:
        	        return 1;
	        default:
        	        break;
        }

        return 0;
}


static void ltc2941_update(struct ltc2941_info *info)
{
	int charge;

	charge = info->charge;

	ltc2941_get_charge_now(info, &info->charge);

	if(charge != info->charge){
		power_supply_changed(&info->supply);
	}
}

static void ltc2941_work(struct work_struct *work)
{
	struct ltc2941_info *info;

	info = container_of(work, struct ltc2941_info, work.work);
	ltc2941_update(info);

	schedule_delayed_work(&info->work, LTC2941_WORK_DELAY * HZ);
}

static enum power_supply_property ltc2941_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int ltc2941_i2c_remove(struct i2c_client *client)
{
	struct ltc2941_info *info = i2c_get_clientdata(client);
	
	cancel_delayed_work(&info->work);

	power_supply_unregister(&info->supply);
	kfree(info->supply.name);

	mutex_lock(&ltc2941_lock);
	idr_remove(&ltc2941_id, info->id);
	mutex_unlock(&ltc2941_lock);

	return 0;
}

static int ltc2941_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltc2941_info *info;
	int ret;
	int num;
	unsigned int Qbat, PrescalerM, rSense, alert_low, alert_high;
	struct device_node *np;

	mutex_lock(&ltc2941_lock);
	ret = idr_alloc(&ltc2941_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&ltc2941_lock);
	if(ret < 0){
		goto fail_id;
	}
	num = ret;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if(info == NULL) {
		dev_err(&client->dev, "Could not allocate memory for device structure!\n");
		ret = -ENOMEM;
		goto fail_info;
	}

	info->supply.name = kasprintf(GFP_KERNEL, "%s-%d", client->name, num);
	if(!info->supply.name){
		dev_err(&client->dev, "Could not allocate memory for device name!\n");
		ret = -ENOMEM;
		goto fail_name;
	}

	np = of_node_get(client->dev.of_node);

	ret = of_property_read_u32(np, "rSense", &rSense);
	if(ret < 0){
		dev_err(&client->dev, "Could not find rSense in devicetree!\n");
		goto fail_name;
	}
	info->rSense = rSense;
	
        ret = of_property_read_u32(np, "PrescalerExponent", &PrescalerM);
        if(ret < 0){
		ret = of_property_read_u32(np, "Qbat", &Qbat);
		if(ret < 0){
			dev_err(&client->dev, "PrescalerExponent AND Qbat not found in devicetree, define at least one!\n");
			goto fail_name;
		}else{
			// TODO: Implement PrescalerM calculation using rSense and Qbat
			// PrescalerM = ;
	               	// dev_warn(&client->dev, "Could not find PrescalerM in devicetree! Using: %d\n", PrescalerM);
			dev_err(&client->dev, "Could not find PrescalerM in devicetree, Qbat is not yet supported!\n");
			ret = -EINVAL;
			goto fail_name;
		}
        }

	info->PrescalerM = (PrescalerM < LTC2941_MAX_PRESCALER_EXP) ? PrescalerM : LTC2941_MAX_PRESCALER_EXP;
	info->Qlsb = ((58 * 50000) / rSense) / (128 / (1 << PrescalerM));

        ret = of_property_read_u32(np, "alert-low", &alert_low);
        if(ret < 0){
                dev_warn(&client->dev, "Could not find alert-low in devicetree! Using default: %04x\n", LTC2941_DEFAULT_ALERT_LOW);
                info->alert_low = convert_bin_to_uAh(info, LTC2941_DEFAULT_ALERT_LOW);
        }else{	
		info->alert_low = alert_low;
	}

        ret = of_property_read_u32(np, "alert-high", &alert_high);
        if(ret < 0){
                dev_warn(&client->dev, "Could not find alert-high in devicetree! Using default: %04x\n", LTC2941_DEFAULT_ALERT_HIGH);
        	info->alert_high = convert_bin_to_uAh(info, LTC2941_DEFAULT_ALERT_HIGH);
	}else{
		info->alert_high = alert_high;
	}
	
	i2c_set_clientdata(client, info);

	info->client = client;
	info->id = num;
	
	info->supply.type			= POWER_SUPPLY_TYPE_BATTERY;
	info->supply.properties			= ltc2941_properties;
	info->supply.num_properties		= ARRAY_SIZE(ltc2941_properties);
	info->supply.get_property		= ltc2941_get_property;
	info->supply.set_property		= ltc2941_set_property;
	info->supply.property_is_writeable	= ltc2941_property_is_writeable;
	info->supply.external_power_changed	= NULL;
//	info->supply.set_charged 		= ltc2941_set_charged;

	INIT_DELAYED_WORK(&info->work, ltc2941_work);

	ret = ltc2941_reset(info);
	if(ret < 0){
		dev_err(&client->dev, "Communication with chip failed!\n");
		goto fail_comm;
	}

	ret = power_supply_register(&client->dev, &info->supply);
	if (ret) {
		dev_err(&client->dev, "failed to register ltc2941\n");
		goto fail_register;
	} else {
		schedule_delayed_work(&info->work, LTC2941_WORK_DELAY * HZ);
	}

	return 0;

fail_register:
	kfree(info->supply.name);
fail_comm:
fail_name:
fail_info:
	mutex_lock(&ltc2941_lock);
	idr_remove(&ltc2941_id, num);
	mutex_unlock(&ltc2941_lock);
fail_id:
	return ret;
}

#ifdef CONFIG_PM_SLEEP

static int ltc2941_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc2941_info *info = i2c_get_clientdata(client);

	cancel_delayed_work(&info->work);
	return 0;
}

static int ltc2941_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc2941_info *info = i2c_get_clientdata(client);

	schedule_delayed_work(&info->work, LTC2941_WORK_DELAY * HZ);
	return 0;
}

static SIMPLE_DEV_PM_OPS(ltc2941_pm_ops, ltc2941_suspend, ltc2941_resume);
#define LTC2941_PM_OPS (&ltc2941_pm_ops)

#else
#define LTC2941_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */


static const struct i2c_device_id ltc2941_i2c_id[] = {
	{"ltc2941", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, ltc2941_i2c_id);

static struct i2c_driver ltc2941_driver = {
	.driver 	= {
		.name	= "LTC2941",
		.owner	= THIS_MODULE,
		.pm 	= LTC2941_PM_OPS,
	},
	.probe		= ltc2941_i2c_probe,
	.remove		= ltc2941_i2c_remove,
	.id_table	= ltc2941_i2c_id,
};
module_i2c_driver(ltc2941_driver);

MODULE_AUTHOR("Auryn Verwegen, Topic Embedded Systems");
MODULE_DESCRIPTION("LTC2941 Battery Gas Gauge IC driver");
MODULE_LICENSE("GPL");
