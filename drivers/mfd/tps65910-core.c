/*
 * tps65910-core.c -- Multifunction core driver for  TPS65910x chips
 *
 * Copyright (C) 2010 Mistral solutions Pvt Ltd <www.mistralsolutions.com>
 *
 * Based on twl-core.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <linux/regulator/machine.h>

#include <linux/i2c.h>
#include <linux/i2c/tps65910.h>
#include <plat/board.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
#include <plat/cpu.h>
#endif

#if defined(CONFIG_GPIO_TPS65910)
#define tps65910_has_gpio()  true
#else
#define tps65910_has_gpio()  false
#endif

#if defined(CONFIG_REGULATOR_TPS65910)
#define tps65910_has_regulator()     true
#else
#define tps65910_has_regulator()     false
#endif

#if defined(CONFIG_RTC_DRV_TPS65910)
#define tps65910_has_rtc()   true
#else
#define tps65910_has_rtc()   false
#endif

enum tps65910x_model {
	TPS65910,   	/* TI processors OMAP3 family */
	TPS659101,  	/* Samsung - S5PV210, S5PC1xx */
	TPS659102,	/* Samsung - S3C64xx */
	TPS659103,	/* Reserved */
	TPS659104,	/* Reserved */
	TPS659105,	/* TI processors - DM643x, DM644x */
	TPS659106,	/* Reserved */
	TPS659107,	/* Reserved */
	TPS659108,	/* Reserved */
	TPS659109,	/* Freescale - i.MX51 */

};

static bool inuse;
static struct work_struct core_work;
static struct mutex work_lock;

/* Structure for each TPS65910 Slave */
struct tps65910_client {
	struct i2c_client *client;
	u8 address;
	/* max numb of i2c_msg required is for read =2 */
	struct i2c_msg xfer_msg[2];
	/* To lock access to xfer_msg */
	struct mutex xfer_lock;
};
static struct tps65910_client tps65910_modules[TPS65910_NUM_SLAVES];



/* bbch = Back-up battery charger control register */

int tps65910_enable_bbch(u8 voltage)
{

	u8 val = 0;
	int err;

	if (voltage == TPS65910_BBSEL_3P0 || voltage == TPS65910_BBSEL_3P0 ||
	voltage == TPS65910_BBSEL_3P15 || voltage == TPS65910_BBSEL_VBAT) {
		val = (voltage | TPS65910_BBCHEN);
		err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
				TPS65910_REG_BBCH);
		if (err) {
			printk(KERN_ERR "Unable write TPS65910_REG_BBCH reg\n");
			return -EIO;
		}
	} else {
		printk(KERN_ERR"Invalid argumnet for %s \n", __func__);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(tps65910_enable_bbch);

int tps65910_disable_bbch(void)
{

	u8 val = 0;
	int err;

	err = tps65910_i2c_read_u8(TPS65910_I2C_ID0, &val, TPS65910_REG_BBCH);

	if (!err) {
		val &= ~TPS65910_BBCHEN;

		err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
				TPS65910_REG_BBCH);
		if (err) {
			printk(KERN_ERR "Unable write TPS65910_REG_BBCH \
					reg\n");
			return -EIO;
		}
	} else {
		printk(KERN_ERR "Unable to read TPS65910_REG_BBCH reg\n");
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(tps65910_disable_bbch);

int tps65910_i2c_read(u8 slave_addr, u8 *value, u8 reg, unsigned num_bytes)
{
	int ret;
	u8 val;
	struct tps65910_client *tps65910;
	struct i2c_msg *msg;

	switch (slave_addr) {
	case TPS65910_I2C_ID0:
		tps65910 = &tps65910_modules[0];
		tps65910->address = TPS65910_I2C_ID0;
		break;
	case TPS65910_I2C_ID1:
		tps65910 = &tps65910_modules[1];
		tps65910->address = TPS65910_I2C_ID1;
	default:
		printk(KERN_ERR "Invalid Slave address for TPS65910\n");
		return -ENODEV;
	}
	mutex_lock(&tps65910->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &tps65910->xfer_msg[0];
	msg->addr = tps65910->address;
	msg->len = 1;
	msg->flags = 0;	/* Read the register value */
	val = reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &tps65910->xfer_msg[1];
	msg->addr = tps65910->address;
	msg->flags = I2C_M_RD;	/* Read the register value */
	msg->len = num_bytes;	/* only n bytes */
	msg->buf = value;

	ret = i2c_transfer(tps65910->client->adapter, tps65910->xfer_msg, 2);
	mutex_unlock(&tps65910->xfer_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		pr_err("%s: i2c_read failed to transfer all messages\n",
				"TPS65910C");
			return -EIO;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(tps65910_i2c_read);


int tps65910_i2c_write(u8 slave_addr, u8 *value, u8 reg, unsigned num_bytes)
{
	int ret;
	struct tps65910_client *tps65910;
	struct i2c_msg *msg;

	switch (slave_addr) {
	case TPS65910_I2C_ID0:
		tps65910 = &tps65910_modules[0];
		tps65910->address = TPS65910_I2C_ID0;
		break;
	case TPS65910_I2C_ID1:
		tps65910 = &tps65910_modules[1];
		tps65910->address = TPS65910_I2C_ID1;
	default:
		printk(KERN_ERR "Invalid Slave address for TPS65910\n");
		return -ENODEV;
	}

	mutex_lock(&tps65910->xfer_lock);
	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	msg = &tps65910->xfer_msg[0];
	msg->addr = tps65910->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	/* over write the first byte of buffer with the register address */
	*value = tps65910->address;
	ret = i2c_transfer(tps65910->client->adapter, tps65910->xfer_msg, 1);
	mutex_unlock(&tps65910->xfer_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		pr_err("%s: i2c_write failed to transfer all messages\n",
				__func__);
	}
	if (ret < 0)
		return -EIO;
	else
		return ret;
}
EXPORT_SYMBOL(tps65910_i2c_write);

int tps65910_i2c_read_u8(u8 mod_no, u8 *value, u8 reg)
{
	if (0 == tps65910_i2c_read(mod_no, value, reg, 1))
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(tps65910_i2c_read_u8);


int tps65910_i2c_write_u8(u8 slave_addr, u8 value, u8 reg)
{

	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;
	printk(KERN_INFO "TPS65910 I2C writing to slave_addr = 0x%x reg = \
		0x val 0x%x reg = 0x%x\n", slave_addr, reg, value);

	if (tps65910_i2c_write(slave_addr, temp_buffer, reg, 1))
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(tps65910_i2c_write_u8);

static void tps65910_core_work(struct work_struct *work)
{
	/* Read the status register and take action  */
	u8 status;
	int err;

	mutex_lock(&work_lock);

	err =  tps65910_i2c_read_u8(TPS65910_I2C_ID0, &status,
			TPS65910_REG_INT_STS);
	if (!err) {
		switch (status) {
		case TPS65910_HOT_DIE_IT:
			break;
		case TPS65910_PWRHOLD_IT:
			break;
		case TPS65910_PWRON_LP_IT:
			/* clear Interrupt,so that it does not reset */
			tps65910_i2c_read_u8(TPS65910_I2C_ID0, &status,
			TPS65910_REG_INT_STS);
			break;
		case TPS65910_PWRON_IT:
			break;
		case TPS65910_VMBHI_IT:
			break;
		case TPS65910_VMBGCH_IT:
			break;
		}
	} else {
		printk(KERN_ERR"tps65910: i2c error %d while reading \
				TPS65910_REG_INT_STS register\n", err);
	}
	mutex_unlock(&work_lock);
}


static irqreturn_t tps65910_isr(int irq, void *data)
{

	/* Disable IRQ, schedule work, enable IRQ and  acknowledge */
	disable_irq(irq);
	(void) schedule_work(&core_work);
	enable_irq(irq);
	return IRQ_HANDLED;
}



static struct device *add_numbered_child(unsigned chip, const char *name,
	int num, void *pdata, unsigned pdata_len, bool can_wakeup, int irq)
{
	struct platform_device  *pdev;
	struct tps65910_client  *tps65910 = &tps65910_modules[chip];
	int  status;

	pdev = platform_device_alloc(name, num);
	if (!pdev) {
		dev_dbg(&tps65910->client->dev, "can't alloc dev\n");
		status = -ENOMEM;
		goto err;
	}

	device_init_wakeup(&pdev->dev, can_wakeup);
	pdev->dev.parent = &tps65910->client->dev;

	if (pdata) {
		status = platform_device_add_data(pdev, pdata, pdata_len);
		if (status < 0) {
			dev_dbg(&pdev->dev, "can't add platform_data\n");
			goto err;
		}
	}
	status = platform_device_add(pdev);

err:
	if (status < 0) {
		platform_device_put(pdev);
		dev_err(&tps65910->client->dev, "can't add %s dev\n", name);
		return ERR_PTR(status);
	}
	return &pdev->dev;

}

static inline struct device *add_child(unsigned chip, const char *name,
		void *pdata, unsigned pdata_len,
		bool can_wakeup, int irq)
{
	return add_numbered_child(chip, name, -1, pdata, pdata_len,
			can_wakeup, irq);
}
static
struct device *add_regulator_linked(int num, struct regulator_init_data *pdata,
		struct regulator_consumer_supply *consumers,
		unsigned num_consumers)
{
	/* regulator framework demands init_data ... */
	if (!pdata)
		return NULL;

	if (consumers) {
		pdata->consumer_supplies = consumers;
		pdata->num_consumer_supplies = num_consumers;
	}
	return add_numbered_child(TPS65910_I2C_ID0, "tps65910_regulator", num,
			pdata, sizeof(*pdata), false, TPS65910_IRQ_LINE);
}

static struct device *
add_regulator(int num, struct regulator_init_data *pdata)
{
	return add_regulator_linked(num, pdata, NULL, 0);
}


static int
add_children(struct tps65910_platform_data *pdata, unsigned long features)
{

	struct device   *child;

	if (tps65910_has_gpio() && pdata->gpio) {
		child = add_child(TPS65910_I2C_ID0, "tps65910_gpio",
				pdata->gpio, sizeof(*pdata->gpio),
				false, pdata->irq_num);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (tps65910_has_rtc()) {
		child = add_child(TPS65910_I2C_ID0, "tps65910_rtc",
				NULL, 0, true, pdata->irq_num);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}
	if (tps65910_has_regulator()) {

		child = add_regulator(TPS65910_VIO, pdata->vio);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDD1, pdata->vdd1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDD2, pdata->vdd2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDD3, pdata->vdd3);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDIG1, pdata->vdig1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDIG2, pdata->vdig2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VAUX33, pdata->vaux33);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VMMC, pdata->vmmc);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VAUX1, pdata->vaux1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VAUX2, pdata->vaux2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VDAC, pdata->vdac);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TPS65910_VPLL, pdata->vpll);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}
	return 0;
}

static int tps65910_remove(struct i2c_client *client)
{
	unsigned i;

	for (i = 0; i < TPS65910_NUM_SLAVES; i++) {

		struct tps65910_client *tps65910 = &tps65910_modules[i];

		if (tps65910->client && tps65910->client != client)
			i2c_unregister_device(tps65910->client);

		tps65910_modules[i].client = NULL;
	}
	inuse = false;
	return 0;
}

int  tps65910_set_i2c_pull_up_down(u8 val)
{
	return tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
				TPS65910_REG_PUADEN);
}

static int __init
tps65910_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int	status;
	unsigned i;

	struct tps65910_platform_data   *pdata = client->dev.platform_data;

	if (!pdata) {
		dev_dbg(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (i2c_check_functionality(client->adapter,
				(I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE)) == 0) {
		dev_dbg(&client->dev, "can't talk I2C?\n");
		return -EIO;
	}
	if (inuse) {
		dev_dbg(&client->dev, "driver is already in use\n");
		return -EBUSY;
	}

	for (i = 0; i < TPS65910_NUM_SLAVES; i++) {

		struct tps65910_client  *tps65910 = &tps65910_modules[i];

		tps65910->address = client->addr + i;

		if (i == 0)
			tps65910->client = client;
		else {
			tps65910->client = i2c_new_dummy(client->adapter,
					tps65910->address);
			if (!tps65910->client) {
				dev_err(&client->dev,
						"can't attach client %d\n", i);
				status = -ENOMEM;
				goto fail;
			}
		}
		mutex_init(&tps65910->xfer_lock);
	}
	inuse = true;

	status = request_irq(pdata->irq_num, tps65910_isr, IRQF_SHARED,
			"TPS65910", NULL);

	if (status < 0) {
		pr_err("tps65910: could not claim irq%d: %d\n", pdata->irq_num,
				status);
		goto fail;
	}

	if (pdata->board_tps65910_config != NULL)
		pdata->board_tps65910_config(pdata);

	INIT_WORK(&core_work, tps65910_core_work);

	mutex_init(&work_lock);

	status = add_children(pdata, id->driver_data);

	if (status < 0)
		goto fail;

	return 0;

fail:
	if (status < 0)
		tps65910_remove(client);

	return status;
}


static int tps65910_i2c_remove(struct i2c_client *client)
{
	unsigned i;

	for (i = 0; i < TPS65910_NUM_SLAVES; i++) {

		struct tps65910_client	*tps65910 = &tps65910_modules[i];

		if (tps65910->client && tps65910->client != client)
			i2c_unregister_device(tps65910->client);

		tps65910_modules[i].client = NULL;
	}
	inuse = false;
	return 0;
}

/* chip-specific feature flags, for i2c_device_id.driver_data */
static const struct i2c_device_id tps65910_i2c_ids[] = {
	{ "tps65910", TPS65910 },
	{ "tps659101", TPS659101 },
	{ "tps659102", TPS659102 },
	{ "tps659103", TPS659103 },
	{ "tps659104", TPS659104 },
	{ "tps659105", TPS659105 },
	{ "tps659106", TPS659106 },
	{ "tps659107", TPS659107 },
	{ "tps659108", TPS659108 },
	{ "tps659109", TPS659109 },
	{/* end of list */ },
};
MODULE_DEVICE_TABLE(i2c, tps65910_i2c_ids);

/* One Client Driver ,3 Clients - Regulator, RTC , GPIO */
static struct i2c_driver tps65910_i2c_driver = {
	.driver.name    = "tps65910",
	.id_table       = tps65910_i2c_ids,
	.probe          = tps65910_i2c_probe,
	.remove         = tps65910_i2c_remove,
};

static int __init tps65910_init(void)
{
	return i2c_add_driver(&tps65910_i2c_driver);
}
subsys_initcall(tps65910_init);

static void __exit tps65910_exit(void)
{
	i2c_del_driver(&tps65910_i2c_driver);
}
module_exit(tps65910_exit);

MODULE_AUTHOR("Mistral Solutions Pvt Ltd");
MODULE_DESCRIPTION("I2C Core interface for TPS65910");
MODULE_LICENSE("GPL");
