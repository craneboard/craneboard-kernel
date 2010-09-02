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
#include <linux/delay.h>

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

        if (voltage == TPS65910_BBSEL_3P0 || voltage == TPS65910_BBSEL_2P52 ||
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

int tps65910_i2c_read_u8(u8 mod_no, u8 *value, u8 reg)
{
	struct tps65910_client *tps65910;

	switch (mod_no) {
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

	(*value) = i2c_smbus_read_byte_data(tps65910->client, reg);
	mdelay (100);
	if (*value < 0)
		return -EIO;
	else
		return 0;
}
EXPORT_SYMBOL(tps65910_i2c_read_u8);


int tps65910_i2c_write_u8(u8 slave_addr, u8 value, u8 reg)
{
	struct tps65910_client *tps65910;
	int ret;
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;

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


	ret = i2c_smbus_write_byte_data(tps65910->client, reg, temp_buffer[1]);
	if (ret < 0)
		return -EIO;
	else
		return 0;
}
EXPORT_SYMBOL(tps65910_i2c_write_u8);

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

static int __init
tps65910_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int	 status;
	unsigned i;

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
