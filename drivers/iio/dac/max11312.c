/*
 *  max11312.c - Support for Maxim max11312
 *
 *  Copyright (C) 2017, thermofisher Scientific vasubabu kandimalla <vasubabu.kandimalla@thermofisher.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define max11312_DRV_NAME	"max11312"

/* Commands */
#define COMMAND_CHANNEL0	0x00
#define COMMAND_CHANNEL1	0x01 /* for MAX518 and MAX519 */
#define COMMAND_PD		0x08 /* Power Down */

enum max11312_device_ids {
	ID_max11312,
};

struct max11312_data {
	struct i2c_client	*client;
	unsigned short		vref_mv[8];
};

/* MAX11312 Register Definitions */
#define MAX11312_DEV_ID_REG                     0x00            /* Device ID 0x1424 */
#define MAX11312_DEV_CONTROL_REG                0x10
#define MAX11312_DEV_CONTROL_DATA               0x0400

#define MAX11312_INT_MASK_REG                   0x11
#define MAX11312_IRQ_MODE_PORT_0_5_REG  0x12
#define MAX11312_IRQ_MODE_PORT_6_1_REG0 0x13

/* port Configuration Registers */
static int port_conf_reg[] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30 };

/* ADC data Registers */
//static int adc_data_reg[] = { 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50 };

/* DAC data Registers */
static int dac_data_reg[] = {0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70 };

#define MAX11312_DAC_CONF_0     0x5100          /* 0V to +10V */ 
#define MAX11312_DAC_CONF_1     0x5200          /* -5V to +5V */        
#define MAX11312_DAC_CONF_2     0x530/0         /* -10V to 0V */        

static int max11312_set_value(struct iio_dev *indio_dev,
	long val, int channel)
{
	struct max11312_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	u8 outbuf[4];
	int res;

	if (val < 0 || val > 2047)
		return -EINVAL;
	if((channel < 0) || (channel > 11))
                return -EINVAL;
	//printk("*\n");
	outbuf[0] = dac_data_reg[channel];
	outbuf[1] = (val >> 8) & 0xF;
	outbuf[2] = val & 0xFF;
	res = i2c_master_send(client, outbuf, 3);
	if (res < 0)
		return res;

	return 0;
}

static int max11312_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct max11312_data *data = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		/* Corresponds to Vref / 2^(bits) */
		*val = data->vref_mv[chan->channel];
		*val2 = 8;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}
	return -EINVAL;
}

static int max11312_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = max11312_set_value(indio_dev, val, chan->channel);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int max11312_suspend(struct device *dev)
{
	u8 outbuf = COMMAND_PD;

	return i2c_master_send(to_i2c_client(dev), &outbuf, 1);
}

static int max11312_resume(struct device *dev)
{
	u8 outbuf = 0;

	return i2c_master_send(to_i2c_client(dev), &outbuf, 1);
}

static SIMPLE_DEV_PM_OPS(max11312_pm_ops, max11312_suspend, max11312_resume);
#define max11312_PM_OPS (&max11312_pm_ops)
#else
#define max11312_PM_OPS NULL
#endif

static const struct iio_info max11312_info = {
	.read_raw = max11312_read_raw,
	.write_raw = max11312_write_raw,
	.driver_module = THIS_MODULE,
};

#define max11312_CHANNEL(chan) {				\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.output = 1,					\
	.channel = (chan),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
	BIT(IIO_CHAN_INFO_SCALE),			\
}

static const struct iio_chan_spec max11312_channels[] = {
	max11312_CHANNEL(0),
	max11312_CHANNEL(1),
	max11312_CHANNEL(2),
	max11312_CHANNEL(3),
	max11312_CHANNEL(4),
	max11312_CHANNEL(5),
	max11312_CHANNEL(6),
	max11312_CHANNEL(7),
	max11312_CHANNEL(8),
	max11312_CHANNEL(9),
	max11312_CHANNEL(10),
	max11312_CHANNEL(11),
};

static int max11312_ports_initialization(struct max11312_data *data)
{
        struct i2c_client *client = data->client;
        u8 out_buf[4];
	int ret, count;


	out_buf[0] = MAX11312_DEV_CONTROL_REG;
        out_buf[1] = 0x00;
        out_buf[2] = 0x40;
        ret = i2c_master_send(client, out_buf, 3);
        if (ret < 0)
          	return -EIO;

	udelay(200);
	
	for(count = 0; count < 12; count++) {
		out_buf[0] = port_conf_reg[count];
		out_buf[1] = 0x51;
		//out_buf[1] = 0x52;
		out_buf[2] = 0x00;
        	ret = i2c_master_send(client, out_buf, 3);
        	if (ret < 0)
                	return -EIO;
		mdelay(2);
	}

	return 0;
}
static int max11312_check_device_id(struct max11312_data *data)
{
        struct i2c_client *client = data->client;
        struct i2c_msg msg[2];
        int ret;
        char outbuf[2];
        char inbuf[8];

        outbuf[0] = MAX11312_DEV_ID_REG;
        outbuf[1] = 0x00;

        msg[0].addr    = client->addr;
        msg[0].flags   = client->flags;
        msg[0].len     = 2;
        msg[0].buf     = outbuf;

        msg[1].addr    = client->addr;
        msg[1].flags   = client->flags | I2C_M_RD ;
        msg[1].len     = 2;
        msg[1].buf     = inbuf;

        ret = i2c_transfer(client->adapter, msg, 2);
        if(ret < 0) {
                printk("I2C transfer Failed\n");
                return ret;
        }

        return ((inbuf[0] << 8) | inbuf[1] );
}

static int max11312_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max11312_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	/* establish that the iio_dev is a child of the i2c device */
	indio_dev->dev.parent = &client->dev;
	indio_dev->num_channels = 12;
	indio_dev->channels = max11312_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &max11312_info;

	ret = max11312_check_device_id(data);
        if(ret < 0) {
                printk("Error: Reading Device ID \n");
                return -EIO;
        }
        else if(ret == 0x1424) {
                printk("Device ID:%x\n", ret);
                ret = max11312_ports_initialization(data);
                if(ret < 0)
                        return -EIO;
        } else {
                printk("Error: wrong Device ID\n");
                return -EIO;
        }

	return iio_device_register(indio_dev);
}

static int max11312_remove(struct i2c_client *client)
{
	iio_device_unregister(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id max11312_id[] = {
	{ "max11312", ID_max11312 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max11312_id);

static struct i2c_driver max11312_driver = {
	.driver = {
		.name	= max11312_DRV_NAME,
		.pm		= max11312_PM_OPS,
	},
	.probe		= max11312_probe,
	.remove		= max11312_remove,
	.id_table	= max11312_id,
};
module_i2c_driver(max11312_driver);

MODULE_AUTHOR("vasubabu kandimalla<vasubabu.kandimalla@thermofisher.com>");
MODULE_DESCRIPTION("12-bit MAX11312 DAC Mixed-Signal I/O driver");
MODULE_LICENSE("GPL");
