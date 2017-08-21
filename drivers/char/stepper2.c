#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>

#include "stepper2.h"

#define STEPPER2_MAJOR	242
#define STEPPER2_STB	69
#define STEPPER2_RESET	170
#define STEPPER2_SLEEP	169

#define  DEBUG_STEPEER2 0 

struct stepper2_data {
	dev_t devt;
	struct device 	*dev;
	struct class	*stepper2_class;
};

unsigned char stepper2_A_B_chan[16]=
{
        0xF3, 0xC0,
        0xF8, 0x00,
        0xF2, 0xC0,
        0x32, 0xE0,
        0xB2, 0xC0,
        0xB8, 0xC0,
        0xB3, 0xC0,
        0x03, 0xE0
};

unsigned char stepper2_C_D_chan[16] = 
{
        0xF3, 0xC8,
        0xF8, 0x08,
        0xF2, 0xC8,
        0x32, 0xE8,
        0xB2, 0xC8,
        0xB8, 0xC8,
        0xB3, 0xC8,
        0x03, 0xE8
};

static char tx_buf[2];
static struct spi_device *stepper2_spi;
static struct mutex stepper2_lock;

static int stepper2_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static int stepper2_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper2_read(struct file *file, char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper2_write(struct file *file, const char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}

static long stepper2_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int count;
	int status;
	int itcount = -1;
	char dir;
	char motorNum;
	static unsigned char st2_pos_1 = 0;
	static unsigned char st2_pos_2 = 0;
	struct spi_device *spi;

	switch(cmd)
	{
		case STEPPER_3_4_ROTATE_45:
			itcount = 1;	
		     break;
		case STEPPER_3_4_ROTATE_90:
			itcount = 100;
		     break;
		case STEPPER_3_4_ROTATE_180:
			itcount = 200;
		     break;			
		case STEPPER_3_4_ROTATE_270:
			itcount = 300;
		     break;
		case STEPPER_3_4_ROTATE_360:
			itcount = 400;
		     break;
		case STEPPER_3_4_ROTATE_OFF:
			itcount = 0;
		     break;
		default:
			return -EINVAL;
	}

	if(itcount == -1)
	{
		return -EINVAL;
	}

	motorNum = ((char )(arg >> 4)) & 0x01;

	if(itcount == 0)
	{
		spi = spi_dev_get(stepper2_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper2_lock);
		
		if(motorNum == 1) // C_D_Channel
        	{
			tx_buf[0] = 0x00;
                	tx_buf[1] = 0x08;

		}
		else if(motorNum == 0)
		{

			tx_buf[0] = 0x00;
                	tx_buf[1] = 0x00;

		}
		else
		{
			/* do nothing */
		}

        	status = spi_write(spi, tx_buf, 2);

        	if(status != 0)
        		printk("spi_write is failed\n");
        	
        	mutex_unlock(&stepper2_lock);

		//printk("Motors off for stepper 2\n");

		return 0; 
	}

        //motorNum = ((char )(arg >> 4)) & 0x01;

        if(motorNum == 1) // C_D_Channel
	{	
		dir = ((char )arg) & 0x01;

		//printk("command for C_D channel, dir = %d\n",dir);

		spi = spi_dev_get(stepper2_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper2_lock);

		for(count = 0; count < itcount; count++)
        	{
			if(dir)
			{
				if( st2_pos_1 < 7 )
		    			st2_pos_1++;
		    		else
		    			st2_pos_1 = 0;
			}  
			else
			{
				if( st2_pos_1 > 0 )
		    			st2_pos_1--;
		    		else
		    			st2_pos_1 = 7;
			}
		
			tx_buf[0] = stepper2_C_D_chan[st2_pos_1 * 2];
			tx_buf[1] = stepper2_C_D_chan[(st2_pos_1 * 2) + 1];

			status = spi_write(spi, tx_buf, 2);

        		if(status != 0)
                		printk("spi_write is failed\n");
	
			udelay(2000);				
		}

        	mutex_unlock(&stepper2_lock);
	}
	else if (motorNum == 0) // A_B_Channel
	{

		dir = ((char )arg) & 0x01;

		//printk("command for A_B channel, dir = %d\n",dir);

                spi = spi_dev_get(stepper2_spi);

                if(spi == NULL)
                        return -ENODEV;

                mutex_lock(&stepper2_lock);

                for(count = 0; count < itcount; count++)
                {
                        if(dir)
                        {
                                if( st2_pos_2 < 7 )
                                        st2_pos_2++;
                                else
                                        st2_pos_2 = 0;
                        }
                        else
                        {
                                if( st2_pos_2 > 0 )
                                        st2_pos_2--;
                                else
                                        st2_pos_2 = 7;
                        }


			tx_buf[0] = stepper2_A_B_chan[st2_pos_2 *2];
                        tx_buf[1] = stepper2_A_B_chan[(st2_pos_2 *2) + 1];

                        status = spi_write(spi, tx_buf, 2);

                        if(status != 0)
                                printk("spi_write is failed\n");

			 udelay(2000);
                }

                mutex_unlock(&stepper2_lock);
	}
	else
	{
		/* Do nothing */
	}

	return 0;
}
 
static struct file_operations stepper2_fops = {
	.open 		= stepper2_open,
	.release	= stepper2_release,
	.read		= stepper2_read,
	.write		= stepper2_write,
	.unlocked_ioctl = stepper2_ioctl,
};

static const struct of_device_id stepper2_spi_dt_ids[] = {
        { .compatible = "stepper2"},
        {}
};
MODULE_DEVICE_TABLE(of, stepper2_spi_dt_ids);

static int stepper2_probe(struct spi_device *spi)
{
	struct stepper2_data *stepper2_data;
	int ret;
	
#if DEBUG_STEPEER2
	printk("func:%s=>Line:%d\n", __FUNCTION__, __LINE__);
#endif

	if(spi->dev.of_node && !of_match_device(stepper2_spi_dt_ids, &spi->dev)) {
                dev_err(&spi->dev, "ERROR DT: STEPPER2 listed directly in DT\n");
                WARN_ON(spi->dev.of_node &&
                        !of_match_device(stepper2_spi_dt_ids, &spi->dev));
        }

	stepper2_data = kzalloc(sizeof(*stepper2_data), GFP_KERNEL);
	if(!stepper2_data)
		return -ENOMEM;
	
	stepper2_spi=spi;
	stepper2_spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	
	mutex_init(&stepper2_lock);

	gpio_export(STEPPER2_STB, 1);
	gpio_direction_output(STEPPER2_STB, 1);
	gpio_set_value(STEPPER2_STB, 0);

	gpio_export(STEPPER2_RESET, 1);
	gpio_direction_output(STEPPER2_RESET, 1);
	gpio_set_value(STEPPER2_RESET, 0);

	gpio_export(STEPPER2_SLEEP, 1);
	gpio_direction_output(STEPPER2_SLEEP, 1);
	gpio_set_value(STEPPER2_SLEEP, 0);

	udelay(1);

	gpio_set_value(STEPPER2_STB, 1);
	gpio_set_value(STEPPER2_RESET, 1);
	gpio_set_value(STEPPER2_SLEEP, 1);

	ret = register_chrdev(STEPPER2_MAJOR, "stepper2", &stepper2_fops);
        if (ret < 0)
                return ret;

        stepper2_data->stepper2_class = class_create(THIS_MODULE, "stepper2");
        if(IS_ERR(stepper2_data->stepper2_class)) {
		ret = -1; 
		goto err;
        }

	stepper2_data->devt = MKDEV(STEPPER2_MAJOR, 0);
        stepper2_data->dev = device_create(stepper2_data->stepper2_class, &spi->dev, stepper2_data->devt, stepper2_data, "stepper2");
        if(IS_ERR(stepper2_data->dev)){
                ret = -ENODEV;
		goto err1;
	}

	ret = spi_setup(spi);
        if (ret < 0) {
                dev_err(&spi->dev, "SPI setup wasn't successful %d", ret);
                ret = -ENODEV;
        }

	spi_set_drvdata(spi, stepper2_data);
	
	tx_buf[0] = 0x00;
	tx_buf[1] = 0x00;
	
	ret = spi_write(spi, tx_buf, 2);
	if(ret < 0)
		printk("spi write error\n");

	return ret;
err1:
	class_destroy(stepper2_data->stepper2_class);
    unregister_chrdev(STEPPER2_MAJOR, "stepper2");
	kfree(stepper2_data);
	return ret;

err:
	unregister_chrdev(STEPPER2_MAJOR, "stepper2");
	kfree(stepper2_data);
	return ret;
}

static int stepper2_remove(struct spi_device *spi)
{
	struct stepper2_data *stepper2_data	= spi_get_drvdata(spi);

    device_destroy(stepper2_data->stepper2_class, stepper2_data->devt);
	class_destroy(stepper2_data->stepper2_class);
    unregister_chrdev(STEPPER2_MAJOR, "stepper2");
	kfree(stepper2_data);
	return 0;
}

static struct spi_driver stepper2_spi_driver = {
	.probe 		= stepper2_probe,
	.remove 	= stepper2_remove,
	.driver 	= {
		.name 	= "stepper2",
		.owner	= THIS_MODULE,
	},
};

static int __init stepper2_init(void)
{
	int ret = -1;

	ret = spi_register_driver(&stepper2_spi_driver);
        if (ret < 0) {
		printk("stepper2 SPi driver registration failed\n");
        }
	
	return ret;
}

module_init(stepper2_init);

static void __exit stepper2_exit(void)
{
	spi_unregister_driver(&stepper2_spi_driver);
}

module_exit(stepper2_exit);

MODULE_AUTHOR("sivakoti, sivakoti.danda@cyient.com");
MODULE_DESCRIPTION(" STEPPER2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:STEPPER2");

