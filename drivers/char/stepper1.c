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

#include "stepper1.h"

#define STEPPER1_MAJOR	241
#define STEPPER1_STB	70
#define STEPPER1_RESET	168
#define STEPPER1_SLEEP	167

#define  DEBUG_STEPEER1 0 

struct stepper1_data {
	dev_t devt;
	struct device 	*dev;
	struct class	*stepper1_class;
};

unsigned char stepper1_A_B_chan[16]=
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

unsigned char stepper1_C_D_chan[16] = 
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
static struct spi_device *stepper1_spi;
static struct mutex stepper1_lock;

static int stepper1_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static int stepper1_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper1_read(struct file *file, char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper1_write(struct file *file, const char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;	
	return ret;
}

static long stepper1_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int count;
	int status;
	int itcount = -1;
	char dir;
	char motorNum;
	static unsigned char st1_pos_1 = 0;
	static unsigned char st1_pos_2 = 0;
	struct spi_device *spi;
	
	switch(cmd) 
	{
		case STEPPER_1_2_ROTATE_45:
			itcount = 1;	
		     break;
		case STEPPER_1_2_ROTATE_90:
			itcount = 100;
		     break;
		case STEPPER_1_2_ROTATE_180:
			itcount = 200;
		     break;			
		case STEPPER_1_2_ROTATE_270:
			itcount = 300;
		     break;
		case STEPPER_1_2_ROTATE_360:
			itcount = 400;
		     break;
	    	case STEPPER_1_2_ROTATE_OFF:
			itcount = 0;
		     break;
		default:
			return -EINVAL;
	};
	
	if(itcount == -1)
	{
		return -EINVAL;
	}

	motorNum = ((char )(arg >> 4)) & 0x01; 

	if(itcount == 0)
	{
		spi = spi_dev_get(stepper1_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper1_lock);
		
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
        	
        	mutex_unlock(&stepper1_lock);

		//printk("Motors off for stepper 1\n");

		return 0; 
	}

	//motorNum = ((char )(arg >> 4)) & 0x01;		

	if(motorNum == 1) // C_D_Channel
	{	
		dir = ((char )arg) & 0x01;

		//printk("command for C_D channel, dir = %d\n",dir);

		spi = spi_dev_get(stepper1_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper1_lock);

		for(count = 0; count < itcount; count++)
        	{
			if(dir)
			{
				if( st1_pos_1 < 7 )
		    			st1_pos_1++;
		    		else
		    			st1_pos_1 = 0;
			}  
			else
			{
				if( st1_pos_1 > 0 )
		    			st1_pos_1--;
		    		else
		    			st1_pos_1 = 7;
			}

		
			tx_buf[0] = stepper1_C_D_chan[st1_pos_1 * 2];
			tx_buf[1] = stepper1_C_D_chan[(st1_pos_1 * 2) + 1];

			status = spi_write(spi, tx_buf, 2);

        		if(status != 0)
            			printk("spi_write is failed\n");
	
			udelay(2000);				
		}

        	mutex_unlock(&stepper1_lock);
	}
	else if (motorNum == 0) // A_B_Channel
	{
		dir = ((char )arg) & 0x01;

		//printk("command for A_B channel, dir = %d\n",dir);

        	spi = spi_dev_get(stepper1_spi);

        	if(spi == NULL)
        		return -ENODEV;

        	mutex_lock(&stepper1_lock);


        	for(count = 0; count < itcount; count++)
        	{
                	if(dir)
                	{
                        	if( st1_pos_2 < 7 )
                                	st1_pos_2++;
                        	else
                                	st1_pos_2 = 0;
                	}
                	else
                	{
                        	if( st1_pos_2 > 0 )
                                	st1_pos_2--;
                        	else
                                	st1_pos_2 = 7;
                	}

			tx_buf[0] = stepper1_A_B_chan[st1_pos_2 *2];
                	tx_buf[1] = stepper1_A_B_chan[(st1_pos_2 *2) + 1];

                	status = spi_write(spi, tx_buf, 2);

                	if(status != 0)
                		printk("spi_write is failed\n");

			 udelay(2000);
         	}

         	mutex_unlock(&stepper1_lock);
	}
	else
	{
		/* Do nothing */
	}	

	return 0;
}
 
static struct file_operations stepper1_fops = {
	.open 		= stepper1_open,
	.release	= stepper1_release,
	.read		= stepper1_read,
	.write		= stepper1_write,
	.unlocked_ioctl = stepper1_ioctl,
};

static const struct of_device_id stepper1_spi_dt_ids[] = {
        { .compatible = "stepper1"},
        {}
};
MODULE_DEVICE_TABLE(of, stepper1_spi_dt_ids);

static int stepper1_probe(struct spi_device *spi)
{
	struct stepper1_data *stepper1_data;
	int ret;

#if DEBUG_STEPEER1
	printk("func:%s==>line:%d\n", __FUNCTION__,__LINE__);	
#endif

	if(spi->dev.of_node && !of_match_device(stepper1_spi_dt_ids, &spi->dev)) {
                dev_err(&spi->dev, "ERROR DT: stepper1 listed directly in DT\n");
                WARN_ON(spi->dev.of_node &&
                        !of_match_device(stepper1_spi_dt_ids, &spi->dev));
        }

	stepper1_data = kzalloc(sizeof(*stepper1_data), GFP_KERNEL);
	if(!stepper1_data)
		return -ENOMEM;
	
	stepper1_spi=spi;
	stepper1_spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	
	mutex_init(&stepper1_lock);

	gpio_export(STEPPER1_STB, 1);
    	gpio_direction_output(STEPPER1_STB, 1);
    	gpio_set_value(STEPPER1_STB, 0);

    	gpio_export(STEPPER1_RESET, 1);
    	gpio_direction_output(STEPPER1_RESET, 1);
    	gpio_set_value(STEPPER1_RESET, 0);

    	gpio_export(STEPPER1_SLEEP, 1);
    	gpio_direction_output(STEPPER1_SLEEP, 1);
    	gpio_set_value(STEPPER1_SLEEP, 0);
    
    	udelay(1);
    
    	gpio_set_value(STEPPER1_STB, 1);
    	gpio_set_value(STEPPER1_RESET, 1);
    	gpio_set_value(STEPPER1_SLEEP, 1);

	ret = register_chrdev(STEPPER1_MAJOR, "stepper1", &stepper1_fops);
    	if (ret < 0)
    		return ret;

        stepper1_data->stepper1_class = class_create(THIS_MODULE, "stepper1");

        if(IS_ERR(stepper1_data->stepper1_class)) 
	{
		ret = -1; 
		goto err;
        }

	stepper1_data->devt = MKDEV(STEPPER1_MAJOR, 0);
        stepper1_data->dev = device_create(stepper1_data->stepper1_class, &spi->dev, stepper1_data->devt, stepper1_data, "stepper1");

        if(IS_ERR(stepper1_data->dev))
	{
                ret = -ENODEV;
		goto err1;
	}
		
	ret = spi_setup(spi);
        if (ret < 0) 
	{
                dev_err(&spi->dev, "SPI setup wasn't successful %d", ret);
                ret = -ENODEV;
        }

	spi_set_drvdata(spi, stepper1_data);
	tx_buf[0] = 0x00;
	tx_buf[1] = 0x00;
	ret = spi_write(spi, tx_buf, 2);
	if(ret < 0)
		printk("spi write error\n");
	
	return ret;
	
err1:
	class_destroy(stepper1_data->stepper1_class);
    	unregister_chrdev(STEPPER1_MAJOR, "stepper1");
	kfree(stepper1_data);
	return ret;

err:
	unregister_chrdev(STEPPER1_MAJOR, "stepper1");
	kfree(stepper1_data);
	return ret;

}

static int stepper1_remove(struct spi_device *spi)
{
	struct stepper1_data *stepper1_data	= spi_get_drvdata(spi);

        device_destroy(stepper1_data->stepper1_class, stepper1_data->devt);
	class_destroy(stepper1_data->stepper1_class);
        unregister_chrdev(STEPPER1_MAJOR, "stepper1");
	kfree(stepper1_data);
	return 0;
}

static struct spi_driver stepper1_spi_driver = {
	.probe 		= stepper1_probe,
	.remove 	= stepper1_remove,
	.driver 	= {
		.name 	= "stepper1",
		.owner	= THIS_MODULE,
	},
};

static int __init stepper1_init(void)
{
	int ret = -1;

	ret = spi_register_driver(&stepper1_spi_driver);
        if (ret < 0) {
		printk("stepper1 SPi driver registration failed\n");
        }
	
	return ret;
}

module_init(stepper1_init);

static void __exit stepper1_exit(void)
{
	spi_unregister_driver(&stepper1_spi_driver);
}

module_exit(stepper1_exit);

MODULE_AUTHOR("sivakoti, sivakoti.danda@cyient.com");
MODULE_DESCRIPTION(" stepper1 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:stepper1");

