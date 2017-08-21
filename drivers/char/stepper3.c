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
#include <linux/spi/stepper3.h>

#define STEPPER3_MAJOR	243
#define STEPPER3_STB	71
#define STEPPER3_RESET	172
#define STEPPER3_SLEEP	171

#define  DEBUG_STEPEER3 0 

static unsigned char st3_pos_1 = 0;

struct stepper3_data {
	dev_t devt;
	struct device 	*dev;
	struct class	*stepper3_class;
};

unsigned char stepper3_A_B_chan[16]=
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

#if 0

unsigned char stepper3_C_D_chan[16] = 
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

#endif

//#if 0
unsigned char stepper3_C_D_chan[8] = 
{
        0xF8, 0x08,
        0x02, 0xE8,
        0xB8, 0x08,
        0x03, 0xE8
};
//#endif

static char tx_buf[2];
static struct spi_device *stepper3_spi;
static struct mutex stepper3_lock;

static int stepper3_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static int stepper3_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper3_read(struct file *file, char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}

static ssize_t stepper3_write(struct file *file, const char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}
int stepper3_chopper_rotate(int rcount)
{
	int count;
	int status;
 	struct spi_device *spi;

	spi = spi_dev_get(stepper3_spi);

	printk("received count = %d\n",rcount);

	for(count = 0; count < rcount; count++)
        {
              	if( st3_pos_1 < 3 )
                	st3_pos_1++;
                else
                	st3_pos_1 = 0;
                
                #if 0                       
                if( st3_pos_1 > 0 )
                     st3_pos_1--;
                else
                     st3_pos_1 = 3;
                #endif

                tx_buf[0] = stepper3_C_D_chan[st3_pos_1 * 2];
                tx_buf[1] = stepper3_C_D_chan[(st3_pos_1 * 2) + 1];

                status = spi_write(spi, tx_buf, 2);
                if(status != 0)
                    printk("spi_write is failed\n");

        	udelay(500);
       	}
	
	return 0;
}
EXPORT_SYMBOL(stepper3_chopper_rotate);

static long stepper3_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int count;
	int status;
	int itcount = -1;
	char dir;
	char motorNum;
	static unsigned char st3_pos_2 = 0;
	struct spi_device *spi;


	switch(cmd)
	{
		case STEPPER_5_6_ROTATE_45:
			itcount = 1;	
		     break;
		case STEPPER_5_6_ROTATE_90:
			itcount = 6;
		     break;
		case STEPPER_5_6_ROTATE_180:
			itcount = 200;
		     break;			
		case STEPPER_5_6_ROTATE_270:
			itcount = 300;
		     break;
		case STEPPER_5_6_ROTATE_360:
			itcount = 400;
		     break;
		case STEPPER_5_6_ROTATE_OFF:
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
		spi = spi_dev_get(stepper3_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper3_lock);
		
		 if(motorNum == 1) // C_D_Channel
                {
                        tx_buf[0] = 0x00;
                        tx_buf[1] = 0x08;

			//st3_pos_1 = 0;

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
        	
        	mutex_unlock(&stepper3_lock);

		//printk("Motors off for stepper 3\n");

		return 0; 
	}


        if(motorNum == 1) // C_D_Channel
	{	
		dir = ((char )arg) & 0x01;

		//printk("command for C_D channel, dir = %d\n",dir);

		spi = spi_dev_get(stepper3_spi);

		if(spi == NULL)
			return -ENODEV;

		mutex_lock(&stepper3_lock);

		
		for(count = 0; count < itcount; count++)
        	{
			if(dir)
			{
				if( st3_pos_1 < 3 )
		    			st3_pos_1++;
		    		else
		    			st3_pos_1 = 0;
			}  
			else
			{
				if( st3_pos_1 > 0 )
		    			st3_pos_1--;
		    		else
		    			st3_pos_1 = 3;
			}
	
			//printk("st3_pos_1 = %d\n",st3_pos_1);
	
			tx_buf[0] = stepper3_C_D_chan[st3_pos_1 * 2];
			tx_buf[1] = stepper3_C_D_chan[(st3_pos_1 * 2) + 1];

			status = spi_write(spi, tx_buf, 2);

        		if(status != 0)
                		printk("spi_write is failed\n");
	

			udelay(1800);
		}
        	mutex_unlock(&stepper3_lock);

	}
	else if (motorNum == 0) // A_B_Channel
	{

		dir = ((char )arg) & 0x01;

		//printk("command for A_B channel, dir = %d\n",dir);

                spi = spi_dev_get(stepper3_spi);

                if(spi == NULL)
                        return -ENODEV;

                mutex_lock(&stepper3_lock);

                for(count = 0; count < itcount; count++)
                {
                        if(dir)
                        {
                                if( st3_pos_2 < 7 )
                                        st3_pos_2++;
                                else
                                        st3_pos_2 = 0;
                        }
                        else
                        {
                                if( st3_pos_2 > 0 )
                                        st3_pos_2--;
                                else
                                        st3_pos_2 = 7;
                        }


			tx_buf[0] = stepper3_A_B_chan[st3_pos_2 *2];
                        tx_buf[1] = stepper3_A_B_chan[(st3_pos_2 *2) + 1];

                        status = spi_write(spi, tx_buf, 2);

                        if(status != 0)
                        	printk("spi_write is failed\n");
			
			 udelay(1000);
                }

                mutex_unlock(&stepper3_lock);
	}
	else
	{
		/* Do nothing */
	}


	return 0;
}
 
static struct file_operations stepper3_fops = {
	.open 		= stepper3_open,
	.release	= stepper3_release,
	.read		= stepper3_read,
	.write		= stepper3_write,
	.unlocked_ioctl = stepper3_ioctl,
};

static const struct of_device_id stepper3_spi_dt_ids[] = {
        { .compatible = "stepper3"},
        {}
};
MODULE_DEVICE_TABLE(of, stepper3_spi_dt_ids);

static int stepper3_probe(struct spi_device *spi)
{
	struct stepper3_data *stepper3_data;
	int ret;
	
#if DEBUG_STEPEER3
	printk("func:%s=>Line:%d\n", __FUNCTION__, __LINE__);
#endif

	if(spi->dev.of_node && !of_match_device(stepper3_spi_dt_ids, &spi->dev)) {
                dev_err(&spi->dev, "ERROR DT: STEPPER3 listed directly in DT\n");
                WARN_ON(spi->dev.of_node &&
                        !of_match_device(stepper3_spi_dt_ids, &spi->dev));
        }

	stepper3_data = kzalloc(sizeof(*stepper3_data), GFP_KERNEL);
	if(!stepper3_data)
		return -ENOMEM;
	
	stepper3_spi=spi;
	stepper3_spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	
	mutex_init(&stepper3_lock);
	
	gpio_export(STEPPER3_STB, 1);
	gpio_direction_output(STEPPER3_STB, 1);
	gpio_set_value(STEPPER3_STB, 0);

	gpio_export(STEPPER3_RESET, 1);
	gpio_direction_output(STEPPER3_RESET, 1);
	gpio_set_value(STEPPER3_RESET, 0);

	gpio_export(STEPPER3_SLEEP, 1);
	gpio_direction_output(STEPPER3_SLEEP, 1);
	gpio_set_value(STEPPER3_SLEEP, 0);

	udelay(1);

	gpio_set_value(STEPPER3_STB, 1);
	gpio_set_value(STEPPER3_RESET, 1);
	gpio_set_value(STEPPER3_SLEEP, 1);

		
	ret = register_chrdev(STEPPER3_MAJOR, "stepper3", &stepper3_fops);
        if (ret < 0)
                return ret;

        stepper3_data->stepper3_class = class_create(THIS_MODULE, "stepper3");
        if(IS_ERR(stepper3_data->stepper3_class)) {
		ret = -1; 
		goto err;
        }

	stepper3_data->devt = MKDEV(STEPPER3_MAJOR, 0);
        stepper3_data->dev = device_create(stepper3_data->stepper3_class, &spi->dev, stepper3_data->devt, stepper3_data, "stepper3");
        if(IS_ERR(stepper3_data->dev)){
                ret = -ENODEV;
		goto err1;
	}
	ret = spi_setup(spi);
        if (ret < 0) {
                dev_err(&spi->dev, "SPI setup wasn't successful %d", ret);
                ret = -ENODEV;
        }

	spi_set_drvdata(spi, stepper3_data);
	
	tx_buf[0] = 0x00;
	tx_buf[1] = 0x00;
	
	ret = spi_write(spi, tx_buf, 2);
	if(ret < 0)
		printk("spi write error\n");
		
	return ret;
err1:
	class_destroy(stepper3_data->stepper3_class);
        unregister_chrdev(STEPPER3_MAJOR, "stepper3");
	kfree(stepper3_data);
	return ret;

err:
	unregister_chrdev(STEPPER3_MAJOR, "stepper3");
	kfree(stepper3_data);
	return ret;
}

static int stepper3_remove(struct spi_device *spi)
{
	struct stepper3_data *stepper3_data	= spi_get_drvdata(spi);

    device_destroy(stepper3_data->stepper3_class, stepper3_data->devt);
	class_destroy(stepper3_data->stepper3_class);
    unregister_chrdev(STEPPER3_MAJOR, "stepper3");
	kfree(stepper3_data);
	return 0;
}

static struct spi_driver stepper3_spi_driver = {
	.probe 		= stepper3_probe,
	.remove 	= stepper3_remove,
	.driver 	= {
		.name 	= "stepper3",
		.owner	= THIS_MODULE,
	},
};

static int __init stepper3_init(void)
{
	int ret = -1;

	ret = spi_register_driver(&stepper3_spi_driver);
        if (ret < 0) {
		printk("stepper3 SPi driver registration failed\n");
        }
	
	return ret;
}

module_init(stepper3_init);

static void __exit stepper3_exit(void)
{
	spi_unregister_driver(&stepper3_spi_driver);
}

module_exit(stepper3_exit);

MODULE_AUTHOR("sivakoti, sivakoti.danda@cyient.com");
MODULE_DESCRIPTION(" STEPPER3 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:STEPPER3");

