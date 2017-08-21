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
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/spi/spi.h>
#include <linux/debugfs.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/param.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/spi/stepper3.h>


/* control register */
#define AD7321_SELECT_CON_MSB_REG	0x80
#define AD7321_SELECT_CON_LSB_REG	0x38

/* Range register */ 
#define AD7321_SELECT_RAG_MSB_REG       0XA8
#define AD7321_SELECT_RAG_LSB_REG	0x00

#define TRUE	0x01

struct adc_data {
	struct input_dev	*input;
	struct spi_message m;
        struct spi_transfer t;
	struct spi_device *spi;
	struct gpio_desc        *gpio;
        const char              *name;
	struct mutex lock;
	struct timer_list timer;
	struct dentry *dir;
	char tx_buf[2];
	char rx_buf[2];
	int irq;
	u16 beam;
	u16 doavgcount;
	unsigned int irq_gpio;
	unsigned int irq_enable;
	unsigned int bselect;
	u16 chopperOn;
};

static int iCount = 0;
static u16 sumOfRef = 0;
static u16 sumOfSam = 0;
static u16 sample = 0;
static u16 reference = 0;

#define EVENT_ON	0x01
#define EVENT_OFF	0x02
#define EVENT_STOP	0x03

/*********************************Start Stepper Motor Driver**************************************/
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

#if 1
unsigned char stepper3_C_D_chan[8] = 
{
        0xF8, 0x08,
        0x02, 0xE8,
        0xB8, 0x08,
        0x03, 0xE8
};
#endif

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
	int itcount = 6;
	char dir;
	struct spi_device *spi;

	dir = 0x01;

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

		udelay(2000);
		udelay(500);
	}
       	mutex_unlock(&stepper3_lock);
	return 0;
}


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
/*********************************End Stepper Motor Driver****************************************/
static int adc_open(struct input_dev *input)
{
	return 0;
}

static void adc_close(struct input_dev *input)
{

}

static int adc_spi_read(struct adc_data *data)
{
	short int value = 0;
	short int temp = 0;
	int ret;

	mutex_lock(&data->lock);
        ret = spi_sync(data->spi, &data->m);
        mutex_unlock(&data->lock);
        if(ret < 0) {
        	printk("ADC spi_sync failed\n");
                return -EIO;
        }

        temp = data->rx_buf[0];
        value = ((((temp << 8) & 0xFFFF) | data->rx_buf[1]));
	return value;
}
static irqreturn_t irq_thread_handler(int irq, void *dev_id)
{
	struct adc_data *data = (struct adc_data *)dev_id;
	u16 value = 0;
        u16 temp = 0;
	u16 avgRefAdc = 0;
	u16 avgSamAdc = 0;
	int ret;
	
	mutex_lock(&data->lock);
	data->rx_buf[0] = 0x00;
	data->rx_buf[1] = 0x00;
	ret = spi_sync(data->spi, &data->m);
        if(ret < 0) {
                printk("ADC spi_sync failed\n");
                goto fail;
        }

        temp = data->rx_buf[0];
        value = ((((temp << 8) & 0xFFFF) | data->rx_buf[1]));

	if(data->beam == EVENT_ON)
	{
		//printk("SamValue:%d\n", value);
        	sumOfSam = sumOfSam + value;
		if(data->doavgcount == EVENT_ON)
	 	{
			avgSamAdc = (sumOfSam / 5);
			sample = avgSamAdc;
			sumOfSam = 0;
			//printk("+++++++++++++++Average Sample Value:%x\n", avgSamAdc);					
		}
	}
	if(data->beam == EVENT_OFF)
	{
		//printk("RefValue:%d\n", value);
		sumOfRef = sumOfRef + value;
                if(data->doavgcount == EVENT_OFF)
		 {
                     	avgRefAdc = (sumOfRef / 5);
			reference = avgRefAdc;
			sumOfRef = 0;                          
                }	
	}
	if(data->chopperOn == EVENT_ON)
	{                     	
		stepper3_chopper_rotate(1);
		udelay(1000);
	}

fail:
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

static irqreturn_t trigger_handler(int irq, void *dev_id)
{
	struct adc_data *data = (struct adc_data *)dev_id;
	iCount++;
	switch(iCount)
	{
		case 1:
			gpio_set_value(data->bselect, 0);
			break;
		case 6:
			break;
		case 7:
		case 8:
		case 9:
		case 10:
			// sample beam
			data->chopperOn = EVENT_STOP;
			data->beam = EVENT_ON;
			data->doavgcount = EVENT_STOP;
			return IRQ_WAKE_THREAD;
			break;
		case 11:
			data->chopperOn = EVENT_STOP;
			data->beam = EVENT_ON;
			data->doavgcount = EVENT_ON;
			return IRQ_WAKE_THREAD;
			break;
		case 13:
			gpio_set_value(data->bselect, 1);
			break;
		case 18:
			break;
		case 19:
		case 20:
		case 21:
		case 22:
			// Reference beam
			data->chopperOn = EVENT_STOP;
			data->beam = EVENT_OFF;
			data->doavgcount = EVENT_STOP;
			return IRQ_WAKE_THREAD;
			break;
		case 23:
			data->chopperOn = EVENT_STOP;
			data->beam = EVENT_OFF;
			data->doavgcount = EVENT_OFF;
			return IRQ_WAKE_THREAD;
			break;
		case 12:
                        data->chopperOn = EVENT_ON;
                        data->beam = EVENT_STOP;
                        data->doavgcount = EVENT_STOP;
			return IRQ_WAKE_THREAD;
 			break;
		case 24:
			data->chopperOn = EVENT_ON;
			data->beam = EVENT_STOP;
			data->doavgcount = EVENT_STOP;
			if(iCount == 24)
			{
				iCount = 0;
			}
			return IRQ_WAKE_THREAD;
                        break;
		default:	
			/* Do nothing */
			break;
 
	}	

	return IRQ_HANDLED;
}
static ssize_t chopper_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
        return 0;
}

static ssize_t chopper_store(struct device *dev, struct device_attribute *attr,
                         const char *buf, size_t count)
{
	struct adc_data *data = dev_get_drvdata(dev);
        unsigned int val;
        int ret;

        ret = kstrtouint(buf, 10, &val);
        if(ret)
                return ret;
        if(val == 1) {
		gpio_set_value(data->bselect, 0);
		iCount = 0;	
	}
        return count;
}

static struct device_attribute dev_attr_chopper = {
        .attr = {
                .name = "chopper",
                .mode = S_IRWXUGO,
        },
        .show   = chopper_show,
        .store  = chopper_store,
};

static ssize_t counter_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	return 0;
}

static ssize_t counter_store(struct device *dev, struct device_attribute *attr,
                         const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if(ret)
		return ret;
	if(val == 1) {
		iCount = 0;
	}
	return count;
}

static struct device_attribute dev_attr_counter = {
	.attr = {
                .name = "counter",
                .mode = S_IRWXUGO,
        },
        .show 	= counter_show,
	.store	= counter_store,
};

static ssize_t adc_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct adc_data *data = dev_get_drvdata(dev);
	
	return sprintf(buf, "%d\n", adc_spi_read(data));
}

struct device_attribute dev_attr_adc = {
        .attr 	= {
                .name = "adc_channel0",
                .mode = S_IRUGO | S_IXUGO,
        },
        .show 	= adc_show,
};

static ssize_t sample_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	return sprintf(buf, "%d", sample);
}

struct device_attribute dev_attr_sample = {
        .attr 	= {
                .name = "sample",
                .mode = S_IRUGO | S_IXUGO,
        },
        .show 	= sample_show,
};

static ssize_t reference_show(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	return sprintf(buf, "%d", reference);
}

struct device_attribute dev_attr_reference = {
        .attr 	= {
                .name = "reference",
                .mode = S_IRUGO | S_IXUGO,
        },
        .show 	= reference_show,
};

static const struct of_device_id adc_spi_dt_ids[] = {
        { .compatible = "adc"},
        {}
};
MODULE_DEVICE_TABLE(of, adc_spi_dt_ids);


static int adc_probe(struct spi_device *spi)
{
	struct adc_data *data;
	struct input_dev *input;
	struct device_node *np = spi->dev.of_node;
	int irq_gpio;
	int bselect;
	int ret;
	
	if(spi->dev.of_node && !of_match_device(adc_spi_dt_ids, &spi->dev)) {
                dev_err(&spi->dev, "ERROR DT: adc listed directly in DT\n");
                WARN_ON(spi->dev.of_node &&
                        !of_match_device(adc_spi_dt_ids, &spi->dev));
        }
	irq_gpio = of_get_named_gpio(np, "gpio", 0);
	
	bselect = of_get_named_gpio(np, "beam-select", 0);
	
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	data->irq_gpio = irq_gpio;	
	data->irq = gpio_to_irq(data->irq_gpio);
	data->bselect = bselect;
	data->spi=spi;
	data->spi->mode = SPI_MODE_2;
	data->chopperOn = EVENT_STOP;
	mutex_init(&data->lock);

	ret = spi_setup(data->spi);
	if (ret < 0) {
                printk(KERN_ERR "SPI setup wasn't successful %d", ret);
                ret = -ENODEV;
        }

	input = input_allocate_device();
	if(!input) {
		printk(KERN_ERR "failed to allocate input device");
		return -ENODEV;
	}
	data->input = input;
	input->name = "adc";
	input->phys = "gpio/input0";
	input->open = adc_open;
	input->close = adc_close;
	input->id.bustype = BUS_SPI;	
	
	input->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

        input_set_abs_params(input, ABS_X, 0, 8200, 0, 0);
        input_set_abs_params(input, ABS_Y, 0, 8200, 0, 0);

	set_bit(KEY_3, input->keybit);

	if (gpio_is_valid(data->irq_gpio)) {
		ret = gpio_request(data->irq_gpio, "trigger_irq_gpio");
		if(ret < 0) {
			printk(KERN_INFO "irq gpio request failed");
			goto free_gpio;
		}
		ret = gpio_direction_input(data->irq_gpio);
		if(ret < 0) {
			printk(KERN_INFO "set direction for irq gpio failed");
			goto free_gpio;
		}
	}
	
	if (gpio_is_valid(data->bselect)) {
                ret = gpio_request(data->bselect, "beam-select");
                if(ret < 0) {
                        printk(KERN_INFO "beam select gpio request failed");
                        goto free_irq_enable;
                }
                ret = gpio_direction_output(data->bselect, 1);
                if(ret < 0) {
                        printk(KERN_INFO "set direction for irq gpio failed");
                        goto free_irq_enable;
                }
        }
	gpio_set_value(data->bselect, 0);
	ret = request_threaded_irq(data->irq, trigger_handler, irq_thread_handler, IRQF_TRIGGER_RISING, "ext-trigger", data);
	if(ret < 0) {
		printk(KERN_INFO "registering the irq is failed");
		goto free_irq;
	}
	
	ret = input_register_device(data->input);
	if (ret < 0) {
		printk(KERN_INFO "Input device registration failed\n");
		goto free_inputdev;
	}

	ret = device_create_file(&spi->dev, &dev_attr_adc);
	if(ret ) {
		dev_err(&spi->dev, "sys file creation failed\n");
                goto free_adc_sys;
	}
	
	ret = device_create_file(&spi->dev, &dev_attr_counter);
        if(ret ) {
                dev_err(&spi->dev, "sys file creation failed\n");
                goto free_counter_sys;
        }

	ret = device_create_file(&spi->dev, &dev_attr_sample);
        if(ret ) {
                dev_err(&spi->dev, "sys file creation failed\n");
                goto free_sample_sys;
        }
	ret = device_create_file(&spi->dev, &dev_attr_reference);
        if(ret ) {
                dev_err(&spi->dev, "sys file creation failed\n");
                goto free_reference_sys;
        }

	ret = device_create_file(&spi->dev, &dev_attr_chopper);
        if(ret ) {
                dev_err(&spi->dev, "sys file creation failed\n");
                goto free_chopper_sys;
        }

	spi_set_drvdata(spi, data);
	
	/* writing data to Range Register */
	data->tx_buf[0] = AD7321_SELECT_RAG_MSB_REG;
	data->tx_buf[1] = AD7321_SELECT_RAG_LSB_REG;

	ret = spi_write(data->spi, data->tx_buf, 2);
	if(ret < 0)
		printk("Error: spi write\n");

	data->tx_buf[0] = AD7321_SELECT_CON_MSB_REG;
	data->tx_buf[1] = AD7321_SELECT_CON_LSB_REG;

	ret = spi_write(data->spi, data->tx_buf, 2);
	if(ret < 0)
		printk("Error: spi write\n");

	data->tx_buf[0] = 0x00;
        data->tx_buf[1] = 0x00;
        data->t.rx_buf = data->rx_buf;
        data->t.tx_buf = data->tx_buf;
	data->t.len = 4;

	spi_message_init(&data->m);
        spi_message_add_tail(&data->t, &data->m);

	return ret;

free_chopper_sys:
	device_remove_file(&spi->dev, &dev_attr_reference);

free_reference_sys:
	device_remove_file(&spi->dev, &dev_attr_sample);

free_sample_sys:
	device_remove_file(&spi->dev, &dev_attr_counter);

free_counter_sys:
	device_remove_file(&spi->dev, &dev_attr_adc);

free_adc_sys:
	input_free_device(data->input);

free_inputdev:
	free_irq(data->irq, data);

free_irq:
	if(gpio_is_valid(data->bselect))
        	gpio_free(data->bselect);

free_irq_enable:
	if(gpio_is_valid(data->irq_gpio))
        	gpio_free(data->irq_gpio);

free_gpio:
	kfree(data);
	return ret;
}


static int adc_remove(struct spi_device *spi)
{
	struct adc_data *data	= spi_get_drvdata(spi);
	
	device_remove_file(&spi->dev, &dev_attr_chopper);	
	device_remove_file(&spi->dev, &dev_attr_reference);	
	device_remove_file(&spi->dev, &dev_attr_sample);
	device_remove_file(&spi->dev, &dev_attr_counter);	
	device_remove_file(&spi->dev, &dev_attr_adc);
	input_unregister_device(data->input);	
	disable_irq(data->irq);
	free_irq(data->irq, data);
	if (gpio_is_valid(data->bselect))
		gpio_free(data->bselect);


        if (gpio_is_valid(data->irq_gpio))
                gpio_free(data->irq_gpio);
	
	input_free_device(data->input);
	kfree(data);
	return 0;
}

static struct spi_driver adc_spi_driver = {
	.probe 		= adc_probe,
	.remove 	= adc_remove,
	.driver 	= {
		.name 	= "adc",
		.owner	= THIS_MODULE,
	},
};

static int __init adc_init(void)
{
	int ret = -1;
	ret = spi_register_driver(&adc_spi_driver);
        if (ret < 0) {
		printk("adc SPi driver registration failed\n");
        }
	
	ret = spi_register_driver(&stepper3_spi_driver);
        if (ret < 0) {
		printk("stepper3 SPi driver registration failed\n");
        }
	
	return ret;
}

module_init(adc_init);

static void __exit adc_exit(void)
{
	spi_unregister_driver(&adc_spi_driver);
	spi_unregister_driver(&stepper3_spi_driver);
}

module_exit(adc_exit);

MODULE_AUTHOR("vasubabu, <vasubabu.kandimalla@cyient.com>");
MODULE_DESCRIPTION(" adc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:adc");

