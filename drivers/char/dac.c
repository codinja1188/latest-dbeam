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

#include "dac.h"

#define DAC_MAJOR	240
static char tx_buf[8];
static struct spi_device *dac_spi;
static struct mutex dac_lock;
static int dac_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static int dac_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}

static ssize_t dac_read(struct file *file, char __user *buf, size_t len, loff_t *f_ops)
{
	int ret = 0;
	return ret;
}

static ssize_t dac_write(struct file *file, const char __user *buf, size_t len, loff_t *f_ops)
{
	int ret;
	int status;
	int size;
	struct spi_device *spi;

	//printk("Length = %d\n", len);
        ret = copy_from_user(tx_buf, buf, len);
        if(ret == 0) {
                //printk("Data copied to kernel buf\n");
		//printk("Drriver:Len:%d\n", len);
                //printk("Driver:tx_buf[0]:%x=>tx_buf[1]:%x\n", tx_buf[0], tx_buf[1]);
                //printk("Driver:tx_buf[2]:%x=>tx_buf[3]:%x\n", tx_buf[2], tx_buf[3]);
        } else {
                ret = -EFAULT;
                //printk("There is no data present in buffer...\n");
        }
	size = len;
	//printk("size = %d\n", size);
	spi = spi_dev_get(dac_spi);
	mutex_lock(&dac_lock);
	status = spi_write(spi, tx_buf, size);
	//printk("Driver:status = %d\n", status);
	if(status < 0)
		//printk("spi_write is success\n");
	//else
		printk("spi_write is failed\n");	
	mutex_unlock(&dac_lock);
	return ret;
}
 
static struct file_operations dac_fops = {
	.open 		= dac_open,
	.release	= dac_release,
	.read		= dac_read,
	.write		= dac_write,
};

static const struct of_device_id dac_spi_dt_ids[] = {
        { .compatible = "dac"},
        {}
};
MODULE_DEVICE_TABLE(of, dac_spi_dt_ids);

static int dac_probe(struct spi_device *spi)
{
	struct dac_data *dac_data;
	int ret;
	
	//printk("func:%s=>Line:%d\n", __FUNCTION__, __LINE__);
	if(spi->dev.of_node && !of_match_device(dac_spi_dt_ids, &spi->dev)) {
                dev_err(&spi->dev, "ERROR DT: DAC listed directly in DT\n");
                WARN_ON(spi->dev.of_node &&
                        !of_match_device(dac_spi_dt_ids, &spi->dev));
        }

	dac_data = kzalloc(sizeof(*dac_data), GFP_KERNEL);
	if(!dac_data)
		return -ENOMEM;

	//dac_data->spi = spi;
	dac_spi=spi;
	dac_spi->mode = SPI_MODE_3;
	mutex_init(&dac_lock);
	ret = register_chrdev(DAC_MAJOR, "dac", &dac_fops);
        if (ret < 0)
                return ret;

        dac_data->dac_class = class_create(THIS_MODULE, "dac");
        if(IS_ERR(dac_data->dac_class)) {
		ret = -1; 
		goto err;
        }

	dac_data->devt = MKDEV(DAC_MAJOR, 0);
        dac_data->dev = device_create(dac_data->dac_class, &spi->dev, dac_data->devt, dac_data, "dac");
        if(IS_ERR(dac_data->dev)){
                ret = -ENODEV;
		goto err1;
	}

	ret = spi_setup(spi);
        if (ret < 0) {
                dev_err(&spi->dev, "SPI setup wasn't successful %d", ret);
                ret = -ENODEV;
        }

	spi_set_drvdata(spi, dac_data);
	return ret;
err1:
	class_destroy(dac_data->dac_class);
        unregister_chrdev(DAC_MAJOR, "dac");
	kfree(dac_data);
	return ret;

err:
	unregister_chrdev(DAC_MAJOR, "dac");
	kfree(dac_data);
	return ret;

}

static int dac_remove(struct spi_device *spi)
{
	struct dac_data *dac_data	= spi_get_drvdata(spi);

	//printk("func:%s=>Line:%d\n", __FUNCTION__, __LINE__);

        device_destroy(dac_data->dac_class, dac_data->devt);
	class_destroy(dac_data->dac_class);
        unregister_chrdev(DAC_MAJOR, "dac");
	kfree(dac_data);
	return 0;
}

static struct spi_driver dac_spi_driver = {
	.probe 		= dac_probe,
	.remove 	= dac_remove,
	.driver 	= {
		.name 	= "dac",
		.owner	= THIS_MODULE,
	},
};

static int __init dac_init(void)
{
	int ret = -1;
	//printk("func:%s=>Line:%d\n", __FUNCTION__, __LINE__);
	ret = spi_register_driver(&dac_spi_driver);
        if (ret < 0) {
		printk("DAC SPi driver registration failed\n");
        }
	
	return ret;
}

module_init(dac_init);

static void __exit dac_exit(void)
{
	spi_unregister_driver(&dac_spi_driver);
}

module_exit(dac_exit);

MODULE_AUTHOR("vasubabu, vasubabu.kandimalla@cyient.com");
MODULE_DESCRIPTION(" DAC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:DAC");

