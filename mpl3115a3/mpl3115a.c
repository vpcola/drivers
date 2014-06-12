/*
 * Chip I2C Driver
 *
 * Copyright (C) 2014 Vergil Cola (vpcola@gmail.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, version 2 of the License.
 *
 * This driver shows how to create a minimal i2c driver for Raspberry Pi.
 * The arbitrary i2c hardware sits on 0x21 using the MCP23017 chip. 
 *
 * PORTA is connected to output leds while PORTB of MCP23017 is connected
 * to dip switches.
 *
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>

#include "bartemp.h"

#define MPL3115_DEVICE_NAME    "mpl3115"

static const unsigned short normal_i2c[] = { 0x60, I2C_CLIENT_END };

/* Our drivers id table */
static const struct i2c_device_id mpl3115_i2c_id[] = {
    { "mpl3115", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, mpl3115_i2c_id);

#define FIFO_SIZE   512 

/* We start by defining our device variables */
static struct semaphore sem;
static wait_queue_head_t inq;
static struct kfifo tempval_fifo;
static struct device * mpl3115_device = NULL;
static struct class * mpl3115_device_class = NULL;
static int mpl3115_major = 0;

#define STR_VAL_SIZE 30

/* Each client has that uses the driver stores 
 * data in this structure. The information
 * hare is about the i2c client device and its
 * related data/information.
 **/
struct mpl3115_data {
    struct i2c_client * client;
    short int irq_gpio17;
    bool     isaltmode;
    // struct timeval last_update_time;
    char    last_temp_value[STR_VAL_SIZE];
    char    last_altbar_value[STR_VAL_SIZE];
};

/* Define the GPIO that cuases the interrupt */
#define GPIO_INT_GPIO17     17

/* The description of this interrupt */
#define GPIO_INT_GPIO17_DESC    "MPL3115 Sensor Interrupt"

/**
 * After consulting the specs, I now know how we 
 * would be able to represent the altimeter values.
 *
 * The format_altbar_value formats the altimeter/
 * barometer value and outputs the str in the buff
 * parameter. Altimeter emits the value in meters
 * while the barometric pressure is expressed in 
 * pascals.
 **/
size_t format_altbar_value(bool isaltmode, 
    uint32_t value,
    unsigned char * buff,
    size_t bufsiz)
{
    int32_t dec_meters = 0, int_meters = 0;
    uint32_t pascals_int = 0, pascals_dec = 0;
    uint16_t msbcsb = 0;
    uint8_t  lsb = 0;
    // Altimeter/Barometer value is stored
    // in 20 bits.
    //
    // In altitude mode, the format is Q16.4,
    // Integer part in meters is MSB.CSB while
    // LSB contains only bits 7-4 (upper nibble),
    // bits 3-0 is not used.
    //
    // For barometer mode, the format is still
    // 20 bits but in Q18.2 format. The integer
    // part is in MSB.CSB.Upper two bits of LSB.
    // The fractional part is in two bits 5-4
    // in the LSB.
    if (isaltmode)
    {
        // Shift down the value by 8 to get only
        // MSB.CSB (illiminating the LSB) to get the
        // integral part.
        msbcsb = (value >> 8) & 0xFFFF;
        // Once shifted down, compare if the last bit
        // is set, this will tell us that the value
        // is negative. If negative then get the 2's
        // complement
        int_meters = (msbcsb > 0x7FFF) ? 
            (~msbcsb + 1) : msbcsb;

        // Now get the fractional part which is in
        // the lsb, but only the high nibble is 
        // significant, so we shift down by 4.
        lsb = (value & 0xFF) >> 4;
        // altimeter has a resolution of .0625
        // for each increment, so we accumulate
        // if each bit is set.
        if (lsb & 0x1) dec_meters += 625;   // bit 1
        if (lsb & 0x2) dec_meters += 1250;  // bit 2
        if (lsb & 0x4) dec_meters += 2500;  // ..
        if (lsb & 0x8) dec_meters += 5000;  // bit 3

        return snprintf(buff, bufsiz, "%d.%04d M", int_meters, dec_meters);
    }
    else
    {
        // To get the integral part, we need to shift
        // down by 6 to get the MSB.CSB.Upper 2 bits.
        // The pressure value is in pascals.
        pascals_int = (value >> 6) & 0x3FFFF;
        // Fractional part is in bits 5 & 4 in the lsb,
        // so we mask the high nibble by 0x30 (leaving
        // only bits 5-4, and shift down by 4.
        lsb = (value & 0x30) >> 4;
        // Pascals dec part has a resolution of .25
        // We conver this to hundreths.
        if (lsb & 0x01) pascals_dec += 25; // bit 1
        if (lsb & 0x02) pascals_dec += 50; // bit 2

        return snprintf(buff, bufsiz, "%d.%02d P", pascals_int, pascals_dec);
    }
}

size_t format_temperature_value( 
        uint32_t value,
        unsigned char * buff,
        size_t bufsiz)
{
    int8_t temp_valmsb;
    uint8_t temp_vallsb;
    
    temp_valmsb = (value >> 8) & 0xFF;

    // The dec part resolution is by factor of 256.
    // but since we need to represent the decimal 
    // part in hundreths, we multiply it by a 
    // hundred first, then shifting by 8 (divide by
    // 256) will get the smae effect.
    temp_vallsb = ((value & 0xFF) * 100) >> 8;

    // Copy the temperature value
    return snprintf(buff, bufsiz, "%d.%02d C",
            temp_valmsb,
            temp_vallsb);
}


/* Our driver attributes/variables are currently exported via sysfs. 
 * For this driver, we export the following attributes
 *
 * altbarmode - (R/W) sets the current altimeter/barometer mode.
 * altbarvalue - The value of the altimeter/barometer in meters/pascals.
 * tempvalue - The temperature value in centigrade.
 **/
static ssize_t set_altbar_mode(struct device *dev, 
        struct device_attribute * devattr,
        const char * buf, 
        size_t count)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s with [%c], current mode [%c]\n", 
        __FUNCTION__,
        buf[0],
        (data->isaltmode ? 'A' : 'B')
        );

    /* Determine if we use B - barometer or A - Altitude
     * mode. We only take the first char, discard all other
     * values of the buffer
     */
    if ((buf[0] != 'A') && (buf[0] != 'B'))
        return -EINVAL;

    disable_irq(data->irq_gpio17);
    // We need to disable interrups from generating on the device
    dev_info(&client->dev, "disabling interrupts on the device\n");
    disable_drdy_interrupt(client);
    mpl_standby(client);
    // Call init to initialize in barometer mode
    data->isaltmode = (buf[0] == 'A');
    // Calling init again will activate the device
    init(client, data->isaltmode); // true - altitude mode, false - barometer mode


    dev_dbg(&client->dev, "%s: Setting altitude/barometer mode to [%s]\n", 
            __FUNCTION__,
            data->isaltmode ? "Altitude":"Barometer");

    // re-enable interrupts again
    enable_irq(data->irq_gpio17);

    return count;
}

/* Gets the altimeter mode. 'B' stands for barometric pressure
 * 'A' for altimeter.
 */
static ssize_t get_altbar_mode(struct device *dev, 
        struct device_attribute * devattr,
        char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    return sprintf(buf, "%c\n", data->isaltmode ? 'A' : 'B');
}

/**
 * Sets the step time - the time interval between
 * sensor reads.  By default, the step time interval
 * is expressed as 2^x seconds, where is x is the value
 * passed into control register 2 of the sensor.
 *
 * The step time dictates how our interrupt will
 * be called periodically. Since 2^0 is 1, this explains
 * why our interrupt is called approximately every 1
 * second.
 **/
static ssize_t set_step_time(struct device *dev, 
        struct device_attribute * devattr,
        const char * buf, 
        size_t count)
{
    int err;
    int val = 0;
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    err = kstrtoint(buf, 10, &val);
    if (err < 0)
        return err;
    /* register data only set 4 bits
     * so filter the value here
     */
    if ((val > 15) || (val < 0))
        return -EINVAL;

    disable_irq(data->irq_gpio17);
    set_devsteptime(client, val);
    // set_devsteptime calls standy, so enable it
    // after the call.
    mpl_activate(client);
    // re-enable interrupts again
    enable_irq(data->irq_gpio17);

    return count;
}

/**
 * Returns the current step time value to the user.
 **/
static ssize_t get_step_time(struct device *dev, 
        struct device_attribute * devattr,
        char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);

    return sprintf(buf, "%d\n", 
        get_devsteptime(client));
}

/* Gets the value of the barometer/altimeter. The value
 * is returned to the client in terms of the mode.
 *
 * For pressure, the value is returned in pascals (postfix
 * P) appended to the actual value string. For meters, 
 * a postfix char of M is appended.
 */
static ssize_t get_altbar_value(struct device *dev, 
        struct device_attribute * devattr,
        char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);


    return sprintf(buf, "%s\n", 
        data->last_altbar_value);
}

/*
 * Returns the value of the temperature in degrees Celcius.
 * The value is postfixed with 'C'.
 */
static ssize_t get_temp_value(struct device *dev, 
    struct device_attribute *dev_attr,
    char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    return sprintf(buf, "%s\n", 
        data->last_temp_value
        );
}

/* attribute to set/get the altitude or barometer mode */
static DEVICE_ATTR(altbarmode, S_IWUGO | S_IRUGO, get_altbar_mode, set_altbar_mode);
/* attribute to set/get the step time */
static DEVICE_ATTR(steptime, S_IWUGO | S_IRUGO, get_step_time, set_step_time);
/* TODO: implement attribute to set/get the osr rate */
// static DEVICE_ATTR(osrrate, S_IWUGO | S_IRUGO, get_osr_rate, set_osr_rate);

/* attribute to read the altitue/barometer value */
static DEVICE_ATTR(altbarvalue, S_IRUGO, get_altbar_value, NULL);
/* attribute to read the temperature value */
static DEVICE_ATTR(tempvalue, S_IRUGO, get_temp_value, NULL);

/*
 * This function is called when the device fils (/dev/mpl3115)
 * is opened. The device file is only readable, so we return
 * an error if its opened in other ways.
 */
static ssize_t fifo_open(struct inode * inode, struct file * filep)
{
    if (((filep->f_flags & O_ACCMODE) == O_WRONLY)
        || ((filep->f_flags & O_ACCMODE) == O_RDWR))
    {
        printk(KERN_WARNING "Write access is prohibited!\n");
        return -EACCES;
    }
    
    return 0;
}

/*
 * This function is called whenever the device file is read.
 * The read function simply reads the data in the kfifo
 * and return the data to the user. If the kfifo is empty,
 * we go into an interruptible wait until there is data
 * in the fifo.
 */
static ssize_t fifo_read(struct file * file,
    char __user *buf,
    size_t count,
    loff_t * ppos)
{
    int retval;
    unsigned int copied;

    if (down_interruptible(&sem))
        return -ERESTARTSYS;

    // Return busy if fifo is empty
    while(kfifo_is_empty(&tempval_fifo))
    {
        up(&sem);

        if(file->f_flags & O_NONBLOCK)
            return -EAGAIN;

        if(wait_event_interruptible(inq, !kfifo_is_empty(&tempval_fifo)))
            return -ERESTARTSYS;

        if(down_interruptible(&sem))
            return -ERESTARTSYS;
    }

    /* Ok, we have data. Transfer it to the user */
    retval = kfifo_to_user(&tempval_fifo, buf, count, &copied);

    up(&sem);

    return retval ? retval : copied;                            

}
/*
 * If our file is opened through the /dev/poll mechanism, 
 * we only update the readable mask whenever there is data 
 * available in the kfifo. The poll_wait will wait until
 * there is data in the fifo, or when inq is signalled.
 */
static unsigned int fifo_poll(struct file * file, poll_table * wait)
{
    unsigned int mask = 0;

    down(&sem);
    poll_wait(file, &inq, wait);
    if(!kfifo_is_empty(&tempval_fifo))
        mask |= POLLIN | POLLRDNORM;
    up(&sem);
    return mask;
}

/**
 * The IOCTL calls only handle one command - to set the 
 * altimeter to barometer/altimeter mode. 
 **/

/**
 * FIFO operations of our char device driver
 **/
static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = fifo_read,
    .open = fifo_open,
    .poll = fifo_poll,
 //   .ioctl = fifo_ioctl,
    .llseek = noop_llseek,
};



/** 
 * Our ISR function. Since this is called from a 
 * threaded irq, we can safely communicate with the 
 * slower I2C bus.
 *
 * The irq function reads the data from our sensors,
 * format the returned values and push the values
 * to the kfifo.
 **/
static irqreturn_t gpio17_isr(int irq, void * dev_id)
{
    unsigned char buff[100];
    uint32_t temp_value, alt_value, len;

    struct mpl3115_data * data = dev_id;
    struct i2c_client * client = data->client;

    // First check if the data is coming from DYRDY
    if (is_data_ready_set(client))
    {
        temp_value = read_temp(client);
        alt_value = read_altbar(client);

        format_altbar_value(data->isaltmode,
            alt_value,
            data->last_altbar_value,
            STR_VAL_SIZE);

        format_temperature_value(
            temp_value,
            data->last_temp_value,
            STR_VAL_SIZE);

        len = sprintf(buff, "%s|%s\n", 
            data->last_temp_value,
            data->last_altbar_value
            );

        if (down_interruptible(&sem))
            return -ERESTARTSYS;
    
        // continue putting data, disregard if fifo is full
        kfifo_in(&tempval_fifo, buff, len);

        // release mutex/semaphore
        up(&sem);

        // Wake up readers
        wake_up_interruptible(&inq);
    }

    return IRQ_HANDLED;

}


/** 
 * The following functions are callback functions of our driver. 
 * Upon successful detection of kernel (via the mpl3115_i2c_detect 
 * function below). The kernel calls the mpl3115_i2c_probe(), the 
 * driver's duty here is to allocate the client's data, initialize
 * the data structures needed, and to call initialize the sensor
 * init(client, data->isalmode) will initialize our hardware. 
 *
 * This function also creates our char device driver and create
 * a device file in /dev/mpl3115. Once the char driver is registerd
 * it will then register an interrupt handler, the interrupt source
 * is coming from GPIO pin #17, that too is allocated and registered.
 *
 * Towards the end of this function, the device files and sysfs 
 * entries are registered.
 **/
static int mpl3115_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct device * dev = &client->dev;
    struct mpl3115_data *data = NULL;
    int error;

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    /* Allocate the client's data here */
    data = kzalloc(sizeof(struct mpl3115_data), GFP_KERNEL);
    if(!data)
    {
        dev_err(&client->dev, "Error allocating memory!\n");
        return -ENOMEM;
    }

    /* Initialize client's data to default */
    data->client = client; // Loopback

    /* initialize our hardware */
    data->isaltmode = false;
    dev_info(&client->dev, "Initializing device with default barometer mode\n");
    init(client, data->isaltmode);

    /* This section creates the char device */
    mpl3115_major = register_chrdev(0, MPL3115_DEVICE_NAME,
        &fops);
    if (mpl3115_major < 0)
    {
        error = mpl3115_major;
        dev_err(&client->dev, "Error registering char dev\n");
        goto error_freemem;
    }

    mpl3115_device_class = class_create(THIS_MODULE,
        MPL3115_DEVICE_NAME);
    if(IS_ERR(mpl3115_device_class))
    {
        error = PTR_ERR(mpl3115_device_class);
        dev_err(&client->dev, "Error registering device class\n");
        goto error_unregchrdev;
    }

    mpl3115_device = device_create(mpl3115_device_class,
        NULL,
        MKDEV(mpl3115_major, 0),
        NULL,
        MPL3115_DEVICE_NAME);
    if(IS_ERR(mpl3115_device))
    {
        error = PTR_ERR(mpl3115_device);
        dev_err(&client->dev, "Error creating device file\n");
        goto error_unregclass;
    }
    dev_info(&client->dev, "Device created with major [%d]\n", mpl3115_major);

    /* Initialize the kfifo used */
    if (kfifo_alloc(&tempval_fifo, FIFO_SIZE, GFP_KERNEL))
    {
        dev_err(&client->dev,"Can not allocate kernel fifo!\n");
        error = -ENOMEM;
        goto error_freegpio;
    }
    /* Initialize the wait queue and semaphore */
    init_waitqueue_head(&inq);

    /* Initialize the semaphore used */
    sema_init(&sem,1);
    dev_info(&client->dev, "Initialize kfifo\n");

    if ((error = gpio_request(GPIO_INT_GPIO17, GPIO_INT_GPIO17_DESC)) < 0)
    {
        dev_err(&client->dev,"GPIO request failure\n");
        goto error_unregclass;
    }

    if ((data->irq_gpio17 = gpio_to_irq(GPIO_INT_GPIO17)) < 0)
    {
        dev_err(&client->dev,"GPIO to IRQ mapping failure\n");
        error = data->irq_gpio17;
        goto error_freegpio;
    }

    dev_info(&client->dev, "Mapped interrupt %d\n",
        data->irq_gpio17);

    i2c_set_clientdata(client, data);
    
    error = request_threaded_irq(data->irq_gpio17, NULL,
        gpio17_isr,
        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
        client->name, data);
    if (error < 0)
    {
        dev_err(&client->dev, "Unable to request IRQ\n");
        goto error_freekfifo;
    }

    dev_info(&client->dev, "Creating sys entries\n");
    // We now register our sysfs attributs. 
    device_create_file(dev, &dev_attr_altbarmode);
    device_create_file(dev, &dev_attr_steptime);
    device_create_file(dev, &dev_attr_altbarvalue);
    device_create_file(dev, &dev_attr_tempvalue);

    return 0;

error_freekfifo:
    kfifo_free(&tempval_fifo);
error_freegpio:
    gpio_free(GPIO_INT_GPIO17);
error_unregclass:
    class_unregister(mpl3115_device_class);
    class_destroy(mpl3115_device_class);
error_unregchrdev:
    unregister_chrdev(mpl3115_major, MPL3115_DEVICE_NAME);
error_freemem:
    kfree(data);
    return error;
}

/** 
 * This function is called whenever the bus or the driver is
 * removed from the system. We perform cleanup here and 
 * unregister our sysfs hooks/attributes, unregister
 * our char device, disconnect the irqs and free data
 * that has been allocated by our driver.
 **/
static int mpl3115_i2c_remove(struct i2c_client * client)
{
    struct device * dev = &client->dev;
    struct mpl3115_data * data = i2c_get_clientdata(client);

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    dev_info(&client->dev, "removing sys entries\n");
    device_remove_file(dev, &dev_attr_altbarmode);
    device_remove_file(dev, &dev_attr_steptime);
    device_remove_file(dev, &dev_attr_altbarvalue);
    device_remove_file(dev, &dev_attr_tempvalue);


    disable_irq(data->irq_gpio17);

    // We need to disable interrups from generating on the device
    dev_info(&client->dev, "disabling interrupts on the device\n");
    disable_drdy_interrupt(client);
    mpl_standby(client);

    dev_info(&client->dev, "freeing irq %d\n", data->irq_gpio17);
    free_irq(data->irq_gpio17, data);


    // we first need to free the gpio lines
    dev_info(&client->dev, "freeing gpio lines\n");
    gpio_free(GPIO_INT_GPIO17);

    // Free the kfifo entry
    dev_info(&client->dev, "freeing kfifo\n");
    kfifo_free(&tempval_fifo);

    // Remove char dev
    dev_info(&client->dev, "unregistering char dev\n");
    device_destroy(mpl3115_device_class, MKDEV(mpl3115_major, 0));
    class_unregister(mpl3115_device_class);
    class_destroy(mpl3115_device_class);
    unregister_chrdev(mpl3115_major, MPL3115_DEVICE_NAME);

    // Finally free the data allocated by the device
    dev_info(&client->dev, "freeing kernel memory\n");
    kfree(data);

    dev_info(&client->dev, "driver removed\n");

    return 0;
}

/**
 * This callback function is called by the kernel 
 * to detect the chip at a given device address. 
 * However since we know that our device is currently 
 * hardwired to 0x60, there is really nothing to detect.
 *
 * For validity, the device also checks the signature
 * register by calling is_valid_device().
 **/
static int mpl3115_i2c_detect(struct i2c_client * client, 
    struct i2c_board_info * info)
{
    struct i2c_adapter *adapter = client->adapter;
    int address = client->addr;
    const char * name = NULL;

    dev_info(&client->dev,"%s\n", __FUNCTION__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENODEV;

    // Since our address is hardwired to 0x21
    // we update the name of the driver. This must
    // match the name of the chip_driver struct below
    // in order for this driver to be loaded.
    if ((address == MPL3115A2_IIC_ADDRESS)
        && is_valid_device(client))
    {
        name = MPL3115_DEVICE_NAME;
        dev_info(&adapter->dev,
            "Chip device found at 0x%02x\n", address);
    }else
        return -ENODEV;

    /* Upon successful detection, we coup the name of the
     * driver to the info struct.
     **/
    strlcpy(info->type, name, I2C_NAME_SIZE);
    return 0;
}


/**
 * This is the main driver description table. It lists 
 * the device types, and the callback functions for this
 * device driver
 **/
static struct i2c_driver mpl3115_driver = {
    .class      = I2C_CLASS_HWMON,
    .driver = {
            .name = MPL3115_DEVICE_NAME,
    },
    .probe          = mpl3115_i2c_probe,
    .remove         = mpl3115_i2c_remove,
    .id_table       = mpl3115_i2c_id,
    .detect         = mpl3115_i2c_detect,
    .address_list   = normal_i2c,
};


module_i2c_driver(mpl3115_driver);

MODULE_AUTHOR("Vergil Cola <vpcola@gmail.com>");
MODULE_DESCRIPTION("MPL3115a I2C Driver");
MODULE_LICENSE("GPL");


