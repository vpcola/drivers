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

#include "bartemp.h"


#define MPL3115_DEVICE_NAME    "mpl3115"

static const unsigned short normal_i2c[] = { 0x60, I2C_CLIENT_END };

/* Our drivers id table */
static const struct i2c_device_id mpl3115_i2c_id[] = {
    { "mpl3115", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, mpl3115_i2c_id);

/* Each client has that uses the driver stores data in this structure */
struct mpl3115_data {
    struct i2c_client * client;
    short int irq_gpio17;
    bool     isaltmode;
    struct timeval last_update_time;
    uint32_t tempval;
    uint32_t altbarval;
	int kind;
};

/* Define the GPIO that cuases the interrupt */
#define GPIO_INT_GPIO17     17

/* The description of this interrupt */
#define GPIO_INT_GPIO17_DESC    "MPL3115 Sensor Interrupt"


/* Our driver attributes/variables are currently exported via sysfs. 
 * For this driver, we export two attributes - chip_led and chip_switch
 * to correspond to MCP23017's PORTA (led) and PORTB(dip switches).
 *
 * The sysfs filesystem is a convenient way to examine these attributes
 * in kernel space from user space. They also provide a mechanism for 
 * setting data form user space to kernel space. 
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

static ssize_t get_altbar_mode(struct device *dev, 
        struct device_attribute * devattr,
        char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    return sprintf(buf, "%c\n", data->isaltmode ? 'A' : 'B');
}


static ssize_t get_altbar_value(struct device *dev, 
        struct device_attribute * devattr,
        char * buf)
{
    struct tm tm_lr;
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);

    // We print the last time the value was read 
    time_to_tm(data->last_update_time.tv_sec, 0, &tm_lr);

    // Convert the raw altitude data to 
    // alt/barr type

    return sprintf(buf, "%d:%d:%d:%ld|%d\n", 
        tm_lr.tm_hour, 
        tm_lr.tm_min, 
        tm_lr.tm_sec, 
        data->last_update_time.tv_usec, 
        data->altbarval);

}

/* The value read by the device driver is actually
 * a 16 bit value. Since we can not do floating point
 * in kernel space, user space can use this value
 * to get the value in Fahrenheit using the formula
 *
 * MSB = (value >> 8) & 0xFF;
 * LSB = (value & 0xFF;
 *
 * LSB = (LSB > 99) (LSB/1000) : (LSB/100);
 *
 * Fahrenheit = MSB + LSB;
 */
static ssize_t get_temp_value(struct device *dev, 
    struct device_attribute *dev_attr,
    char * buf)
{
    struct tm tm_lr;
    struct i2c_client * client = to_i2c_client(dev);
    struct mpl3115_data * data = i2c_get_clientdata(client);


    // We print the last time the value was read 
    time_to_tm(data->last_update_time.tv_sec, 0, &tm_lr);

    // Convert the raw altitude data to 
    // alt/barr type

    return sprintf(buf, "%d:%d:%d:%ld|%d\n", 
        tm_lr.tm_hour, 
        tm_lr.tm_min, 
        tm_lr.tm_sec, 
        data->last_update_time.tv_usec, 
        data->tempval);
}

/* attribute to set/get the altitude or barometer mode */
static DEVICE_ATTR(altbarmode, S_IWUGO | S_IRUGO, get_altbar_mode, set_altbar_mode);
/* attribute to read the altitue/barometer value */
static DEVICE_ATTR(altbarvalue, S_IRUGO, get_altbar_value, NULL);
/* attribute to read the temperature value */
static DEVICE_ATTR(tempvalue, S_IRUGO, get_temp_value, NULL);


/* Our ISR function. Here we can safely communicate with the 
 * I2C bus.
 */
static irqreturn_t gpio17_isr(int irq, void * dev_id)
{
    struct mpl3115_data * data = dev_id;
    struct i2c_client * client = data->client;

    // Retrieve data from the device, update the client data
    // and the last time read
    //dev_info(&client->dev, "Interrupt called!\n");
   
    // First check if the data is coming from DYRDY
    if (is_data_ready_set(client))
    {
        //dev_info(&client->dev, "Data ready set, reading temperature and %s values!\n",
        //    data->isaltmode ? "altitude" : "barometric");
        // Read temperature sensor
        data->tempval = read_temp(client);
        data->altbarval = read_altbar(client);

        // Update the last read time
        do_gettimeofday(&(data->last_update_time));
    }

    return IRQ_HANDLED;

}


/* The following functions are callback functions of our driver. 
 * Upon successful detection of kernel (via the chip_detect function below). 
 * The kernel calls the chip_i2c_probe(), the driver's duty here 
 * is to allocate the client's data, initialize
 * the data structures needed, and to call chip_init_client() which
 * will initialize our hardware. 
 *
 * This function is also needed to initialize sysfs files on the system.
 */
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
    data->kind = id->driver_data;
    data->tempval = 0;
    data->altbarval = 0;

    /* initialize our hardware */
    data->isaltmode = false;
    init(client, data->isaltmode);

    if ((error = gpio_request(GPIO_INT_GPIO17, GPIO_INT_GPIO17_DESC)) < 0)
    {
        dev_err(&client->dev,"GPIO request failure\n");
        goto error_freemem;
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
        goto error_freegpio;
    }

    // We now register our sysfs attributs. 
    device_create_file(dev, &dev_attr_altbarmode);
    device_create_file(dev, &dev_attr_altbarvalue);
    device_create_file(dev, &dev_attr_tempvalue);


    return 0;

error_freegpio:
    gpio_free(GPIO_INT_GPIO17);

error_freemem:
    kfree(data);
    return error;

}

/* This function is called whenever the bus or the driver is
 * removed from the system. We perform cleanup here and 
 * unregister our sysfs hooks/attributes.
 **/
static int mpl3115_i2c_remove(struct i2c_client * client)
{
    struct device * dev = &client->dev;
    struct mpl3115_data * data = i2c_get_clientdata(client);

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    disable_irq(data->irq_gpio17);

    // We need to disable interrups from generating on the device
    dev_info(&client->dev, "disabling interrupts on the device\n");
    disable_drdy_interrupt(client);
    mpl_standby(client);

    dev_info(&client->dev, "freeing irq %d\n", data->irq_gpio17);
    free_irq(data->irq_gpio17, data);


    dev_info(&client->dev, "removing sys entries\n");
    device_remove_file(dev, &dev_attr_altbarmode);
    device_remove_file(dev, &dev_attr_altbarvalue);
    device_remove_file(dev, &dev_attr_tempvalue);


    // we first need to free the gpio lines
    gpio_free(GPIO_INT_GPIO17);
    dev_info(&client->dev, "freeing gpio lines\n");

    // Finally free the data allocated by the device
    dev_info(&client->dev, "freeing kernel memory\n");
    kfree(data);

    dev_info(&client->dev, "driver removed\n");

    return 0;
}

/* This callback function is called by the kernel 
 * to detect the chip at a given device address. 
 * However since we know that our device is currently 
 * hardwired to 0x21, there is really nothing to detect.
 * We simply return -ENODEV if the address is not 0x21.
 */
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


/* This is the main driver description table. It lists 
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
MODULE_DESCRIPTION("Chip I2C Driver");
MODULE_LICENSE("GPL");

