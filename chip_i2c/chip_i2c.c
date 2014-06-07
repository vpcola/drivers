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


#define LED_I2C_DEVICE_NAME    "chip_i2c"

/* Define the addresses to scan. Of course, we know that our
 * hardware is found on 0x21, the chip_i2c_detect() function
 * below is used by the kernel to enumerate the i2c bus, the function
 * returns 0 for success or -ENODEV if the device is not found.
 * The kernel enumerates this array for i2c addresses. This 
 * structure is also passed as a member to the i2c_driver struct.
 **/
static const unsigned short normal_i2c[] = { 0x20, 0x21, I2C_CLIENT_END };

/* Our drivers id table */
static const struct i2c_device_id chip_i2c_id[] = {
    { "chip_i2c", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, chip_i2c_id);

/* Each client has that uses the driver stores data in this structure */
struct chip_data {
    struct i2c_client * client;
    short int irq_gpio23;
	uint8_t  count;	/* In jiffies */
	int kind;
};

/* Define the GPIO that cuases the interrupt */
#define GPIO_INT_GPIO23     23

/* The description of this interrupt */
#define GPIO_INT_GPIO23_DESC    "GPIO 23 Interrupt source"

/* We define the MCP23017 registers. We only need to set the
 * direction registers for input and output
 **/
#define REG_CHIP_DIR_PORTA	0x00
#define REG_CHIP_DIR_PORTB  0x01

#define REG_CHIP_PORTA_LIN  0x12
#define REG_CHIP_PORTB_LIN  0x13
#define REG_CHIP_PORTA_LOUT	0x14
#define REG_CHIP_PORTB_LOUT 0x15


/* Input/Output functions of our driver to read/write
 * data on the i2c bus. We us the i2c_smbus_read_byte_data()
 * and i2c_smbus_write_byte_data() (i2c.h) for doing the 
 * low level i2c read/write to our device. To make sure no 
 * other client is writing/reading from the device at the same time, 
 * we use the client data's mutex for synchronization.
 *
 * The chip_read_value() function reads the status of the
 * dip switches connected to PORTB of MCP23017 while the
 * chip_write_value() sets the value of PORTA (leds).
 */
int chip_read_value(struct i2c_client *client, u8 reg)
{
    int val = 0;

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    val = i2c_smbus_read_byte_data(client, reg);

    dev_info(&client->dev, "%s : read reg [%02x] returned [%d]\n", 
            __FUNCTION__, reg, val);

    return val;
}

int chip_write_value(struct i2c_client *client, u8 reg, u16 value)
{
    int ret = 0;

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    ret =  i2c_smbus_write_byte_data(client, reg, value);

    dev_info(&client->dev, "%s : write reg [%02x] with val [%02x] returned [%d]\n", 
            __FUNCTION__, reg, value, ret);

    return ret;
}


/* Our driver attributes/variables are currently exported via sysfs. 
 * For this driver, we export two attributes - chip_led and chip_switch
 * to correspond to MCP23017's PORTA (led) and PORTB(dip switches).
 *
 * The sysfs filesystem is a convenient way to examine these attributes
 * in kernel space from user space. They also provide a mechanism for 
 * setting data form user space to kernel space. 
 **/
static ssize_t set_chip_led(struct device *dev, 
    struct device_attribute * devattr,
    const char * buf, 
    size_t count)
{
    struct i2c_client * client = to_i2c_client(dev);
    struct chip_data * data = i2c_get_clientdata(client);
    int value, err;

    dev_dbg(&client->dev, "%s\n", __FUNCTION__);

    err = kstrtoint(buf, 10, &value);
    if (err < 0)
        return err;

    data->count = (uint8_t) value; // discard overflow

    dev_dbg(&client->dev, "%s: write to i2c with val %d\n", 
        __FUNCTION__,
        data->count);

    chip_write_value(client, REG_CHIP_PORTA_LOUT, (u16) data->count);

    return count;
}

static ssize_t get_chip_switch(struct device *dev, 
    struct device_attribute *dev_attr,
    char * buf)
{
    struct i2c_client * client = to_i2c_client(dev);
    int value = 0;

    dev_dbg(&client->dev, "%s\n", __FUNCTION__);

    value = chip_read_value(client, REG_CHIP_PORTB_LIN);

    dev_info(&client->dev,"%s: read returned with %d!\n", 
        __FUNCTION__, 
        value);
    // Copy the result back to buf
    return sprintf(buf, "%d\n", value);
}

/* chip led is write only */
static DEVICE_ATTR(chip_led, S_IWUGO, NULL, set_chip_led);
/* chip switch is read only */
static DEVICE_ATTR(chip_switch, S_IRUGO, get_chip_switch, NULL);


/* This function is called to initialize our driver chip
 * MCP23017.
 *
 * For MCP23017 to function, we first need to setup the 
 * direction register at register address 0x0 (PORTA) and
 * 0x01 (PORTB). Bit '1' represents input while '0' is latched
 * output, so we need to write 0x00 for PORTA (led out), and
 * all bits set for PORTB - 0xFF.
 */
static void chip_init_client(struct i2c_client *client)
{
    /* Set the direction registers to PORTA = out (0x00),
     * PORTB = in (0xFF)
     */
    dev_info(&client->dev, "%s\n", __FUNCTION__);

    chip_write_value(client, REG_CHIP_DIR_PORTA, 0x00);
    chip_write_value(client, REG_CHIP_DIR_PORTB, 0xFF);

    // Now clear our leds
    chip_write_value(client, REG_CHIP_PORTA_LIN, 0x00);
}

/* Our ISR function. Here we can safely communicate with the 
 * I2C bus.
 */
static irqreturn_t gpio23_isr(int irq, void * dev_id)
{
    struct chip_data * data = dev_id;
    struct i2c_client * client = data->client;

    // Here we simply output the value of count to
    // our leds
    data->count++;
    dev_info(&client->dev, "Interrupt! (value=%d)\n", data->count);
    chip_write_value(client, REG_CHIP_PORTA_LOUT, data->count);

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
static int chip_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct device * dev = &client->dev;
    struct chip_data *data = NULL;
    int error;

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    /* Allocate the client's data here */
    data = devm_kzalloc(&client->dev, sizeof(struct chip_data), GFP_KERNEL);
    if(!data)
        return -ENOMEM;

    /* Initialize client's data to default */
    data->client = client; // Loopback
    data->kind = id->driver_data;
    data->count = 0; // Number of time push button was pressed since loaded.

    /* initialize our hardware */
    chip_init_client(client);

    if ((error = gpio_request(GPIO_INT_GPIO23, GPIO_INT_GPIO23_DESC)) < 0)
    {
        dev_err(&client->dev,"GPIO request failure\n");
        goto error_freemem;
    }

    if ((data->irq_gpio23 = gpio_to_irq(GPIO_INT_GPIO23)) < 0)
    {
        dev_err(&client->dev,"GPIO to IRQ mapping failure\n");
        error = data->irq_gpio23;
        goto error_freegpio;
    }

    dev_info(&client->dev, "Mapped interrupt %d\n",
        data->irq_gpio23);

    i2c_set_clientdata(client, data);
    
    error = request_threaded_irq(data->irq_gpio23, NULL,
        gpio23_isr,
        IRQF_TRIGGER_RISING | IRQF_ONESHOT,
        client->name, data);
    if (error < 0)
    {
        dev_err(&client->dev, "Unable to request IRQ\n");
        goto error_freegpio;
    }

    // We now register our sysfs attributs. 
    device_create_file(dev, &dev_attr_chip_led);
    device_create_file(dev, &dev_attr_chip_switch);

    return 0;

error_freegpio:
    gpio_free(GPIO_INT_GPIO23);

error_freemem:
    kfree(data);
    return error;

}

/* This function is called whenever the bus or the driver is
 * removed from the system. We perform cleanup here and 
 * unregister our sysfs hooks/attributes.
 **/
static int chip_i2c_remove(struct i2c_client * client)
{
    struct device * dev = &client->dev;
    struct chip_data * data = i2c_get_clientdata(client);

    dev_info(&client->dev, "%s\n", __FUNCTION__);

    dev_info(&client->dev, "removing sys entries\n");
    device_remove_file(dev, &dev_attr_chip_led);
    device_remove_file(dev, &dev_attr_chip_switch);

    dev_info(&client->dev, "freeing irqs\n");
    free_irq(data->irq_gpio23, data);

    // we first need to free the gpio lines
    gpio_free(GPIO_INT_GPIO23);
    dev_info(&client->dev, "freeing gpio lines\n");

    // Before we go out completely, we reset our leds 
    // back to off
    chip_write_value(client, REG_CHIP_PORTA_LOUT, 0x00);

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
static int chip_i2c_detect(struct i2c_client * client, 
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
    if (address == 0x21)
    {
        name = LED_I2C_DEVICE_NAME;
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
static struct i2c_driver chip_driver = {
    .class      = I2C_CLASS_HWMON,
    .driver = {
            .name = LED_I2C_DEVICE_NAME,
    },
    .probe          = chip_i2c_probe,
    .remove         = chip_i2c_remove,
    .id_table       = chip_i2c_id,
    .detect         = chip_i2c_detect,
    .address_list   = normal_i2c,
};

module_i2c_driver(chip_driver);

MODULE_AUTHOR("Vergil Cola <vpcola@gmail.com>");
MODULE_DESCRIPTION("Chip I2C Driver");
MODULE_LICENSE("GPL");

