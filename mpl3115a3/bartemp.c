#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>

#include "bartemp.h"


bool is_valid_device(struct i2c_client * client)
{
    return (i2c_smbus_read_byte_data(client, MPL3115A2_WHO_AM_I) == WHO_AM_I);
}

void init(struct i2c_client * client, bool altmode)
{
    // Set ctrl reg 1, alt or bar mode, 
    // and oversampling ratio. MPL3115A2_CTRL_REG1 
    // is 0x26
    if (altmode)
        i2c_smbus_write_byte_data(client,
            MPL3115A2_CTRL_REG1, 
            (CTRL1_ALT_MASK | CTRL1_OS2_MASK | CTRL1_OS1_MASK | CTRL1_OS0_MASK)
            );
    else
        i2c_smbus_write_byte_data(client,
            MPL3115A2_CTRL_REG1, 
            (CTRL1_OS2_MASK | CTRL1_OS1_MASK | CTRL1_OS0_MASK)
            );


    // Sets the data event flag registers
    // MPL3115A2_PT_DATA_CFG = 0x13
    i2c_smbus_write_byte_data(client, 
            MPL3115A2_PT_DATA_CFG, 
            (CTRL0_DREM_MASK | CTRL0_PDEFE_MASK | CTRL0_TDEFE_MASK)
            );


    // Set ctrl reg 3. Interrupt control reg
    // Set interrupts to active low, open drain
    // MPL3115A2_CTRL_REG3 = 0x28
    i2c_smbus_write_byte_data(client,
            MPL3115A2_CTRL_REG3, 
            (CTRL3_PPOD1_MASK | CTRL3_PPOD2_MASK)
            );

    // Enable the DRDY Interrupt. Right now this is
    // the only flag we enable. There are other
    // interrupt sources such as interrrupt on temp change
    // or barometer change. I will deal with those
    // later.
    i2c_smbus_write_byte_data(client,
            MPL3115A2_CTRL_REG4, 
            CTRL4_INTEN_DRDY_MASK);

    // Set active
    mpl_activate(client);
}

void disable_drdy_interrupt(struct i2c_client * client)
{
    uint8_t value = 0; 

    // Read the value of the register first
    value = i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG4);
    // Mask, disable data ready interrupt
    value &= 0x7F; // Clear most significant bit

    // Enable the DRDY Interrupt
    i2c_smbus_write_byte_data(client,
            MPL3115A2_CTRL_REG4, 
            value);
}

void enable_drdy_interrupt(struct i2c_client * client)
{
    uint8_t value = 0;
    // Read the value of the register first
    value = i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG4);
    // Mask, disable data ready interrupt
    value |= CTRL4_INTEN_DRDY_MASK; // Set most significant bit

    i2c_smbus_write_byte_data(client, 
            MPL3115A2_CTRL_REG4,
            value);
}

// When an interrupt is triggered, we need to know
// what triggered it (or where it comes from). 
// The INT_SOURCE register will give us the answer.
bool is_data_ready_set(struct i2c_client * client)
{
    uint8_t source;

    source = i2c_smbus_read_byte_data(client,
       MPL3115A2_INT_SOURCE);

    return (source & 0x80);

}

void mpl_activate(struct i2c_client * client)
{
    i2c_smbus_write_byte_data(client, 
        MPL3115A2_CTRL_REG1, 
        /* Read the control reg and modify the first bit */
        (i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG1)) | ACTIVE_MASK
        );
}

uint8_t mpl_standby(struct i2c_client * client)
{
    uint8_t n;

    n = i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG1);
    /* Set first bit to zero */
    i2c_smbus_write_byte_data(client, MPL3115A2_CTRL_REG1, n & STANDBY_SBYB_0);

    return n;
}

uint32_t read_altbar(struct i2c_client * client)
{
    uint32_t value = 0;

    value = i2c_smbus_read_byte_data(client,OUT_P_MSB_REG);
    value <<= 8;
    value += i2c_smbus_read_byte_data(client,OUT_P_CSB_REG);
    value <<= 8;
    value += i2c_smbus_read_byte_data(client,OUT_P_LSB_REG);

    return value;
}

uint32_t read_temp(struct i2c_client * client)
{
    uint32_t value = 0;

    value = i2c_smbus_read_byte_data(client,OUT_T_MSB_REG);
    value <<= 8;
    value += i2c_smbus_read_byte_data(client,OUT_T_LSB_REG);

    return value;
}

uint8_t get_devosr(struct i2c_client * client)
{
    return ((i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG1)) >> 3) & 0x07;
}

bool set_devosr(struct i2c_client * client, uint8_t osr)
{
    if (osr < 8)
    {
        osr <<= 3;

        osr |= mpl_standby(client) & CLEAR_OSR;
        i2c_smbus_write_byte_data(client, MPL3115A2_CTRL_REG1, osr);

        return true;
    }else
        return false;
}

uint8_t get_devsteptime(struct i2c_client * client)
{
    return (i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG2) & 0x0F);
}

bool set_devsteptime(struct i2c_client * client, uint8_t step)
{
    if ( step < 0x0f)
    {
        mpl_standby(client);
        step |= i2c_smbus_read_byte_data(client, MPL3115A2_CTRL_REG2) & CLEAR_ST_MASK;
        i2c_smbus_write_byte_data(client, MPL3115A2_CTRL_REG2, step);
        return true;
    }else
        return false;
}

