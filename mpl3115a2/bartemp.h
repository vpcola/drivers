#ifndef __BARTEMP_H_
#define __BARTEMP_H_


#define MPL3115A2_IIC_ADDRESS 0x60
#define MPL3115A2_ID          0xC4

/* Internal sensor registers */
enum
{
    MPL31152A2_STATUS_REG = 0x0,
    MPL3115A2_OUT_P_MSB,              // 0x01 
    MPL3115A2_OUT_P_CSB,              // 0x02  
    MPL3115A2_OUT_P_LSB,              // 0x03  
    MPL3115A2_OUT_T_MSB,              // 0x04  
    MPL3115A2_OUT_T_LSB,              // 0x05  
    MPL3115A2_OUT_DR_STATUS,          // 0x06  
    MPL3115A2_OUT_P_DELTA_MSB,        // 0x07  
    MPL3115A2_OUT_P_DELTA_CSB,        // 0x08  
    MPL3115A2_OUT_P_DELTA_LSB,        // 0x09  
    MPL3115A2_OUT_T_DELTA_MSB,        // 0x0A  
    MPL3115A2_OUT_T_DELTA_LSB,        // 0x0B  
    MPL3115A2_WHO_AM_I,               // 0x0C  
    MPL3115A2_F_STATUS,               // 0x0D  
    MPL3115A2_F_DATA,                 // 0x0E  
    MPL3115A2_F_SETUP,                // 0x0F  
    MPL3115A2_TIME_DLY,               // 0x10  
    MPL3115A2_SYSMOD,                 // 0x11  
    MPL3115A2_INT_SOURCE,             // 0x12  
    MPL3115A2_PT_DATA_CFG,            // 0x13  
    MPL3115A2_BAR_IN_MSB,             // 0x14  
    MPL3115A2_BAR_IN_LSB,             // 0x15  
    MPL3115A2_P_ARM_MSB,              // 0x16  
    MPL3115A2_P_ARM_LSB,              // 0x17  
    MPL3115A2_T_ARM,                  // 0x18  
    MPL3115A2_P_ARM_WND_MSB,          // 0x19  
    MPL3115A2_P_ARM_WND_LSB,          // 0x1A  
    MPL3115A2_T_ARM_WND,              // 0x1B  
    MPL3115A2_P_MIN_MSB,              // 0x1C  
    MPL3115A2_P_MIN_CSB,              // 0x1D  
    MPL3115A2_P_MIN_LSB,              // 0x1E  
    MPL3115A2_T_MIN_MSB,              // 0x1F  
    MPL3115A2_T_MIN_LSB,              // 0x20  
    MPL3115A2_P_MAX_MSB,              // 0x21  
    MPL3115A2_P_MAX_CSB,              // 0x22  
    MPL3115A2_P_MAX_LSB,              // 0x23  
    MPL3115A2_T_MAX_MSB,              // 0x24  
    MPL3115A2_T_MAX_LSB,              // 0x25  
    MPL3115A2_CTRL_REG1,              // 0x26  
    MPL3115A2_CTRL_REG2,              // 0x27  
    MPL3115A2_CTRL_REG3,              // 0x28  
    MPL3115A2_CTRL_REG4,              // 0x29  
    MPL3115A2_CTRL_REG5,              // 0x2A  
    MPL3115A2_OFF_P,                  // 0x2B  
    MPL3115A2_OFF_T,                  // 0x2C  
    MPL3115A2_OFF_H                   // 0x2D
};

/* Data configuration register mask */
#define CTRL0_DREM_MASK     0x04    /* 1 = Generate data ready event flag on pressure/altitude/temp change */
#define CTRL0_PDEFE_MASK    0x02    /* 1 = Raise event flag on new pressure data */
#define CTRL0_TDEFE_MASK    0x01    /* 1 = Raise event flag on new temperature data */

/* Control register mask values */
/* Masks for the control register */
#define CTRL1_ALT_MASK 0x80  /* Alt or Bar mode. - 0 - Barometer, 1-Altitude */
#define CTRL1_RAW_MASK 0x40  /* Raw data mode get raw data from adc, overrides alt/bar mask */
#define CTRL1_OS2_MASK 0x20  /* OS[0-2] sets oversample ratio. */
#define CTRL1_OS1_MASK 0x10
#define CTRL1_OS0_MASK 0x08  
#define CTRL1_RST_MASK 0x04  /* Software reset. 0 - device is disabled, 1-enabled */
#define CTRL1_OST_MASK 0x02  /* Initiate immediate measurement if enabled */
#define CTRL1_SBY_MASK 0x01  /* System Standby Mask, 0 - standby, 1 - active */

#define STANDBY_SBYB_0        0xFE
#define ACTIVE_MASK           0x01
#define RESET_MASK            0x04
#define ACTIVE_SBYB_OST       OST_MASK+SBYB_MASK
#define FULL_SCALE_STANDBY    0x00
#define CLEAR_OSR             0xC3
#define OSR_2                 0x08
#define OSR_4                 0x10
#define OSR_8                 0x18
#define OSR_16                0x20
#define OSR_32                0x28
#define OSR_64                0x30
#define OSR_128               0x38

#define DR_MASK               0x38
#define MPL_MODE_MASK             0x80
#define CLEAR_MPL_MODE_MASK       0x7F


/* Interrupt control register masks*/

#define FIFO_GATE_MASK  0x10
#define ST3_MASK        0x08
#define ST2_MASK        0x04
#define ST1_MASK        0x02
#define ST0_MASK        0x01
#define ST_MASK         0x0F
#define CLEAR_ST_MASK   0xF0
#define CLEAR_CTRLREG2  0x00


/* Interrupt control register masks*/
#define CTRL3_IPOL1_MASK 0x20 /* Interrupt polarity 1 = Active high, 0 = Active Low */
#define CTRL3_PPOD1_MASK 0x10 /* Push pull or open drain. 0 = Internal pull up, 1= open drain */
#define CTRL3_IPOL2_MASK 0x02 
#define CTRL3_PPOD2_MASK 0x01


#define CTRL4_INTEN_DRDY_MASK   0x80    /* 1 = Data ready interrupt enabled */
#define CTRL4_INTEN_FIFO_MASK   0x40    /* 1 = FIFO interrupt enabled */
#define CTRL4_INTEN_PW_MASK     0x20    /* 1 = Enable interrupt when pressure is within window */
#define CTRL4_INTEN_TW_MASK     0x10    /* 1 = Enable inteerupt when temperature is within window */
#define CTRL4_INTEN_PTH_MASK    0x08    /* 1 = Enable interrupts when pressure reaches threshold */
#define CTRL4_INTEN_TTH_MASK    0x04    /* 1 = Enable interrupts when temperature reaches threshold */
#define CTRL4_INTEN_PCHG_MASK   0x02    /* 1 = Enable interrupt when pressure changes */
#define CTRL4_INTEN_TCHG_MASK   0x01    /* 1 = Enable interrupt when temperature changes */

#define CTRL5_CFG_DRDY_MASK     0x80
#define CTRL5_CFG_FIFO_MASK     0x40
#define CTRL5_CFG_PW_MASK       0x20
#define CTRL5_CFG_TW_MASK       0x10
#define CTRL5_CFG_PTH_MASK      0x08
#define CTRL5_CFG_TTH_MASK      0x04
#define CTRL5_CFG_PCHG_MASK     0x02
#define CTRL5_CFG_TCHG_MASK     0x01

/*
 * **  8-Bit OUT Data Registers
 * */
#define OUT_P_MSB_REG        0x01
#define OUT_P_CSB_REG        0x02
#define OUT_P_LSB_REG        0x03
#define OUT_T_MSB_REG        0x04
#define OUT_T_LSB_REG        0x05
                             
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B

#define WHO_AM_I         0x0C

/* MPL31152A initialization functions,
 * can be initialized to provide barometric
 * pressure or altimeter functions
 */
bool is_valid_device(struct i2c_client * client);
void init(struct i2c_client * client, bool altmode);
bool is_data_ready_set(struct i2c_client * client);

void disable_drdy_interrupt(struct i2c_client * client);
void enable_drdy_interrupt(struct i2c_client * client);

uint8_t mpl_standby(struct i2c_client * client);
void mpl_activate(struct i2c_client * client);

uint32_t read_altbar(struct i2c_client * client);
uint32_t read_temp(struct i2c_client * client);


bool set_osr(struct i2c_client * client, uint8_t osr);
bool set_step_time(struct i2c_client * client, uint8_t step);


#endif
