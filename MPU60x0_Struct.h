/* 
 * File:   MPU60x0_Struct.h
 * Author: Isaev
 *
 * Created on 23 июня 2017 г., 22:48
 */

#ifndef MPU60X0_STRUCT_H
#define	MPU60X0_STRUCT_H

//******************************************************************************
// Секция include (подключаем заголовочные файлы используемых модулей)
#include <stdint.h>
//******************************************************************************


//******************************************************************************
// Секция определения констант
#define I2C_EN
#define dsPIC33E_I2C_Raptor


//==============================================================================
// Если работаем по "I2C"
//------------------------------------------------------------------------------
#ifdef I2C_EN
#include "I2C.h"

// Раскоментировать строку ниже если "AD0 = 1"
#define MPU60x0_AD0_HIGHT 


//------------------------------------------------------------------------------
// Константы для работы по "I2C"
#ifdef MPU60x0_AD0_HIGHT
#define MPU60x0_ADDRESS_WRITE                   0b11010010
#define MPU60x0_ADDRESS_READ                    0b11010011

//------------------------------------------------------------------------------
// Маски для выставки бита чтения/записи регистра адреса устройства
#define MPU60x0_I2C_MASK_WRITE                  0b11111110
#define MPU60x0_I2C_MASK_READ                   0b00000001
//------------------------------------------------------------------------------
#endif // #ifdef MPU60x0_AD0_HIGHT


#ifndef MPU60x0_AD0_HIGHT
#define devAdd                   0b11010000
#define MPU60x0_ADDRESS_READ                    0b11010001
#endif
#endif


//------------------------------------------------------------------------------
#ifdef dsPIC33E_I2C_Raptor
// Переобъявление названий функций (PIC24E/dsPIC33E)
#define I2C_Start()                             I2C2_Start()
#define I2C_ReStart()                           I2C2_Restart()
#define I2C_Stop()                              I2C2_Stop()
#define I2C_Write(Write_Data)                   I2C2_Write_Data(Write_Data)
#define I2C_Read(Read_Data)                     I2C2_Read_Data(Read_Data)
#define I2C_Master_ACK()                        I2C2_Master_ACK()
#define I2C_Master_NoACK()                      I2C2_Master_NoACK()
#endif


//==============================================================================
// Если работаем по SPI
#ifndef I2C_EN
#define SPI_EN
#include "SPI.h"
#endif


//==============================================================================
// MPU60x0 Registers
#define MPU_REG_WHO_AM_I                        0x75
#define READ_FLAG                               0x80

#define MPU_REG_INT_STATUS                      0x3A //     Адрес начала чтения показаний инерциального
//                                                          датчика (Регистр прерываний MPU60x0)
#define MPU_REG_CONFIG                          0x1A
#define MPU_REG_GYRO_CONFIG                     0x1B
#define MPU_REG_ACCEL_CONFIG                    0x1C
#define MPU_REG_FIFO_EN                         0x23
#define MPU_REG_I2C_MST_CTRL                    0x24
#define MPU_REG_INT_PIN_CFG                     0x37
#define MPU_REG_INT_ENABLE                      0x38
#define MPU_REG_INT_STATUS                      0x3A

#define MPU_REG_ACCEL_XOUT_H                    0x3B
#define MPU_REG_ACCEL_XOUT_L                    0x3C
#define MPU_REG_ACCEL_YOUT_H                    0x3D
#define MPU_REG_ACCEL_YOUT_L                    0x3E
#define MPU_REG_ACCEL_ZOUT_H                    0x3F
#define MPU_REG_ACCEL_ZOUT_L                    0x40

#define MPU_REG_TEMP_OUT_H                      0x41
#define MPU_REG_TEMP_OUT_L                      0x42

#define MPU_REG_GYRO_XOUT_H                     0x43
#define MPU_REG_GYRO_XOUT_L                     0x44      
#define MPU_REG_GYRO_YOUT_H                     0x45
#define MPU_REG_GYRO_YOUT_L                     0x46
#define MPU_REG_GYRO_ZOUT_H                     0x47
#define MPU_REG_GYRO_ZOUT_L                     0x48

#define MPU_REG_USER_CTRL                       0x6A
#define MPU_REG_PWR_MGMT_1                      0x6B
#define MPU_REG_PWR_MGMT_2                      0x6C


//------------------------------------------------------------------------------
// Configuration bits MPU600x
//Register 26 – Configuration CONFIG (SYNC_SET)
#define MPU_BIT_EXT_SYNC_SET_INPUT_DISABLED     0x00
#define MPU_BIT_EXT_SYNC_SET_TEMP_OUT_L         0x08
#define MPU_BIT_EXT_SYNC_SET_GYRO_XOUT_L        0x10
#define MPU_BIT_EXT_SYNC_SET_GYRO_YOUT_L        0x18
#define MPU_BIT_EXT_SYNC_SET_GYRO_ZOUT_L        0x20
#define MPU_BIT_EXT_SYNC_SET_ACCEL_XOUT_L       0x28
#define MPU_BIT_EXT_SYNC_SET_ACCEL_YOUT_L       0x30
#define MPU_BIT_EXT_SYNC_SET_ACCEL_ZOUT_L       0x38

//Register 26 – Configuration CONFIG (DLPF_CFG)
#define MPU_BIT_DLPF_CFG_256                    0x00
#define MPU_BIT_DLPF_CFG_188                    0x01
#define MPU_BIT_DLPF_CFG_98                     0x02
#define MPU_BIT_DLPF_CFG_42                     0x03
#define MPU_BIT_DLPF_CFG_20                     0x04
#define MPU_BIT_DLPF_CFG_10                     0x05
#define MPU_BIT_DLPF_CFG_5                      0x06

// Register 27 – Gyroscope Configuration GYRO_CONFIG (FS_SEL)
#define MPU_BIT_FS_SEL_250                      0x00
#define MPU_BIT_FS_SEL_500                      0b00001000
#define MPU_BIT_FS_SEL_1000                     0b00010000
#define MPU_BIT_FS_SEL_2000                     0b00011000

// Register 28 – Accelerometer Configuration ACCEL_CONFIG (AFS_SEL)
#define MPU_BIT_AFS_SEL_2                       0x00
#define MPU_BIT_AFS_SEL_4                       0b00001000
#define MPU_BIT_AFS_SEL_8                       0b00010000
#define MPU_BIT_AFS_SEL_16                      0b00011000

// Register 106 – User Control USER_CTRL
#define MPU_BIT_I2C_MST_EN                      0x00
#define MPU_BIT_I2C_IF_DIS                      0b00010000

// Register 107 – Power Management 1 PWR_MGMT_1
#define MPU_BIT_DEVICE_RESET                    0b10000000
#define MPU_BIT_CLKSEL_8MHz_OSCILLATOR          0b00000000
#define MPU_BIT_CLKSEL_PLL_GYRO_X               0b00000001
#define MPU_BIT_CLKSEL_PLL_GYRO_Y               0b00000010
#define MPU_BIT_CLKSEL_PLL_GYRO_Z               0b00000011
//******************************************************************************


//******************************************************************************
// Секция определения типов
//==============================================================================

typedef struct {
    uint8_t devAddr;
    
    //--------------------------------------------------------------------------
    // Гироскоп
    float gX;
    float gY;
    float gZ;
    float gLSB_Rad;
    float gLSB_Deg;
    //--------------------------------------------------------------------------
    
    
    //--------------------------------------------------------------------------
    // Акселерометр
    float aX;
    float aY;
    float aZ;
    float aLSB_G;
    //--------------------------------------------------------------------------
    
    
    //--------------------------------------------------------------------------
    // Температура
    float temperature;
    //--------------------------------------------------------------------------
} MPU60x0TypeDef;
//==============================================================================
//******************************************************************************


//******************************************************************************
// Секция определения глобальных переменных
//******************************************************************************


//******************************************************************************
// Секция прототипов глобальных функций
extern void MPU60x0_I2C_Config1(MPU60x0TypeDef *MPU);
extern void MPU60x0_I2C_ReadData(MPU60x0TypeDef *MPU);
extern void MPU60x0_I2C_ReadGyro(MPU60x0TypeDef *MPU);
extern void MPU60x0_I2C_Read_LSB(MPU60x0TypeDef *MPU);
extern void MPU60x0_CovertToNED(MPU60x0TypeDef *MPU);
//******************************************************************************


//******************************************************************************
// Секция определения макросов
#define MPU60x0_I2C_ADDR_W(x)   (x & 0xFE) //           Clear R/W bit of I2C addr
#define MPU60x0_I2C_ADDR_R(x)   (x | 0x01) //           Set R/W bit of I2C addr
//******************************************************************************


////////////////////////////////////////////////////////////////////////////////
// END OF FILE
////////////////////////////////////////////////////////////////////////////////

#endif	/* MPU60X0_STRUCT_H */

