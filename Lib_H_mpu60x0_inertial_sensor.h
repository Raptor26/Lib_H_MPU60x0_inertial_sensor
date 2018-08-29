/**
 * File:   %<%NAME%>%.%<%EXTENSION%>%
 * Author: %<%USER%>%
 *
 * Created on %<%DATE%>%, %<%TIME%>%
 */

#ifndef LIB_H_MPU60X0_INERTIAL_SENSOR_H
#define	LIB_H_MPU60X0_INERTIAL_SENSOR_H

//******************************************************************************
//  Секция include (подключаем заголовочные файлы используемых модулей)
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
//******************************************************************************


//******************************************************************************
//  Секция определения констант
#define MPU60x0_PI                                  3.14159f
//  MPU60x0 Registers
#define MPU60x0_REG_WHO_AM_I                        0x75
#define MPU60x0_READ_FLAG                           0x80
#define MPU60x0_WRITE_FLAG                          0x7F

#define MPU60x0_REG_INT_STATUS                      0x3A
#define MPU60x0_REG_CONFIG                          0x1A
#define MPU60x0_REG_GYRO_CONFIG                     0x1B
#define MPU60x0_REG_ACCEL_CONFIG                    0x1C
#define MPU60x0_REG_FIFO_EN                         0x23
#define MPU60x0_REG_I2C_MST_CTRL                    0x24
#define MPU60x0_REG_INT_PIN_CFG                     0x37
#define MPU60x0_REG_INT_ENABLE                      0x38
#define MPU60x0_REG_INT_STATUS                      0x3A

#define MPU60x0_REG_ACCEL_XOUT_H                    0x3B
#define MPU60x0_REG_ACCEL_XOUT_L                    0x3C
#define MPU60x0_REG_ACCEL_YOUT_H                    0x3D
#define MPU60x0_REG_ACCEL_YOUT_L                    0x3E
#define MPU60x0_REG_ACCEL_ZOUT_H                    0x3F
#define MPU60x0_REG_ACCEL_ZOUT_L                    0x40

#define MPU60x0_REG_TEMP_OUT_H                      0x41
#define MPU60x0_REG_TEMP_OUT_L                      0x42

#define MPU60x0_REG_GYRO_XOUT_H                     0x43
#define MPU60x0_REG_GYRO_XOUT_L                     0x44
#define MPU60x0_REG_GYRO_YOUT_H                     0x45
#define MPU60x0_REG_GYRO_YOUT_L                     0x46
#define MPU60x0_REG_GYRO_ZOUT_H                     0x47
#define MPU60x0_REG_GYRO_ZOUT_L                     0x48

#define MPU60x0_REG_USER_CTRL                       0x6A
#define MPU60x0_REG_PWR_MGMT_1                      0x6B
#define MPU60x0_REG_PWR_MGMT_2                      0x6C

//------------------------------------------------------------------------------
//  Accel LSB
#define MPU60x0_ACCEL_LSB_2G                        16384
#define MPU60x0_ACCEL_LSB_4G                        8192
#define MPU60x0_ACCEL_LSB_8G                        4096
#define MPU60x0_ACCEL_LSB_16G                       2048

//  Gyro LSB
#define MPU60x0_GYRO_LSB_250                        ((float)131.0f)
#define MPU60x0_GYRO_LSB_500                        ((float)65.5f)
#define MPU60x0_GYRO_LSB_1000                       ((float)32.8f)
#define MPU60x0_GYRO_LSB_2000                       ((float)16.4f)

#define MPU60x0_LSB_MASK                            0b00011000
//------------------------------------------------------------------------------
//  Configuration bits MPU600x
//  Register 26 – Configuration CONFIG (SYNC_SET)
#define MPU60x0_BIT_EXT_SYNC_SET_INPUT_DISABLED     0x00
#define MPU60x0_BIT_EXT_SYNC_SET_TEMP_OUT_L         0x08
#define MPU60x0_BIT_EXT_SYNC_SET_GYRO_XOUT_L        0x10
#define MPU60x0_BIT_EXT_SYNC_SET_GYRO_YOUT_L        0x18
#define MPU60x0_BIT_EXT_SYNC_SET_GYRO_ZOUT_L        0x20
#define MPU60x0_BIT_EXT_SYNC_SET_ACCEL_XOUT_L       0x28
#define MPU60x0_BIT_EXT_SYNC_SET_ACCEL_YOUT_L       0x30
#define MPU60x0_BIT_EXT_SYNC_SET_ACCEL_ZOUT_L       0x38
//  Register 26 – Configuration CONFIG (DLPF_CFG)
#define MPU60x0_BIT_DLPF_CFG_256                    0x00
#define MPU60x0_BIT_DLPF_CFG_188                    0x01
#define MPU60x0_BIT_DLPF_CFG_98                     0x02
#define MPU60x0_BIT_DLPF_CFG_42                     0x03
#define MPU60x0_BIT_DLPF_CFG_20                     0x04
#define MPU60x0_BIT_DLPF_CFG_10                     0x05
#define MPU60x0_BIT_DLPF_CFG_5                      0x06

//  Register 27 – Gyroscope Configuration GYRO_CONFIG (FS_SEL)
#define MPU60x0_BIT_FS_SEL_250                      0b00000000
#define MPU60x0_BIT_FS_SEL_500                      0b00001000
#define MPU60x0_BIT_FS_SEL_1000                     0b00010000
#define MPU60x0_BIT_FS_SEL_2000                     0b00011000

//  Register 28 – Accelerometer Configuration ACCEL_CONFIG (AFS_SEL)
#define MPU60x0_BIT_AFS_SEL_2                       0x00
#define MPU60x0_BIT_AFS_SEL_4                       0b00001000
#define MPU60x0_BIT_AFS_SEL_8                       0b00010000
#define MPU60x0_BIT_AFS_SEL_16                      0b00011000

//  Register 35 - FIFO Enable
//  (Конфигурирование данными именованными константами возможно через операцию "|")
#define MPU60x0_BIT_FIFO_EN_TEMP_FIFO_EN            0b10000000
#define MPU60x0_BIT_FIFO_EN_XG_FIFO_EN              0b01000000
#define MPU60x0_BIT_FIFO_EN_YG_FIFO_EN              0b00100000
#define MPU60x0_BIT_FIFO_EN_ZG_FIFO_EN              0b00010000
#define MPU60x0_BIT_FIFO_EN_ACCEL_FIFO_EN           0b00001000
#define MPU60x0_BIT_FIFO_EN_SLV2_FIFO_EN            0b00000100
#define MPU60x0_BIT_FIFO_EN_SLV1_FIFO_EN            0b00000010
#define MPU60x0_BIT_FIFO_EN_SLV0_FIFO_EN            0b00000001

//  Register 55 - INT pin / Bypass Enable Configuration
//  (Конфигурирование данными именованными константами возможно через операцию "|")
#define MPU60x0_BIT_INT_PIN_INT_LEVEL               0b10000000
#define MPU60x0_BIT_INT_PIN_INT_OPEN                0b01000000
#define MPU60x0_BIT_INT_PIN_LATCH_INT_EN            0b00100000
#define MPU60x0_BIT_INT_PIN_INT_RD_CLEAR            0b00010000
#define MPU60x0_BIT_INT_PIN_FSYNC_INT_LEVEL         0b00001000
#define MPU60x0_BIT_INT_PIN_FSYNC_INT_EN            0b00000100
#define MPU60x0_BIT_INT_PIN_I2C_BYPASS_EN           0b00000010

//  Register 56 - Interrupt Enable
//  (Конфигурирование данными именованными константами возможно через операцию "|")
#define MPU60x0_BIT_INT_EN_FIFO_OFLOW_EN            0b00010000
#define MPU60x0_BIT_INT_EN_I2C_MST_INT_EN           0b00001000
#define MPU60x0_BIT_INT_EN_DATA_RDY_EN              0b00000001

//  Register 106 – User Control USER_CTRL
#define MPU60x0_BIT_I2C_MST_EN                      0x00
#define MPU60x0_BIT_I2C_IF_DIS                      0b00010000

//  Register 107 – Power Management 1 PWR_MGMT_1
#define MPU60x0_BIT_DEVICE_RESET                    0b10000000
#define MPU60x0_BIT_CLKSEL_8MHz_OSCILLATOR          0b00000000
#define MPU60x0_BIT_CLKSEL_PLL_GYRO_X               0b00000001
#define MPU60x0_BIT_CLKSEL_PLL_GYRO_Y               0b00000010
#define MPU60x0_BIT_CLKSEL_PLL_GYRO_Z               0b00000011
#define MPU60x0_BIT_CLKSEL_PLL_EXTERNAL_32_768      0b00000100
#define MPU60x0_BIT_CLKSEL_PLL_EXTERNAL_19_2        0b00000101
#define MPU60x0_BIT_CLKSEL_PLL_GENERATOR_RESET      0b00000111
//******************************************************************************


//******************************************************************************
// Секция определения типов

/**
 * @brief   Перечисляемый тип содержит нумерацию осей датчика для более удобной работы
 *          с массивами, в которых содержатся показания датчика;
 */
enum MPU60x0_ARR {
    MPU60x0_X = 1,
    MPU60x0_Y = 2,
    MPU60x0_Z = 3,
}; /**
    *  Перечисляемый тип содержит нумерацию осей датчика для более удобной работы
    *  с массивами, в которых содержатся показания датчика;
    */

/**
 * @brief   Стуктура содержит указатели на функции, обеспечивающие
 *          работу на шине SPI. Указатели должны быть проинициализированы в
 *          вызывающей функции;
 */

typedef struct {
    /**
     * Указатель на функцию передачи данных в 8-ми битном режиме;
     */
    void (*Transmit_8bits) (uint8_t *pTxData, uint16_t cnt);

    /**
     * Указатель на функцию приема данных в 8-ми битном режиме;
     */
    void (*Receive_8bits) (uint8_t *pRxData, uint16_t cnt);

    /**
     * Указатель на функцию, которая устанавливает низкий логический уровень
     * на соответствующем выводе датчика MPU6000;
     */
    void (*CS_On) (void);

    /**
     * Указатель на функцию, которая устанавливает высокий логический уровень
     * на соответствующем выводе датчика MPU6000;
     */
    void (*CS_Off) (void);

    /**
     * Указатель на функцию, которая обеспечивает программную задержку в 1 мкс;
     */
    void (*Delay_1_us) (void);

} mpu60x0_spi_s; /**
                 * Стуктура содержит указатели на функции, обеспечивающие
                 * работу на шине SPI. Указатели должны быть проинициализированы в
                 * вызывающей функции;
                 */

/**
 * @brief   Структура содержит массив данных гироскопа, массив данных
 *          акселерометра и температуру датчика;
 */
typedef struct {
    /**
     * Массив[4], содержащий измерения угловой скорости:
     * [0] - нуль;
     * [1] - измерение угловой скорости по оси "X";
     * [2] - измерение угловой скорости по оси "Y";
     * [3] - измерение угловой скорости по оси "Z";
     */
    float gyrArr[4];

    /**
     * Массив[4], содержащий измерения линейных ускорений:
     * [0] - нуль;
     * [1] - измерение линейного ускорения по оси "X";
     * [2] - измерение линейного ускорения по оси "Y";
     * [3] - измерение линейного ускорения по оси "Z";
     */
    float accelArr[4];

    /**
     * Температура датчика MPU6000
     */
    float temper;
} mpu60x0_data_s; /**
                  * Структура содержит массив данных гироскопа, массив данных
                  * акселерометра и температуру датчика;
                  */

/**
 * @brief   Стуктура содержит масштабные коэффициенты для перевода
 *          "сырых" показаний датчика в определенные величины;
 */
typedef struct {
    /**
     * Масштабный коэффициент для приведения сырых измерений угловой скорости
     * к град/сек;
     */
    float gyrLSB_DEG;

    /**
     * Масштабный коэффициент для приведения сырых измерений угловой скорости
     * к рад/сек;
     */
    float gyrLSB_RAD;

    /**
     * Масштабный коэффициент для приведения сырых измерений линейных ускорений
     * к гравитационной единице;
     */
    float accelLSB_G;

    /**
     * Масштабный коэффициент для приведения сырых показаний температуры
     * датчика к градусам;
     */
    float temperature_GRAD;
} mpu60x0_lsb_s; /**
                 * Стуктура содержит масштабные коэффициенты для перевода
                 * "сырых" показаний датчика в определенные величины;
                 */

/**
 * @brief   Структура содержит значения регистров для
 *          последующей записи этих значений в аппаратные регистры датчика MPU6000;
 */
typedef struct {
    /**
     * Digital low pass filter (REG_26);
     */
    uint8_t dlpf_26;

    /**
     * Gyroscope Configuration (REG_27);
     */
    uint8_t gyroConf_27;

    /**
     * Accel Configuration (REG_28);
     */
    uint8_t accelConf_28;

    /**
     * FIFO Enable (REG_35);
     */
    uint8_t fifo_En_35;

    /**
     * Interrupt pin / Bypass Enable Configuration (REG_55);
     */
    uint8_t int_pin_55;

    /**
     * Interrupt Enable (REG_56);
     */
    uint8_t int_En_56;

    /**
     * Interrupt status (REG_58);
     */
    uint8_t int_status_58;

    /**
     * Power Managment 1 (REG_107);
     */
    uint8_t pwr_managment_107;
} mpu60x0_regs_s; /**
                  * Структура содержит значения регистров для
                  * последующей записи этих значений в аппаратные регистры
                  * датчика MPU6000;
                  */
//******************************************************************************


//******************************************************************************
// Секция определения глобальных переменных
//******************************************************************************


//******************************************************************************
// Секция прототипов глобальных функций
extern mpu60x0_lsb_s MPU60x0_SPI_Config(
        mpu60x0_spi_s *spi,
        mpu60x0_regs_s *conf);

extern mpu60x0_lsb_s MPU60x0_SPI_GyroAccel_LSB(
        mpu60x0_spi_s *spi);

extern void MPU60x0_SPI_Read_Gyr(
        mpu60x0_spi_s *spi,
        float *pGyr);

extern void MPU60x0_SPI_Read_Accel(
        mpu60x0_spi_s *spi,
        float *pAccel);

extern void MPU60x0_Data_Convert(
        float *pTemp,
        float lsb);

extern void MPU60x0_SPI_GetAllNormData(
        mpu60x0_spi_s *spi,
        mpu60x0_data_s *data,
        mpu60x0_lsb_s *lsb);

extern void MPU60x0_AccelCalib(float *pArr,
        float calibMartix[][3]);
//******************************************************************************


//******************************************************************************
// Секция определения макросов
#define __MPU60x0_SPI_SET_READ_FLAG(__x__)      ((__x__) | (0x80))
#define __MPU60x0_SPI_SET_WRITE_FLAG(__x__)     ((__x__) & (0x7F))
//******************************************************************************

#endif

////////////////////////////////////////////////////////////////////////////////
// END OF FILE
////////////////////////////////////////////////////////////////////////////////

