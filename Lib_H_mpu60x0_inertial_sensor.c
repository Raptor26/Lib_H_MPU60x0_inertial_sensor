/** 
 * File:   %<%NAME%>%.%<%EXTENSION%>%
 * Author: %<%USER%>%
 *
 * Created on %<%DATE%>%, %<%TIME%>%
 */

//******************************************************************************
// Секция include: здесь подключается заголовочный файл к модулю
#include "Lib_H_mpu60x0_inertial_sensor.h"
//******************************************************************************


//******************************************************************************
//==============================================================================
// Глобальные переменные
//==============================================================================


//==============================================================================
// Локальные переменные
//==============================================================================
//******************************************************************************


//******************************************************************************
// Секция прототипов локальных функций
float MPU60x0_TwoBytesInFloat(uint8_t *pArr);
void MPU60x0_WriteDataInReg(mpu60x0_spi_t *spi,
                            uint8_t redAddr,
                            uint8_t data);
uint8_t* MPU60x0_SPI_ReadData(mpu60x0_spi_t *spi,
                              uint8_t addr,
                              uint8_t *rxArr,
                              uint16_t cnt);
void MPU60x0_SPI_Read_All(mpu60x0_spi_t *spi,
                          mpu60x0_data_t *data);
//******************************************************************************


//******************************************************************************
// Секция описания функций (сначала глобальных, потом локальных)
//

/**
 *  @brief  Функция выполняет конфигурирование датчика
 *  @param  *spi:   Структура, содержащая указатели на функции для работы с шиной SPI
 *  @param  *conf:  Структура, содержащая значени регистров датчика, которые 
 *                  необходимо отправить по шине SPI в инерциальный датчик
 *  @retval None
 *  @see    MPU60x0_WriteDataInReg()
 */
void MPU60x0_SPI_Config(mpu60x0_spi_t *spi,
                        mpu60x0_regs_t *conf)
{
    //  Reg 106
    //  Отключение модуля I2C (иначе SPI не будет работать))
    MPU60x0_WriteDataInReg(spi, __MPU60x0_SPI_WRITE_FLAG(MPU60x0_REG_USER_CTRL), MPU60x0_BIT_I2C_IF_DIS);

    //  Reg 107
    //  Сброс устройства
    MPU60x0_WriteDataInReg(spi, __MPU60x0_SPI_WRITE_FLAG(MPU60x0_REG_PWR_MGMT_1), MPU60x0_BIT_DEVICE_RESET);

    // Ждем в цикле пока не пройдет заданное количество времени
    size_t i = 0;
    for (i = 0; i < 5000; i++)
    {
        spi->delay_1us();
    }

    //  Reg 106
    //  Отключение модуля I2C (иначе SPI не будет работать))
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_USER_CTRL, MPU60x0_BIT_I2C_IF_DIS);

    //  Reg 107
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_PWR_MGMT_1, conf->pwr_managment_107);

    //  Reg 26
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_CONFIG, conf->dlpf_26);

    //  Reg 27
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_GYRO_CONFIG, conf->gyroConf_27);

    //  Reg 28
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_ACCEL_CONFIG, conf->accelConf_28);

    //  Reg 35
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_FIFO_EN, conf->fifo_En_35);

    //  Reg 55
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_INT_PIN_CFG, conf->int_pin_55);

    //  Reg 56
    MPU60x0_WriteDataInReg(spi, MPU60x0_REG_INT_ENABLE, conf->int_En_56);
}

/**
 *  @brief  Функция выполняет чтение данных гироскопа, акселерометра и температуры 
 *          самого датчика а затем нормирует эти показания
 *  @param  *spi:   Указатель на стурктуру, содержащую указатели на функции для 
 *                  работы с шиной SPI
 *  @param  *data:  Указатель на структуру, в которую необходимо записать 
 *                  преобразованные и нормированные показания датчиков
 *  @param  *lsb:   Указатель на стурктуру, содержащую масштабные коэффициенты 
 *                  для нормирования показаний инерциального датчика
 *  @retval None
 *  @see    MPU60x0_SPI_Read_All()
 *  @see    MPU60x0_Data_Convert()
 */
void MPU60x0_SPI_GetAllNormData(mpu60x0_spi_t *spi,
                                mpu60x0_data_t *data,
                                mpu60x0_lsb_t *lsb)
{
    MPU60x0_SPI_Read_All(spi, data);
    MPU60x0_Data_Convert(data->gyrArr, lsb->gyrLSB_RAD);
    MPU60x0_Data_Convert(data->accelArr, lsb->accelLSB_G);
    data->temper /= lsb->temperature_GRAD;
    data->temper += 36.53;
}

/**
 *  @brief  Функция выполняет чтение данных гироскопа, акселерометра и температуры 
 *          самого датчика
 *  @param  *spi:   Указатель на стурктуру, содержащую указатели на функции для 
 *                  работы с шиной SPI
 *  @param  *data:  Указатель на структуру данных, в которую необходимо занести 
 *                  "сырые" данные типа "float", считанные с датчика   
 *  @retval None
 *  @see    MPU60x0_SPI_ReadData()
 */
void MPU60x0_SPI_Read_All(mpu60x0_spi_t *spi, mpu60x0_data_t *data)
{
    uint8_t rxArr[14];

    // Читаем все данные датчика
    MPU60x0_SPI_ReadData(spi, MPU60x0_REG_ACCEL_XOUT_H, rxArr, 14);

    //  Выполняем преобразование считанных данных в 
    data->accelArr[MPU60x0_X] = MPU60x0_TwoBytesInFloat(&rxArr[0]);
    data->accelArr[MPU60x0_Y] = MPU60x0_TwoBytesInFloat(&rxArr[2]);
    data->accelArr[MPU60x0_Z] = MPU60x0_TwoBytesInFloat(&rxArr[4]);
    data->temper = MPU60x0_TwoBytesInFloat(&rxArr[6]);
    data->gyrArr[MPU60x0_X] = MPU60x0_TwoBytesInFloat(&rxArr[8]);
    data->gyrArr[MPU60x0_Y] = MPU60x0_TwoBytesInFloat(&rxArr[10]);
    data->gyrArr[MPU60x0_Z] = MPU60x0_TwoBytesInFloat(&rxArr[12]);
}

/**
 *  @brief  Функция выполняет чтение данных датчика из указаного регистра с учетом 
 *          автоинкремента адреса регистра внутри датчика 
 *  @param  *spi:   Указатель на стурктуру, содержащую указатели на функции для 
 *                  работы с шиной SPI
 *  @param  addr:   Адрес регистра устройства
 *  @param  *rxArr: Указатель на элемент массива, с которого начинается запись данных
 *  @param  cnt:    Количество байт которое необходимо прочитать из датчика
 *  @retval Указатель на элемент массива, следующий после того в который была произведена крайняя запись
 */
uint8_t* MPU60x0_SPI_ReadData(mpu60x0_spi_t *spi,
                              uint8_t addr,
                              uint8_t *rxArr,
                              uint16_t cnt)
{
    addr = __MPU60x0_SPI_READ_FLAG(addr);
    spi->sc_ON();
    spi->delay_1us();
    spi->transmit(&addr, 1);
    spi->receive(rxArr, cnt);
    spi->cs_OFF();
    spi->delay_1us();
    return rxArr++; //  Возвращаем указатель на следующий после крайней записи 
    //                  элемент массива
}

void MPU60x0_SPI_Read_Gyr(mpu60x0_spi_t *spi,
                          float *pGyr)
{
    uint8_t gyrArr[6];

    //  Выполняем чтение данных инерциального датчика
    MPU60x0_SPI_ReadData(spi,
                         __MPU60x0_SPI_READ_FLAG(MPU60x0_REG_GYRO_XOUT_H),
                         gyrArr,
                         6);

    //  Выполняем приведение 2 байт данных в одну переменную типа "float"
    *++pGyr = MPU60x0_TwoBytesInFloat(&gyrArr[0]);
    *++pGyr = MPU60x0_TwoBytesInFloat(&gyrArr[2]);
    *++pGyr = MPU60x0_TwoBytesInFloat(&gyrArr[4]);
}

void MPU60x0_SPI_Read_Accel(mpu60x0_spi_t *spi,
                            float *pAccel)
{
    uint8_t accelArr[6];

    // Выполняем чтение данных инерциального датчика
    MPU60x0_SPI_ReadData(spi, MPU60x0_REG_ACCEL_XOUT_H, accelArr, 6);

    //  Выполняем приведение 2 байт данных в одну переменную типа "float"
    *++pAccel = MPU60x0_TwoBytesInFloat(&accelArr[0]);
    *++pAccel = MPU60x0_TwoBytesInFloat(&accelArr[2]);
    *++pAccel = MPU60x0_TwoBytesInFloat(&accelArr[4]);
}

void MPU60x0_SPI_Read_Temp(mpu60x0_spi_t *spi,
                           float *pTemp)
{
    uint8_t tempArr[2];

    // Выполняем чтение данных инерциального датчика
    MPU60x0_SPI_ReadData(spi, MPU60x0_REG_TEMP_OUT_H, tempArr, 2);

    //  Выполняем приведение 2 байт данных в одну переменную типа "float"
    *pTemp = MPU60x0_TwoBytesInFloat(tempArr);
}

/**
 *  @brief  Функция выполняет чтение регистра датчика, содержащего масштабные коэффициенты гироскопа и акселерометра
 *  @param  *spi:   Указатель на стурктуру, содержащую указатели на функции для 
 *                  работы с шиной SPI
 *  @retval Структура, содержащая актуальные масштабные коэффициенты акселерометра и гироскопа
 */
mpu60x0_lsb_t MPU60x0_SPI_GyroAccel_LSB(mpu60x0_spi_t *spi)
{
    mpu60x0_lsb_t lsb;
    uint8_t tempLSB = 0;

    //  Чтение масштабного коэффициента гироскопа
    MPU60x0_SPI_ReadData(spi, MPU60x0_REG_GYRO_CONFIG, &tempLSB, 1);

    //  Определяем масштабный коэффициент гироскопа
    if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_FS_SEL_250)
    {
        lsb.gyrLSB_DEG = MPU60x0_GYRO_LSB_250;
        lsb.gyrLSB_RAD = lsb.gyrLSB_DEG * (180 / MPU60x0_PI);
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_FS_SEL_500)
    {
        lsb.gyrLSB_DEG = MPU60x0_GYRO_LSB_500;
        lsb.gyrLSB_RAD = lsb.gyrLSB_DEG * (180 / MPU60x0_PI);
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_FS_SEL_1000)
    {
        lsb.gyrLSB_DEG = MPU60x0_GYRO_LSB_1000;
        lsb.gyrLSB_RAD = lsb.gyrLSB_DEG * (180 / MPU60x0_PI);
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_FS_SEL_2000)
    {
        lsb.gyrLSB_DEG = MPU60x0_GYRO_LSB_2000;
        lsb.gyrLSB_RAD = lsb.gyrLSB_DEG * (180 / MPU60x0_PI);
    }

    //  Чтение масштабного коэффициента акселерометра
    MPU60x0_SPI_ReadData(spi, MPU60x0_REG_ACCEL_CONFIG, &tempLSB, 1);

    //  Определяем масштабный коэффициент акселерометра
    if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_AFS_SEL_2)
    {
        lsb.accelLSB_G = MPU60x0_ACCEL_LSB_2G;
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_AFS_SEL_4)
    {
        lsb.accelLSB_G = MPU60x0_ACCEL_LSB_4G;
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_AFS_SEL_8)
    {
        lsb.accelLSB_G = MPU60x0_ACCEL_LSB_8G;
    }
    else if ((tempLSB & MPU60x0_LSB_MASK) == MPU60x0_BIT_AFS_SEL_16)
    {
        lsb.accelLSB_G = MPU60x0_ACCEL_LSB_16G;
    }

    // Заполняем коэффициент для температуры (см. 4.18 In "MPU-6000/MPU-6050 Register Map and Descriptions")
    lsb.temperature_GRAD = 340.0f;
    return lsb;
}

/**
 *  @brief  Функция выполняет запись байта данных по указанному адресу
 *  @param  *spi:   Указатель на стурктуру, содержащую указатели на функции для 
 *                  работы с шиной SPI
 *  @param  redAddr:    Адрес регистра, в который будет произведена запись байта
 *  @param  data:   Байт данны, который будет записан по указанному адресу
 *  @retval None
 */
void MPU60x0_WriteDataInReg(mpu60x0_spi_t *spi, uint8_t redAddr, uint8_t data)
{
    uint8_t arr[2] = {__MPU60x0_SPI_WRITE_FLAG(redAddr),
        data};
    spi->sc_ON();
    spi->delay_1us();
    spi->transmit(arr, 2);
    spi->cs_OFF();
    spi->delay_1us();
}

void MPU60x0_AccelCalib(float *pArr, float calibMartix[][3])
{
    pArr++;
    float aXtemp = *pArr++, aYtemp = *pArr++, aZtemp = *pArr;
    pArr = pArr - 2;

    //  Нормироване оси "X"
    *pArr = calibMartix[3][0] + aXtemp * calibMartix[0][0] + aYtemp * calibMartix[1][0] + aZtemp * calibMartix[2][0];
    *pArr++ /= 500;

    //  Нормироване оси "Y"
    *pArr = calibMartix[3][1] + aXtemp * calibMartix[0][1] + aYtemp * calibMartix[1][1] + aZtemp * calibMartix[2][1];
    *pArr++ /= 500;
    
    //  Нормироване оси "Z"  
    *pArr = calibMartix[3][2] + aXtemp * calibMartix[0][2] + aYtemp * calibMartix[1][2] + aZtemp * calibMartix[2][2];
    *pArr /= 500;
}

/**
 *  @brief  Функция выполняет нормирование "сырых" показаний инерциального датчика
 *  @param  *pTemp: Указатель на нулевой элемент массива в котором содержатся 
 *          показания инерцальнго датчика
 *  @param  lsb:    Масштабный коэффициент на который будут делится "сырые" данные
 *  @retval None
 */
void MPU60x0_Data_Convert(float *pTemp, float lsb)
{
    //  !!! Проверить инкремент указателя
    *++pTemp /= lsb;
    *++pTemp /= lsb;
    *++pTemp /= lsb;
}

float MPU60x0_TwoBytesInFloat(uint8_t *pArr)
{
    int16_t temp = ((((int16_t) (*pArr++) << 8)) & 0xFF00);
    temp |= ((int16_t) (*pArr)) & 0x00F;
    return (float) temp;
}
//******************************************************************************


////////////////////////////////////////////////////////////////////////////////
// END OF FILE
////////////////////////////////////////////////////////////////////////////////