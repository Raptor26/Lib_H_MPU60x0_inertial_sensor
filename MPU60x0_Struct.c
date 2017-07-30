#include "MPU60x0_Struct.h"

//******************************************************************************
// ����������

//******************************************************************************
// ������� �� ������ ����� ��������� I2C
#ifdef I2C_EN

void MPU60x0_I2C_Config1(MPU60x0TypeDef *MPU)
{
    //--------------------------------------------------------------------------
    // ���������������� 26 �������� MPU60x0 (CONFIG)
    I2C_Start();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_CONFIG); //               ����� ��������
    I2C_Write(0b00000000); //                   ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 27 �������� MPU60x0 (GYRO_CONFIG)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_GYRO_CONFIG); //          ����� ��������
    I2C_Write(MPU_BIT_FS_SEL_500); //           ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 28 �������� MPU60x0 (ACCEL_CONFIG)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_ACCEL_CONFIG); //         ����� ��������
    I2C_Write(MPU_BIT_AFS_SEL_2); //            ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 35 �������� MPU60x0 (FIFO_EN)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(35); //                           ����� ��������
    I2C_Write(0b00000000); //                   ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 55 �������� MPU60x0 (INT_PIN_CFG)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(55); //                           ����� ��������
    I2C_Write(0b01101000); //                   ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 56 �������� MPU60x0 (INT_ENABLE)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(56); //                           ����� ��������
    I2C_Write(0b00010000); //                   ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // ���������������� 107 �������� MPU60x0 (PWR_MGMT_1)
    I2C_ReStart();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(107); //                          ����� ��������
    I2C_Write(0b00000001); //                   ������ ��� ������ � �������
    I2C_Stop();
    //--------------------------------------------------------------------------
} // void MPU60x0_I2C_Config1 (void)

void MPU60x0_I2C_ReadData(MPU60x0TypeDef *MPU)
{
    //==========================================================================
    // ����������
    int16_t i_ACCEL_XOUT_H = 0, i_ACCEL_XOUT_L = 0, i_ACCEL_YOUT_H = 0, i_ACCEL_YOUT_L = 0, i_ACCEL_ZOUT_H = 0,
            i_ACCEL_ZOUT_L = 0, i_TEMP_OUT_H = 0, i_TEMP_OUT_L = 0, i_GYRO_XOUT_H = 0, i_GYRO_XOUT_L = 0,
            i_GYRO_YOUT_H = 0, i_GYRO_YOUT_L = 0, i_GYRO_ZOUT_H = 0, i_GYRO_ZOUT_L = 0;

    int16_t i_AccelX = 0, i_AccelY = 0, i_AccelZ = 0;
    int16_t i_GyroX = 0, i_GyroY = 0, i_GyroZ = 0;

    int16_t i_TempValue = 0;
    int16_t MPU60x0_Flags = 0;

    I2C_Start();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_INT_STATUS);
    I2C_ReStart();
    I2C_Write(MPU->devAddr | MPU60x0_I2C_MASK_READ);


    //==========================================================================
    // ������ ������
    MPU60x0_Flags = I2C_Read(I2C_ACK); //   ������ ������� MPU60x0 � �������

    //    I2C_Master_ACK();
    i_ACCEL_XOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_ACCEL_XOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_ACCEL_YOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_ACCEL_YOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_ACCEL_ZOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_ACCEL_ZOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_TEMP_OUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_TEMP_OUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_XOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_XOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_YOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_YOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_ZOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_ZOUT_L = I2C_Read(I2C_NoACK);

    // ���������� ������ ������
    //    I2C_Master_NoACK();
    I2C_Stop();


    //==========================================================================
    // ������� ������ ������� � ����������
    //--------------------------------------------------------------------------
    // Accel axis "X" 
    i_AccelX = (i_ACCEL_XOUT_H << 8) & 0xFF00;
    i_AccelX |= (i_ACCEL_XOUT_L & 0x00FF);
    //        i_AccelX = i_ACCEL_XOUT_H << 8;
    //        i_AccelX += i_ACCEL_XOUT_L;

    // Accel axis "Y"
    i_AccelY = (i_ACCEL_YOUT_H << 8) & 0xFF00;
    i_AccelY |= (i_ACCEL_YOUT_L & 0x00FF);

    // Accel axis "Z"
    i_AccelZ = (i_ACCEL_ZOUT_H << 8) & 0xFF00;
    i_AccelZ |= (i_ACCEL_ZOUT_L & 0x00FF);


    //--------------------------------------------------------------------------
    // Gyro axis "X"
    i_GyroX = (i_GYRO_XOUT_H << 8) & 0xFF00;
    i_GyroX |= (i_GYRO_XOUT_L & 0x00FF);

    // Gyro axis "Y"
    i_GyroY = (i_GYRO_YOUT_H << 8) & 0xFF00;
    i_GyroY |= (i_GYRO_YOUT_L & 0x00FF);

    // Gyro axis "Z"
    i_GyroZ = (i_GYRO_ZOUT_H << 8) & 0xFF00;
    i_GyroZ |= (i_GYRO_ZOUT_L & 0x00FF);


    //--------------------------------------------------------------------------
    i_TempValue = (i_TEMP_OUT_H << 8) & 0xFF00;
    i_TempValue |= (i_TEMP_OUT_L & 0x00FF);


    //--------------------------------------------------------------------------
    // ������� ��������� ��������� � ������������� �� int � float ("int" to "float")
    MPU->aX = ((float) i_AccelX) / MPU->aLSB_G;
    MPU->aY = ((float) i_AccelY) / MPU->aLSB_G;
    MPU->aZ = ((float) i_AccelZ) / MPU->aLSB_G;
    MPU->gX = ((float) i_GyroX) / MPU->gLSB_Rad;
    MPU->gY = ((float) i_GyroY) / MPU->gLSB_Rad;
    MPU->gZ = ((float) i_GyroZ) / MPU->gLSB_Rad;
    MPU->temperature = ((float) i_TempValue);
    MPU->temperature = MPU->temperature / 340 + 36.53;

} // AHRS_Sens MPU60x0_I2C_ReadData(void)

void MPU60x0_I2C_ReadGyro(MPU60x0TypeDef *MPU)
{
    int16_t i_GYRO_XOUT_H = 0, i_GYRO_XOUT_L = 0,
            i_GYRO_YOUT_H = 0, i_GYRO_YOUT_L = 0, i_GYRO_ZOUT_H = 0, i_GYRO_ZOUT_L = 0;
    int16_t i_GyroX = 0, i_GyroY = 0, i_GyroZ = 0;

    I2C_Start();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_GYRO_XOUT_H);
    I2C_ReStart();
    I2C_Write(MPU->devAddr | MPU60x0_I2C_MASK_READ);

    // ������ ������ ���������
    //    I2C_Master_ACK();
    i_GYRO_XOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_XOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_YOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_YOUT_L = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_ZOUT_H = I2C_Read(I2C_ACK);

    //    I2C_Master_ACK();
    i_GYRO_ZOUT_L = I2C_Read(I2C_NoACK);

    // ���������� ������ ������
    //    I2C_Master_NoACK();
    I2C_Stop();

    // ������� ������ ������� � ����������
    // Gyro axis "X"
    i_GyroX = (i_GYRO_XOUT_H << 8) & 0xFF00;
    i_GyroX |= (i_GYRO_XOUT_L & 0x00FF);

    // Gyro axis "Y"
    i_GyroY = (i_GYRO_YOUT_H << 8) & 0xFF00;
    i_GyroY |= (i_GYRO_YOUT_L & 0x00FF);

    // Gyro axis "Z"
    i_GyroZ = (i_GYRO_ZOUT_H << 8) & 0xFF00;
    i_GyroZ |= (i_GYRO_ZOUT_L & 0x00FF);

    // ������� ��������� ��������� � ������������� �� int � float ("int" to "float")
    MPU->gX = ((float) i_GyroX) / MPU->gLSB_Rad;
    MPU->gY = ((float) i_GyroY) / MPU->gLSB_Rad;
    MPU->gZ = ((float) i_GyroZ) / MPU->gLSB_Rad;
} // void MPU60x0_I2C_ReadGyro(ahrs_sens *MPU)

void MPU60x0_I2C_Read_LSB(MPU60x0TypeDef *MPU)
{
    int16_t i_GyroConfigValue = 0;
    int16_t i_AccelConfigValue = 0;

    // ������ ������ �� �������� �������
    I2C_Start();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_GYRO_CONFIG);
    I2C_ReStart();
    I2C_Write(MPU->devAddr | MPU60x0_I2C_MASK_READ);
    i_GyroConfigValue = I2C_Read(I2C_NoACK);
    I2C_Stop();

    // ��������� ����� � �������� �������
    i_GyroConfigValue = i_GyroConfigValue >> 3;
    i_GyroConfigValue &= 0b0000000000000011;

    // ����������� ����������� ������������ ���������
    if (i_GyroConfigValue == 0)
    {
        MPU->gLSB_Rad = 7505.747116f;
        MPU->gLSB_Deg = 131.0f;
    }

    if (i_GyroConfigValue == 1)
    {
        MPU->gLSB_Rad = 3752.8735581f;
        MPU->gLSB_Deg = 65.5f;
    }

    if (i_GyroConfigValue == 2)
    {
        MPU->gLSB_Rad = 1879.30156f;
        MPU->gLSB_Deg = 32.8f;
    }

    if (i_GyroConfigValue == 3)
    {
        MPU->gLSB_Rad = 936.65078f;
        MPU->gLSB_Deg = 16.4f;
    }

    // ������ ������ �� �������� �������������
    I2C_Start();
    I2C_Write(MPU->devAddr & MPU60x0_I2C_MASK_WRITE);
    I2C_Write(MPU_REG_ACCEL_CONFIG);
    I2C_ReStart();
    I2C_Write(MPU->devAddr | MPU60x0_I2C_MASK_READ);
    i_AccelConfigValue = I2C_Read(I2C_NoACK);
    I2C_Stop();

    // ��������� ����� � �������� �������
    i_AccelConfigValue = i_AccelConfigValue >> 3;
    i_AccelConfigValue &= 0b0000000000000011;

    // ����������� ����������� ������������ �������������
    if (i_AccelConfigValue == 0)
    {
        MPU->aLSB_G = 16384.0f;
    }

    if (i_AccelConfigValue == 1)
    {
        MPU->aLSB_G = 8192.0f;
    }

    if (i_AccelConfigValue == 2)
    {
        MPU->aLSB_G = 4096.0f;
    }

    if (i_AccelConfigValue == 3)
    {
        MPU->aLSB_G = 2048.0f;
    }
} // AHRS_Sens MPU60x0_I2C_ReadGyroAccelConf(void)
#endif

void MPU60x0_CovertToNED(MPU60x0TypeDef *MPU)
{
    float gX = MPU->gX, gY = MPU->gY, gZ = MPU->gZ;
    float aX = MPU->aX, aY = MPU->aY, aZ = MPU->aZ;

    // Convert to NED Gyro
    MPU->gX = gY;
    MPU->gY = gX;
    MPU->gZ = -gZ;

    // Convert to NED Accel
    MPU->aX = -aY;
    MPU->aY = -aX;
    MPU->aZ = aZ;
} // AHRS_Sens MPU60x0_CovertToNED (AHRS_Sens MPU)