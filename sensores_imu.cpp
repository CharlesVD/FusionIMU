#include "sensores_imu.h"
#include <stdio.h>

Sensores_IMU::Sensores_IMU()
{
    //gyro
    direccion_gyro = wiringPiI2CSetup(L3GD20_ADDRESS_GYRO);
    gyro_X=0;
    gyro_Y=0;
    gyro_Z=0;

    u_int8_t id= wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_WHO_AM_I);
    if(id == 0xD4)
    {
        //printf("L3GD20_ID\n");
    }
    else
    {
        if(id==0xD7)
        {
            //printf("L3GD20H_ID\n");
        }
        else
        {
            //printf("%d\n",id);
        }
    }
    // modo normal
    wiringPiI2CWriteReg8(direccion_gyro,GYRO_REGISTER_CTRL_REG1, 0x0F);

    // rango
    wiringPiI2CWriteReg8(direccion_gyro,GYRO_REGISTER_CTRL_REG4, 0x20);

    // pasa altos
    wiringPiI2CWriteReg8(direccion_gyro,GYRO_REGISTER_CTRL_REG5, 0b00);
    //acel

    direccion_acel = wiringPiI2CSetup(LSM303_ADDRESS_ACCEL);
    acel_X = 0;
    acel_Y = 0;
    acel_Z = 0;

    // frecuencia 0x27 10hz  o 0x57 para 100hz o 0b01100111 200hz
    wiringPiI2CWriteReg8(direccion_acel,LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

    // actualizaciòn continua
    //wiringPiI2CWriteReg8(direccion_acel,LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x00);
    // actualizaciòn alta resolucion
    wiringPiI2CWriteReg8(direccion_acel,LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x08);

    //mag

    direccion_mag = wiringPiI2CSetup(LSM303_ADDRESS_MAG);
    mag_X=0;
    mag_Y=0;
    mag_Z=0;

    // Conversión continua(0x00)
    wiringPiI2CWriteReg8(direccion_mag,LSM303_REGISTER_MAG_MR_REG_M, 0x00);

    // Velocidad de salida 30hz
    wiringPiI2CWriteReg8(direccion_mag,LSM303_REGISTER_MAG_CRA_REG_M, 0x14);
}

void Sensores_IMU::leerGiro()
{
    //gyro x lsb
    u_int8_t datoGyro_0 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_X_L);

    //gyro x msb
    u_int8_t datoGyro_1 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_X_H);

    //gyro y lsb
    u_int8_t datoGyro_2 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_Y_L);

    //gyro y msb
    u_int8_t datoGyro_3 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_Y_H);

    //gyro z lsb
    u_int8_t datoGyro_4 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_Z_L);

    //gyro z msb
    u_int8_t datoGyro_5 = wiringPiI2CReadReg8(direccion_gyro,GYRO_REGISTER_OUT_Z_H);

    int xGyro = (int16_t)(datoGyro_1 * 256 + datoGyro_0);
    if(xGyro > 32767)
    {
        xGyro -= 65536;
    }
    int yGyro = (int16_t)(datoGyro_3 * 256 + datoGyro_2);
    if(yGyro > 32767)
    {
        yGyro -= 65536;
    }
    int zGyro = (int16_t)(datoGyro_5 * 256 + datoGyro_4);
    if(zGyro > 32767)
    {
        zGyro -= 65536;
    }

    xGyro*=GYRO_SENSITIVITY_2000DPS;
    yGyro*=GYRO_SENSITIVITY_2000DPS;
    zGyro*=GYRO_SENSITIVITY_2000DPS;

    gyroPuroX=xGyro;
    gyroPuroY=yGyro;
    gyroPuroZ=zGyro;
}

void Sensores_IMU::leerAcel()
{
    //acel x lsb
    u_int8_t datoAcel_0 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_X_L_A);

    //acel x msb
    u_int8_t datoAcel_1 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_X_H_A);

    //acel y lsb
    u_int8_t datoAcel_2 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_Y_L_A);

    //acel y msb
    u_int8_t datoAcel_3 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_Y_H_A);

    //acel z lsb
    u_int8_t datoAcel_4 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_Z_L_A);

    //acel z msb
    u_int8_t datoAcel_5 = wiringPiI2CReadReg8(direccion_acel,LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    // Convert the data
    int xAccl = (int16_t)(datoAcel_1 * 256 + datoAcel_0) >> 4;
    if(xAccl > 32767)
    {
        xAccl -= 65536;
    }

    int yAccl = (int16_t)(datoAcel_3 * 256 + datoAcel_2) >> 4;
    if(yAccl > 32767)
    {
        yAccl -= 65536;
    }

    int zAccl = (int16_t)(datoAcel_5 * 256 + datoAcel_4) >> 4;
    if(zAccl > 32767)
    {
        zAccl -= 65536;
    }

    double acelRuido_X = (double)xAccl*_lsm303Accel_MG_LSB* SENSORS_GRAVITY_STANDARD;
    double acelRuido_Y = (double)yAccl*_lsm303Accel_MG_LSB* SENSORS_GRAVITY_STANDARD;
    double acelRuido_Z = (double)zAccl*_lsm303Accel_MG_LSB* SENSORS_GRAVITY_STANDARD;

    if(acelRuido_X > 2*SENSORS_GRAVITY_STANDARD)
    {
        acelRuido_X = sqrt(pow(acelRuido_X - 40.1582,2));
    }
    if(acelRuido_Y > 2*SENSORS_GRAVITY_STANDARD)
    {
        acelRuido_Y = sqrt(pow(acelRuido_Y - 40.1582,2));
    }
    if(acelRuido_Z > 2*SENSORS_GRAVITY_STANDARD)
    {
        acelRuido_Z = sqrt(pow(acelRuido_Z - 40.1582,2));
    }

    acelPuroX=acelRuido_X;
    acelPuroY=acelRuido_Y;
    acelPuroZ=acelRuido_Z;
}

void Sensores_IMU::leerMag()
{
    //mag x msb
    u_int8_t datoMag_0 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_X_H_M);

    //mag x lsb
    u_int8_t datoMag_1 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_X_L_M);

    //mag y msb
    u_int8_t datoMag_2 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_Y_H_M);

    //mag y lsb
    u_int8_t datoMag_3 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_Y_L_M);

    //mag z msb
    u_int8_t datoMag_4 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_Z_H_M);

    //mag z lsb
    u_int8_t datoMag_5 = wiringPiI2CReadReg8(direccion_mag,LSM303_REGISTER_MAG_OUT_Z_L_M);

    int xMag = (int16_t)(int16_t)(datoMag_0 * 256 + (int16_t)datoMag_1);
    if(xMag > 32767)
    {
        xMag -= 65536;
    }

    int yMag = (int16_t)((int16_t)datoMag_4 * 256 + (int16_t)datoMag_5) ;
    if(yMag > 32767)
    {
        yMag -= 65536;
    }

    int zMag = (int16_t)((int16_t)datoMag_2 * 256 + (int16_t)datoMag_3) ;
    if(zMag > 32767)
    {
        zMag -= 65536;
    }

    magPuroX=xMag;
    magPuroY=yMag;
    magPuroZ=zMag;
}

void Sensores_IMU::leerSensores()
{

    leerGiro();
    gyro_X=  ((double)gyro_X*0.6 + (double)(gyroPuroX-compensacionGiroX)*0.4);
    gyro_Y=  ((double)gyro_Y*0.6 + (double)(gyroPuroY-compensacionGiroY)*0.4);
    gyro_Z=  ((double)gyro_Z*0.6 + (double)(gyroPuroZ-compensacionGiroZ)*0.4);

    leerAcel();
    acel_X = acel_X*0.9 + acelPuroX*0.1;
    acel_Y = acel_Y*0.9 + acelPuroY*0.1;
    acel_Z = acel_Z*0.9 + acelPuroZ*0.1;

    leerMag();
    mag_X= mag_X*0.9 + (magPuroX/_lsm303Mag_Gauss_LSB_XY-compensacionMagX) *0.1;
    mag_Y= mag_Y*0.9 + (magPuroY/_lsm303Mag_Gauss_LSB_XY-compensacionMagY)*0.1;
    mag_Z= mag_Z*0.9 + (magPuroZ/_lsm303Mag_Gauss_LSB_Z-compensacionMagZ)*0.1;
}

void Sensores_IMU::calibrarGiro(uint muestras)
{
    double giroXacum=0;
    double giroYacum=0;
    double giroZacum=0;
    for(uint i=0;i<muestras;i++)
    {
        leerGiro();
        giroXacum +=gyroPuroX;
        giroYacum +=gyroPuroY;
        giroZacum +=gyroPuroZ;
    }
    compensacionGiroX = (giroXacum/muestras);
    compensacionGiroY = (giroYacum/muestras);
    compensacionGiroZ = (giroZacum/muestras);
}

void Sensores_IMU::calibrarAcel(uint muestras)
{

}

void Sensores_IMU::calibrarMag(uint muestras)
{
//    double magXacum=0;
//    double magYacum=0;
//    double magZacum=0;
//    for(uint i=0;i<muestras;i++)
//    {
//        leerGiro();
//        magXacum +=magPuroX;
//        magYacum +=magPuroY;
//        magZacum +=magPuroZ;
//    }
//    compensacionMagX = (magXacum/muestras);
//    compensacionMagY = (magYacum/muestras);
//    compensacionMagZ = (magXacum/muestras);
}

int Sensores_IMU::gyroX_sinFiltro()
{
    return gyroPuroX;
}

int Sensores_IMU::gyroY_sinFiltro()
{
    return gyroPuroY;
}

int Sensores_IMU::gyroZ_sinFiltro()
{
    return gyroPuroZ;
}

double Sensores_IMU::gyroX()
{
    return gyro_X;
}

double Sensores_IMU::gyroY()
{
    return gyro_Y;
}

double Sensores_IMU::gyroZ()
{
    return gyro_Z;
}

double Sensores_IMU::acelX()
{
    return acel_X;
}

double Sensores_IMU::acelY()
{
    return acel_Y;
}

double Sensores_IMU::acelZ()
{
    return acel_Z;
}

int Sensores_IMU::acelX_sinFiltro()
{
    return acelPuroX;
}

int Sensores_IMU::acelY_sinFiltro()
{
    return acelPuroY;
}

int Sensores_IMU::acelZ_sinFiltro()
{
    return acelPuroZ;
}

double Sensores_IMU::magX()
{
    return mag_X;
}

double Sensores_IMU::magY()
{
    return mag_Y;
}

double Sensores_IMU::magZ()
{
    return mag_Z;
}

int Sensores_IMU::magX_sinFiltro()
{
    return magPuroX;
}

int Sensores_IMU::magY_sinFiltro()
{
    return magPuroY;
}

int Sensores_IMU::magZ_sinFiltro()
{
    return magPuroZ;
}
