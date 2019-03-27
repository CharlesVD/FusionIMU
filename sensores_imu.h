#ifndef SENSORES_IMU_H
#define SENSORES_IMU_H

#include <sys/types.h>
#include <wiringPiI2C.h>
#include <cmath>

/*=========================================================================
    CONSTANTES
    -----------------------------------------------------------------------*/
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */
#define PI_F                              (3.14159265F)
/*=========================================================================*/

/*=========================================================================
    DIRECCIONES Y CONFIGURACIONES I2C DEL GIROSCOPIO
    -----------------------------------------------------------------------*/
#define L3GD20_ADDRESS_GYRO      (0x6B)        // 1101011
#define L3GD20_POLL_TIMEOUT      (100)         // lectura màxima
#define L3GD20_ID                (0xD4)
#define L3GD20H_ID               (0xD7)
// valores de sensivilidad tomados de la hoja del datos del producto
#define GYRO_SENSITIVITY_250DPS  (0.00875F)
#define GYRO_SENSITIVITY_500DPS  (0.0175F)
#define GYRO_SENSITIVITY_2000DPS (0.070F)
/*=========================================================================*/

/*=========================================================================
    DIRECCIONES I2C DEL ACELERÒMETRO Y MAGNETÒMETRO
    -----------------------------------------------------------------------*/
#define LSM303_ADDRESS_ACCEL          (0x19)         // 0011001x
#define LSM303_ADDRESS_MAG            (0x1E)         // 0011110x
/*=========================================================================*/

class Sensores_IMU
{
public:
    Sensores_IMU();

    void leerGiro();
    void leerAcel();
    void leerMag();

    void leerSensores();

    double gyroX();
    double gyroY();
    double gyroZ();

    int gyroX_sinFiltro();
    int gyroY_sinFiltro();
    int gyroZ_sinFiltro();

    double acelX();
    double acelY();
    double acelZ();

    int acelX_sinFiltro();
    int acelY_sinFiltro();
    int acelZ_sinFiltro();

    double magX();
    double magY();
    double magZ();

    int magX_sinFiltro();
    int magY_sinFiltro();
    int magZ_sinFiltro();

    void calibrarGiro(uint muestras);
    void calibrarAcel(uint muestras);
    void calibrarMag(uint muestras);
private:

    /*=========================================================================
        REGISTROS DEL GIROSCOPIO
        -----------------------------------------------------------------------*/
    u_int8_t GYRO_REGISTER_WHO_AM_I            = 0x0F;   // 11010100   r
    u_int8_t GYRO_REGISTER_CTRL_REG1           = 0x20;   // 00000111   rw
    u_int8_t GYRO_REGISTER_CTRL_REG2           = 0x21;   // 00000000   rw
    u_int8_t GYRO_REGISTER_CTRL_REG3           = 0x22;   // 00000000   rw
    u_int8_t GYRO_REGISTER_CTRL_REG4           = 0x23;   // 00000000   rw
    u_int8_t GYRO_REGISTER_CTRL_REG5           = 0x24;   // 00000000   rw
    u_int8_t GYRO_REGISTER_REFERENCE           = 0x25;   // 00000000   rw
    u_int8_t GYRO_REGISTER_OUT_TEMP            = 0x26;   //            r
    u_int8_t GYRO_REGISTER_STATUS_REG          = 0x27;   //            r
    u_int8_t GYRO_REGISTER_OUT_X_L             = 0x28;   //            r
    u_int8_t GYRO_REGISTER_OUT_X_H             = 0x29;   //            r
    u_int8_t GYRO_REGISTER_OUT_Y_L             = 0x2A;   //            r
    u_int8_t GYRO_REGISTER_OUT_Y_H             = 0x2B;   //            r
    u_int8_t GYRO_REGISTER_OUT_Z_L             = 0x2C;   //            r
    u_int8_t GYRO_REGISTER_OUT_Z_H             = 0x2D;   //            r
    u_int8_t GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E;   // 00000000   rw
    u_int8_t GYRO_REGISTER_FIFO_SRC_REG        = 0x2F;   //            r
    u_int8_t GYRO_REGISTER_INT1_CFG            = 0x30;   // 00000000   rw
    u_int8_t GYRO_REGISTER_INT1_SRC            = 0x31;   //            r
    u_int8_t GYRO_REGISTER_TSH_XH              = 0x32;   // 00000000   rw
    u_int8_t GYRO_REGISTER_TSH_XL              = 0x33;   // 00000000   rw
    u_int8_t GYRO_REGISTER_TSH_YH              = 0x34;   // 00000000   rw
    u_int8_t GYRO_REGISTER_TSH_YL              = 0x35;   // 00000000   rw
    u_int8_t GYRO_REGISTER_TSH_ZH              = 0x36;   // 00000000   rw
    u_int8_t GYRO_REGISTER_TSH_ZL              = 0x37;   // 00000000   rw
    u_int8_t GYRO_REGISTER_INT1_DURATION       = 0x38;    // 00000000   rw
    /*=========================================================================*/


    /*=========================================================================
        REGISTROS DEL ACELERÒMETRO
        -----------------------------------------------------------------------*/
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20;   // 00000111   rw
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21;   // 00000000   rw
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22;   // 00000000   rw
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23;   // 00000000   rw
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24;   // 00000000   rw
    u_int8_t LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25;   // 00000000   rw
    u_int8_t LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26;   // 00000000   r
    u_int8_t LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27;   // 00000000   r
    u_int8_t LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28;
    u_int8_t LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29;
    u_int8_t LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A;
    u_int8_t LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B;
    u_int8_t LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C;
    u_int8_t LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D;
    u_int8_t LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E;
    u_int8_t LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F;
    u_int8_t LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30;
    u_int8_t LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31;
    u_int8_t LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32;
    u_int8_t LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33;
    u_int8_t LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34;
    u_int8_t LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35;
    u_int8_t LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36;
    u_int8_t LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37;
    u_int8_t LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38;
    u_int8_t LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39;
    u_int8_t LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A;
    u_int8_t LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B;
    u_int8_t LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C;
    u_int8_t LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D;
    /*=========================================================================*/


    float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
    float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
    float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain


    /*=========================================================================
        REGISTROS DEL MEGNETÒMETRO
        -----------------------------------------------------------------------*/
    u_int8_t LSM303_REGISTER_MAG_CRA_REG_M             = 0x00;
    u_int8_t LSM303_REGISTER_MAG_CRB_REG_M             = 0x01;
    u_int8_t LSM303_REGISTER_MAG_MR_REG_M              = 0x02;
    u_int8_t LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03;
    u_int8_t LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04;
    u_int8_t LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05;
    u_int8_t LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06;
    u_int8_t LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07;
    u_int8_t LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08;
    u_int8_t LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09;
    u_int8_t LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A;
    u_int8_t LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B;
    u_int8_t LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C;
    u_int8_t LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31;
    u_int8_t LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32;
    /*=========================================================================*/


    /*=========================================================================
        GANANCIAS DEL MAGENTÒMETRO
        -----------------------------------------------------------------------*/

    u_int8_t LSM303_MAGGAIN_1_3                        = 0x20;  // +/- 1.3
    u_int8_t LSM303_MAGGAIN_1_9                        = 0x40;  // +/- 1.9
    u_int8_t LSM303_MAGGAIN_2_5                        = 0x60;  // +/- 2.5
    u_int8_t LSM303_MAGGAIN_4_0                        = 0x80;  // +/- 4.0
    u_int8_t LSM303_MAGGAIN_4_7                        = 0xA0;  // +/- 4.7
    u_int8_t LSM303_MAGGAIN_5_6                        = 0xC0;  // +/- 5.6
    u_int8_t LSM303_MAGGAIN_8_1                        = 0xE0;   // +/- 8.1

    /*=========================================================================*/

    /*=========================================================================
        VELOCIDADES DEL MAGNETÒMETRO
        -----------------------------------------------------------------------*/

    u_int8_t LSM303_MAGRATE_0_7                        = 0x00;  // 0.75 Hz
    u_int8_t LSM303_MAGRATE_1_5                        = 0x01;  // 1.5 Hz
    u_int8_t LSM303_MAGRATE_3_0                        = 0x62;  // 3.0 Hz
    u_int8_t LSM303_MAGRATE_7_5                        = 0x03;  // 7.5 Hz
    u_int8_t LSM303_MAGRATE_15                         = 0x04;  // 15 Hz
    u_int8_t LSM303_MAGRATE_30                         = 0x05;  // 30 Hz
    u_int8_t LSM303_MAGRATE_75                         = 0x06;  // 75 Hz
    u_int8_t LSM303_MAGRATE_220                        = 0x07;   // 200 Hz

    /*=========================================================================*/

    u_int8_t direccion_gyro;
    u_int8_t direccion_acel;
    u_int8_t direccion_mag;

    double gyro_X;
    double gyro_Y;
    double gyro_Z;

    int gyroPuroX;
    int gyroPuroY;
    int gyroPuroZ;

    double compensacionGiroX=0;
    double compensacionGiroY=-6;
    double compensacionGiroZ=+2;

    double acel_X;
    double acel_Y;
    double acel_Z;

    double acelPuroX;
    double acelPuroY;
    double acelPuroZ;

    double compensacionAcelX=0;
    double compensacionAcelY=0;
    double compensacionAcelZ=0;

    double mag_X;
    double mag_Y;
    double mag_Z;

    int magPuroX;
    int magPuroY;
    int magPuroZ;

    double compensacionMagX=0;
    double compensacionMagY=0;
    double compensacionMagZ=0;
};

#endif
