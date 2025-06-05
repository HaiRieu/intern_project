#include "functiondef.h"

uint32_t timestamp;
//bool imu1Status = true;
// bool imu2Status = true;
 bool imu1Status = false;
 bool imu2Status = false;

/*
@brief Starts the IMU sensors and initializes the sensor fusion.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param addressLsm6ds I2C address for the LSM6DS3TRC sensor
@param addressLis3mdl I2C address for the LIS3MDL sensor
@param overallDataUnion Reference to the OverallStatusDataUnion structure to store status data
@param fusionImu Reference to the Adafruit_NXPSensorFusion object for sensor fusion
@return True if both IMU sensors are initialized successfully, false otherwise
*/
bool imu1Start(Adafruit_LSM6DS3TRC &lsm6ds,
               Adafruit_LIS3MDL &lis3mdl,
               uint8_t addressLsm6ds,
               uint8_t addressLis3mdl,
               OverallStatusDataUnion &overallDataUnion,
               Adafruit_NXPSensorFusion &fusionImu)
{
    bool imuStatusLsm6ds = false;
    bool imuStatusLis3mdl = false;
   

    if (lsm6ds.begin_I2C(addressLsm6ds))
    {
        switch (addressLsm6ds)
        {
        case ADDRESS_LSM6DS1:
            Serial.println("LSM6DS1 initialized successfully");
            imuStatusLsm6ds = true;
            break;

        case ADDRESS_LSM6DS2:
            Serial.println("LSM6DS2 initialized successfully");
            imuStatusLsm6ds = true;
            break;
        default:
            Serial.println("Unknown LSM6DS3TRC address");
            imuStatusLsm6ds = false;
            break;
        }
    }

    if (lis3mdl.begin_I2C(addressLis3mdl))
    {

        switch (addressLis3mdl)
        {
        case ADDRESS_LIS3MDL1:
            Serial.println("LIS3MDL1 initialized successfully");
            imuStatusLis3mdl = true;
            break;

        case ADDRESS_LIS3MDL2:
            Serial.println("LIS3MDL2 initialized successfully");
            imuStatusLis3mdl = true;
            break;
        default:
            Serial.println("Unknown LIS3MDL address");
            imuStatusLis3mdl = false;
            break;
        }
    }
    imu1Status = imuStatusLsm6ds && imuStatusLis3mdl;
    if (imu1Status)
    {
        fusionImu.begin(FILTER_UPDATE_RATE_HZ);
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    }
    else
    {
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
    }
    return imu1Status;
}

/*
@brief Starts the IMU sensors and initializes the sensor fusion.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param addressLsm6ds I2C address for the LSM6DS3TRC sensor
@param addressLis3mdl I2C address for the LIS3MDL sensor
@param overallDataUnion Reference to the OverallStatusDataUnion structure to store status data
@param fusionImu Reference to the Adafruit_NXPSensorFusion object for sensor fusion
@return True if both IMU sensors are initialized successfully, false otherwise
*/
bool imu2Start(Adafruit_LSM6DS3TRC &lsm6ds,
               Adafruit_LIS3MDL &lis3mdl,
               uint8_t addressLsm6ds,
               uint8_t addressLis3mdl,
               OverallStatusDataUnion &overallDataUnion,
               Adafruit_NXPSensorFusion &fusionImu)
{
    bool imuStatusLsm6ds = false;
    bool imuStatusLis3mdl = false;

    if (lsm6ds.begin_I2C(addressLsm6ds))
    {
        switch (addressLsm6ds)
        {
        case 0x6A:
            Serial.println("LSM6DS1 initialized successfully");
            imuStatusLsm6ds = true;
            break;

        case 0x6B:
            Serial.println("LSM6DS2 initialized successfully");
            imuStatusLsm6ds = true;
            break;
        default:
            Serial.println("Unknown LSM6DS3TRC address");
            imuStatusLsm6ds = false;
            break;
        }
    }

    if (lis3mdl.begin_I2C(addressLis3mdl))
    {

        switch (addressLis3mdl)
        {
        case 0x1E:
            Serial.println("LIS3MDL1 initialized successfully");
            imuStatusLis3mdl = true;
            break;

        case 0x1C:
            Serial.println("LIS3MDL2 initialized successfully");
            imuStatusLis3mdl = true;
            break;
        default:
            Serial.println("Unknown LIS3MDL address");
            imuStatusLis3mdl = false;
            break;
        }
    }
    imu2Status = imuStatusLsm6ds && imuStatusLis3mdl;
    if (imu2Status)
    {
        fusionImu.begin(FILTER_UPDATE_RATE_HZ);
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    }
    else
    {
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
    }

    return imu2Status;
}

/*
brief Sets the data rate and range for the IMU sensors.
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
*/
void setupIMU2DataRate(Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl2)
{

    lsm6ds2.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds2.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds2.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl2.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl2.setRange(LIS3MDL_RANGE_4_GAUSS);
}

/*
@brief Sets the data rate and range for the first IMU sensors.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
*/
void setupIMU1DataRate(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LIS3MDL &lis3mdl1)
{
    lsm6ds1.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds1.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl1.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);
}


/*
@brief Reads data from the IMU sensors and updates the IMUDataRawUnion and IMUEulernUnion arrays.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object for calibration
@param fusion1 Reference to the first Adafruit_NXPSensorFusion object for sensor fusion
@param fusion2 Reference to the second Adafruit_NXPSensorFusion object for sensor fusion
@param ImuArray Array of IMUDataRawUnion structures to store raw IMU data
@param IMUeurle Array of IMUEulernUnion structures to store Euler angles
*/

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LSM6DS3TRC &lsm6ds2,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_LIS3MDL &lis3mdl2,
                 Adafruit_Sensor_Calibration_EEPROM &cal1,
                 Adafruit_Sensor_Calibration_EEPROM &cal2,   
                 Adafruit_NXPSensorFusion &fusion1,
                 Adafruit_NXPSensorFusion &fusion2,
                 IMUDataRawUnion ImuArray[NUM_IMUS],
                 IMUEulernUnion &IMUeurle1,
                 IMUEulernUnion &IMUeurle2)
{
    if (imu1Status)
    {

        sensors_event_t accel1, gyro1, mag1;

        lsm6ds1.getEvent(&accel1, &gyro1, NULL);
        lis3mdl1.getEvent(&mag1);

        cal1.calibrate(accel1);
        cal1.calibrate(gyro1);
        cal1.calibrate(mag1);
        cal1.saveCalibration();

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
        {
            return;
        }
        timestamp = millis();
        ImuArray[0].dataImu.accelXmg = accel1.acceleration.x;
        ImuArray[0].dataImu.accelYmg = accel1.acceleration.y;
        ImuArray[0].dataImu.accelZmg = accel1.acceleration.z;

        ImuArray[0].dataImu.GyroXrads = gyro1.gyro.x;
        ImuArray[0].dataImu.GyroYrads = gyro1.gyro.y;
        ImuArray[0].dataImu.GyroZrads = gyro1.gyro.z;

        ImuArray[0].dataImu.MagXuT = mag1.magnetic.x;
        ImuArray[0].dataImu.MagYuT = mag1.magnetic.y;
        ImuArray[0].dataImu.MagZuT = mag1.magnetic.z;

        fusion1.update(ImuArray[0].dataImu.accelXmg, ImuArray[0].dataImu.accelYmg, ImuArray[0].dataImu.accelZmg,
                       ImuArray[0].dataImu.GyroXrads, ImuArray[0].dataImu.GyroYrads, ImuArray[0].dataImu.GyroZrads,
                       ImuArray[0].dataImu.MagXuT, ImuArray[0].dataImu.MagYuT, ImuArray[0].dataImu.MagZuT);

        IMUeurle1.eulerCalibStatus.EulerRolldeg = fusion1.getRoll();
        IMUeurle1.eulerCalibStatus.EulerPitchdeg = fusion1.getPitch();
        IMUeurle1.eulerCalibStatus.EulerYawdeg = fusion1.getYaw();
    }else {
       IMUeurle1.eulerCalibStatus.calibation = false ; 
    }

    if (imu2Status)
    {

        sensors_event_t accel2, gyro2, mag2;
        lsm6ds2.getEvent(&accel2, &gyro2, NULL);

        lis3mdl2.getEvent(&mag2);

        cal2.calibrate(accel2);
        cal2.calibrate(gyro2);
        cal2.calibrate(mag2);
        cal2.saveCalibration();

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
        {
            return;
        }
        timestamp = millis();

        ImuArray[1].dataImu.accelXmg = accel2.acceleration.x;
        ImuArray[1].dataImu.accelYmg = accel2.acceleration.y;
        ImuArray[1].dataImu.accelZmg = accel2.acceleration.z;

        ImuArray[1].dataImu.GyroXrads = gyro2.gyro.x;
        ImuArray[1].dataImu.GyroYrads = gyro2.gyro.y;
        ImuArray[1].dataImu.GyroZrads = gyro2.gyro.z;

        ImuArray[1].dataImu.MagXuT = mag2.magnetic.x;
        ImuArray[1].dataImu.MagYuT = mag2.magnetic.y;
        ImuArray[1].dataImu.MagZuT = mag2.magnetic.z;

        fusion2.update(ImuArray[1].dataImu.accelXmg, ImuArray[1].dataImu.accelYmg, ImuArray[1].dataImu.accelZmg,
                       ImuArray[1].dataImu.GyroXrads, ImuArray[1].dataImu.GyroYrads, ImuArray[1].dataImu.GyroZrads,
                       ImuArray[1].dataImu.MagXuT, ImuArray[1].dataImu.MagYuT, ImuArray[1].dataImu.MagZuT);

        IMUeurle2.eulerCalibStatus.EulerRolldeg = fusion2.getRoll();
        IMUeurle2.eulerCalibStatus.EulerPitchdeg = fusion2.getPitch();
        IMUeurle2.eulerCalibStatus.EulerYawdeg = fusion2.getYaw();
    }else {
          IMUeurle2.eulerCalibStatus.calibation = false ;
    }
}
