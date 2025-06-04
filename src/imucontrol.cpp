#include "functiondef.h"

uint32_t timestamp;
bool IMUsAvailable = false;
bool imu1Status = true;
bool imu2Status = true;

/*
@brief Initializes the first IMU sensors and sensor fusion filter.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param fusion Reference to the sensor fusion filter
@param overallStatusDatapPacked Reference to the packed data structure for overall status
@return true if initialization is successful, false otherwise
*/
bool initIMU1(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              Adafruit_NXPSensorFusion &fusion,
              OverallStatusDataUnion &overallStatusDatapPacked)
{

    if (!lsm6ds.begin_I2C(0x6A))
    {
        Serial.println("Failed to find LSM6DS1 chip");
        overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
        imu1Status = false;
    }
    else
    {
        Serial.println("LSM6DS1 initialized successfully");
    }

    if (!lis3mdl.begin_I2C(0x1E))
    {
        Serial.println("Failed to find LIS3MDL chip ");
        overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
        imu1Status = false;
    }
    else
    {
        Serial.println("LIS3MDL1 initialized successfully");
    }

    if (imu1Status)
    {
        fusion.begin(FILTER_UPDATE_RATE_HZ);
        overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    }
    else
    {
        Serial.println("IMU1 initialization FAILED");
    }
    return imu1Status;
}

/*
@brief Initializes the second IMU sensors and sensor fusion filter.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param fusion Reference to the sensor fusion filter
@param overallStatusDatapPacked Reference to the packed data structure for overall status
@return true if initialization is successful, false otherwise
*/
bool initIMU2(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              Adafruit_NXPSensorFusion &fusion,
              OverallStatusDataUnion &overallStatusDatapPacked)
{

    if (!lsm6ds.begin_I2C(0x6B))
    {
        Serial.println("Failed to find LSM6DS2 chip");
        overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
        imu2Status = false;
    }
    else
    {
        Serial.println("LSM6DS2 initialized successfully");
    }

    if (!lis3mdl.begin_I2C(0x1C))
    {
        Serial.println("Failed to find LIS3MDL2 chip ");
        overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
        imu2Status = false;
    }
    else
    {
        Serial.println("LIS3MDL2 initialized successfully");
    }

    if (imu2Status)
    {
        fusion.begin(FILTER_UPDATE_RATE_HZ);
         overallStatusDatapPacked.overallStatusData.Imu2Status= statusCodeSensor::RUNNING;
    }
    else
    {
        Serial.println("IMU2 initialization FAILED");
    }
    return imu2Status;
}

/*
@brief Starts the IMU sensors and initializes the sensor fusion filters.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param fillsion1 Reference to the first sensor fusion filter
@param fillsion2 Reference to the second sensor fusion filter
@param overallStatusDatapPacked Reference to the packed data structure for overall status
@return true if at least one IMU is available, false otherwise
*/
bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds1,
              Adafruit_LSM6DS3TRC &lsm6ds2,
              Adafruit_LIS3MDL &lis3mdl1,
              Adafruit_LIS3MDL &lis3mdl2,
              Adafruit_NXPSensorFusion &fillsion1,
              Adafruit_NXPSensorFusion &fillsion2,
              OverallStatusDataUnion &overallStatusDatapPacked)
{

    initIMU1(lsm6ds1, lis3mdl1, fillsion1, overallStatusDatapPacked);
    initIMU2(lsm6ds2, lis3mdl2, fillsion2, overallStatusDatapPacked);

    bool imusAvailable = imu1Status || imu2Status;
    return imusAvailable;
}

/*
brief Sets up the data rate for the IMU sensors.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
*/
void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds1,
                      Adafruit_LSM6DS3TRC &lsm6ds2,
                      Adafruit_LIS3MDL &lis3mdl1,
                      Adafruit_LIS3MDL &lis3mdl2)
{
    lsm6ds1.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds1.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

    lis3mdl1.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);

    lsm6ds2.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds2.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds2.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

    lis3mdl2.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl2.setRange(LIS3MDL_RANGE_4_GAUSS);
}

/*
brief Reads data from the IMU sensors and updates the sensor fusion filter.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param cal Reference to the sensor calibration object
@param fillsion1 Reference to the first sensor fusion filter
@param fillsion2 Reference to the second sensor fusion filter
@param imu1DataRawPacked Reference to the packed data structure for IMU 1
@param imu2DataRawPacked Reference to the packed data structure for IMU 2
@param imu1EulerCalibration Reference to the packed data structure for IMU 1 Euler calibration status
@param imu2EulerCalibration Reference to the packed data structure for IMU 2 Euler calibration status
*/
void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LSM6DS3TRC &lsm6ds2,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_LIS3MDL &lis3mdl2,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fillsion1,
                 Adafruit_NXPSensorFusion &fillsion2,
                 IMUDataRawUnion ImuArray[NUM_IMUS],
                 IMUEulernUnion IMUeurle[NUM_IMUS])
{

    if (IMUsAvailable)
    {

        sensors_event_t accel1, gyro1, mag1;
        sensors_event_t accel2, gyro2, mag2;

        lsm6ds1.getEvent(&accel1, &gyro1, NULL);
        lsm6ds2.getEvent(&accel2, &gyro2, NULL);
        lis3mdl1.getEvent(&mag1);
        lis3mdl2.getEvent(&mag2);

        cal.calibrate(accel1);
        cal.calibrate(gyro1);
        cal.calibrate(mag1);
        cal.calibrate(accel2);
        cal.calibrate(gyro2);
        cal.calibrate(mag2);
        cal.saveCalibration();

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

        ImuArray[1].dataImu.accelXmg = accel2.acceleration.x;
        ImuArray[1].dataImu.accelYmg = accel2.acceleration.y;
        ImuArray[1].dataImu.accelZmg = accel2.acceleration.z;

        ImuArray[1].dataImu.GyroXrads = gyro2.gyro.x;
        ImuArray[1].dataImu.GyroYrads = gyro2.gyro.y;
        ImuArray[1].dataImu.GyroZrads = gyro2.gyro.z;

        ImuArray[1].dataImu.MagXuT = mag2.magnetic.x;
        ImuArray[1].dataImu.MagXuT = mag2.magnetic.y;
        ImuArray[1].dataImu.MagXuT = mag2.magnetic.z;

        fillsion1.update(ImuArray[0].dataImu.accelXmg, ImuArray[0].dataImu.accelYmg, ImuArray[0].dataImu.accelZmg,
                         ImuArray[0].dataImu.GyroXrads, ImuArray[0].dataImu.GyroYrads, ImuArray[0].dataImu.GyroZrads,
                         ImuArray[0].dataImu.MagXuT, ImuArray[0].dataImu.MagYuT, ImuArray[0].dataImu.MagZuT);

        fillsion2.update(ImuArray[1].dataImu.accelXmg, ImuArray[1].dataImu.accelYmg, ImuArray[1].dataImu.accelZmg,
                         ImuArray[1].dataImu.GyroXrads, ImuArray[1].dataImu.GyroYrads, ImuArray[1].dataImu.GyroZrads,
                         ImuArray[1].dataImu.MagXuT, ImuArray[1].dataImu.MagYuT, ImuArray[1].dataImu.MagZuT);

        IMUeurle[0].eulerCalibStatus.EulerRolldeg = fillsion1.getRoll();
        IMUeurle[0].eulerCalibStatus.EulerPitchdeg = fillsion1.getPitch();
        IMUeurle[0].eulerCalibStatus.EulerYawdeg = fillsion1.getYaw();

        IMUeurle[1].eulerCalibStatus.EulerRolldeg = fillsion2.getRoll();
        IMUeurle[1].eulerCalibStatus.EulerPitchdeg = fillsion2.getPitch();
        IMUeurle[1].eulerCalibStatus.EulerYawdeg = fillsion2.getYaw();
    }
    
}
