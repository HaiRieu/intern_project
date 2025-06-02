#include "functiondef.h"

uint32_t timestamp;
bool IMUsAvailable = false;

// static IMU_data_Raw_packed ImuArray[NUM_IMUS] ;

/*
brief Initializes the IMU sensors and sensor fusion filters.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param fillsion1 Reference to the first sensor fusion filter
@param fillsion2 Reference to the second sensor fusion filter
@param overallStatusDatapPacked Reference to the packed data structure for overall status
@return true if initialization is successful, false otherwise

*/
bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
             Adafruit_LSM6DS3TRC &lsm6ds2,
             Adafruit_LIS3MDL &lis3mdl1,
             Adafruit_LIS3MDL &lis3mdl2,
             Adafruit_NXPSensorFusion &fillsion1,
             Adafruit_NXPSensorFusion &fillsion2,
             Overall_status_data_Union &overallStatusDatapPacked)
{

    if (!lsm6ds1.begin_I2C(0x6A))
    {
        Serial.println("Failed to find LSM6DS1 chip");
        overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
        overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    
    }
    if (!lsm6ds2.begin_I2C(0x6B))
    {
        Serial.println("Failed to find LSM6DS2 chip");
        overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
      
    }
    if (!lis3mdl1.begin_I2C(0x1E))
    {
        Serial.println("Failed to find LIS3MDL chip 1");
    
    }
    if (!lis3mdl1.begin_I2C(0x1C))
    {
        Serial.println("Failed to find LIS3MDL chip 2");
    
    }

    fillsion1.begin(FILTER_UPDATE_RATE_HZ);
    fillsion2.begin(FILTER_UPDATE_RATE_HZ);
    timestamp = millis();

    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
    IMUsAvailable = true;
    return true;
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
    lsm6ds1.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    lsm6ds1.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    lsm6ds1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

    lis3mdl1.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);

    lsm6ds2.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    lsm6ds2.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
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
                 IMU_data_Raw_packed ImuArray[NUM_IMUS],
                 IMU1_euler_calib_status_Union &imu1EulerCalibration,
                 IMU2_euler_calib_status_Union &imu2EulerCalibration)
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
        ImuArray[0].data_Imu.accelX = accel1.acceleration.x;
        ImuArray[0].data_Imu.accelY = accel1.acceleration.y;
        ImuArray[0].data_Imu.accelZ = accel1.acceleration.z;

        ImuArray[0].data_Imu.GyroX = gyro1.gyro.x;
        ImuArray[0].data_Imu.GyroY = gyro1.gyro.y;
        ImuArray[0].data_Imu.GyroZ = gyro1.gyro.z;

        ImuArray[0].data_Imu.MagX = mag1.magnetic.x;
        ImuArray[0].data_Imu.MagY = mag1.magnetic.y;
        ImuArray[0].data_Imu.MagZ = mag1.magnetic.z;

        ImuArray[1].data_Imu.accelX = accel2.acceleration.x;
        ImuArray[1].data_Imu.accelY = accel2.acceleration.y;
        ImuArray[1].data_Imu.accelZ = accel2.acceleration.z;

        ImuArray[1].data_Imu.GyroX = gyro2.gyro.x;
        ImuArray[1].data_Imu.GyroY = gyro2.gyro.y;
        ImuArray[1].data_Imu.GyroZ = gyro2.gyro.z;

        ImuArray[1].data_Imu.MagX = mag2.magnetic.x;
        ImuArray[1].data_Imu.MagX = mag2.magnetic.y;
        ImuArray[1].data_Imu.MagX = mag2.magnetic.z;

        fillsion1.update(ImuArray[0].data_Imu.accelX, ImuArray[0].data_Imu.accelY, ImuArray[0].data_Imu.accelZ,
                         ImuArray[0].data_Imu.GyroX, ImuArray[0].data_Imu.GyroY, ImuArray[0].data_Imu.GyroZ,
                         ImuArray[0].data_Imu.MagX, ImuArray[0].data_Imu.MagY, ImuArray[0].data_Imu.MagZ);

        fillsion2.update(ImuArray[1].data_Imu.accelX, ImuArray[1].data_Imu.accelY, ImuArray[1].data_Imu.accelZ,
                         ImuArray[1].data_Imu.GyroX, ImuArray[1].data_Imu.GyroY, ImuArray[1].data_Imu.GyroZ,
                         ImuArray[1].data_Imu.MagX, ImuArray[1].data_Imu.MagY, ImuArray[1].data_Imu.MagZ);

        imu1EulerCalibration.eulerCalibStatus.EulerRoll = fillsion1.getRoll();
        imu1EulerCalibration.eulerCalibStatus.EulerPitch = fillsion1.getPitch();
        imu1EulerCalibration.eulerCalibStatus.EulerYaw = fillsion1.getYaw();

        imu2EulerCalibration.eulerCalibStatus.EulerRoll = fillsion2.getRoll();
        imu2EulerCalibration.eulerCalibStatus.EulerPitch = fillsion2.getPitch();
        imu2EulerCalibration.eulerCalibStatus.EulerYaw = fillsion2.getYaw();
    }
    else
    {
    }
}

