#include "functiondef.h"

uint32_t timestamp;
bool IMUsAvailable = false;



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
             Overall_status_data_packed &overallStatusDatapPacked)
{

    if (!lsm6ds1.begin_I2C(0x6A))
    {
        Serial.println("Failed to find LSM6DS1 chip");
        overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;

        overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
        return false;
    }
    if (!lsm6ds2.begin_I2C(0x6B))
    {
        Serial.println("Failed to find LSM6DS2 chip");
        overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
        return false;
    }
    if (!lis3mdl1.begin_I2C(0x1E))
    {
        Serial.println("Failed to find LIS3MDL chip 1");
        return false;
    }
    if (!lis3mdl1.begin_I2C(0x1C))
    {
        Serial.println("Failed to find LIS3MDL chip 2");
        return false;
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
                 IMU1_data_Raw_packed &imu1DataRawPacked,
                 IMU2_data_Raw_packed &imu2DataRawPacked,
                 IMU1_euler_calib_status_packed &imu1EulerCalibration,
                 IMU2_euler_calib_status_packed &imu2EulerCalibration)
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
        imu1DataRawPacked.data_Imu1.accelX = accel1.acceleration.x;
        imu1DataRawPacked.data_Imu1.accelY = accel1.acceleration.y;
        imu1DataRawPacked.data_Imu1.accelZ = accel1.acceleration.z;

        imu1DataRawPacked.data_Imu1.GyroX = gyro1.gyro.x;
        imu1DataRawPacked.data_Imu1.GyroY = gyro1.gyro.y;
        imu1DataRawPacked.data_Imu1.GyroZ = gyro1.gyro.z;

        imu1DataRawPacked.data_Imu1.MagX = mag1.magnetic.x;
        imu1DataRawPacked.data_Imu1.MagY = mag1.magnetic.y;
        imu1DataRawPacked.data_Imu1.MagZ = mag1.magnetic.z;

        imu2DataRawPacked.data_Imu2.accelX = accel2.acceleration.x;
        imu2DataRawPacked.data_Imu2.accelY = accel2.acceleration.y;
        imu2DataRawPacked.data_Imu2.accelZ = accel2.acceleration.z;

        imu2DataRawPacked.data_Imu2.GyroX = gyro2.gyro.x;
        imu2DataRawPacked.data_Imu2.GyroY = gyro2.gyro.y;
        imu2DataRawPacked.data_Imu2.GyroZ = gyro2.gyro.z;

        imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.x;
        imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.y;
        imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.z;

        fillsion1.update(imu1DataRawPacked.data_Imu1.accelX, imu1DataRawPacked.data_Imu1.accelY, imu1DataRawPacked.data_Imu1.accelZ,
                         imu1DataRawPacked.data_Imu1.GyroX, imu1DataRawPacked.data_Imu1.GyroY, imu1DataRawPacked.data_Imu1.GyroZ,
                         imu1DataRawPacked.data_Imu1.MagX, imu1DataRawPacked.data_Imu1.MagY, imu1DataRawPacked.data_Imu1.MagZ);

        fillsion2.update(imu2DataRawPacked.data_Imu2.accelX, imu2DataRawPacked.data_Imu2.accelY, imu2DataRawPacked.data_Imu2.accelZ,
                         imu2DataRawPacked.data_Imu2.GyroX, imu2DataRawPacked.data_Imu2.GyroY, imu2DataRawPacked.data_Imu2.GyroZ,
                         imu2DataRawPacked.data_Imu2.MagX, imu2DataRawPacked.data_Imu2.MagY, imu2DataRawPacked.data_Imu2.MagZ);

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

/*
brief Sends IMU data over BLE if the gamepad is connected.
@param bleGamepad Reference to the BleGamepad object
@param imu1DataRawPacked Reference to the packed data structure for IMU 1
@param imu2DataRawPacked Reference to the packed data structure for IMU 2
@param imu1EulerCalibration Reference to the packed data structure for IMU 1 Euler calibration status
@param imu2EulerCalibration Reference to the packed data structure for IMU 2 Euler calibration status
*/
void senDataBLE(BleGamepad &bleGamepad,
                IMU1_data_Raw_packed &imu1DataRawPacked,
                IMU2_data_Raw_packed &imu2DataRawPacked,
                IMU1_euler_calib_status_packed &imu1EulerCalibration,
                IMU2_euler_calib_status_packed &imu2EulerCalibration)
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, imu1DataRawPacked.rawData, sizeof(imu1DataRawPacked.rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, imu2DataRawPacked.rawData, sizeof(imu2DataRawPacked.rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, imu1EulerCalibration.rawData, sizeof(imu1EulerCalibration.rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, imu2EulerCalibration.rawData, sizeof(imu2EulerCalibration.rawData));
    }
}
