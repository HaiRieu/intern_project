#include "app.h"
#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

void appsetup();
void appprocess();


bool restoreSettings(  IMU_config_data_anJoystick_packed &configData);

void saveSetting(IMU_config_data_anJoystick_packed &configData) ;

bool validateSettings(IMU_config_data_anJoystick_packed &configData) ; 

void setDefaultSettings(IMU_config_data_anJoystick_packed &configData);

bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
             Adafruit_LSM6DS3TRC &lsm6ds2,
             Adafruit_LIS3MDL &lis3mdl1,
             Adafruit_LIS3MDL &lis3mdl2,
             Adafruit_NXPSensorFusion &fillsion1,
             Adafruit_NXPSensorFusion &fillsion2,
             Overall_status_data_packed &overallStatusDatapPacked);

void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds1,
                      Adafruit_LSM6DS3TRC &lsm6ds2,
                      Adafruit_LIS3MDL &lis3mdl1,
                      Adafruit_LIS3MDL &lis3mdl2);

void setupIMUInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1,
                        Adafruit_LSM6DS3TRC &lsm6ds2,
                        Adafruit_LIS3MDL &lis3mdl1,
                        Adafruit_LIS3MDL &lis3mdl2);

void IRAM_ATTR imu1InterruptHandler();

bool loadCalibration(IMU1_euler_calib_status_packed &imu1EulerCalibration
  , IMU2_euler_calib_status_packed &imu2EulerCalibration
  , calibratee &calibrationData);

void setupBLEGamepad(BleGamepad &bleGamepad,
                     BleGamepadConfiguration &bleGamepadConfig);

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge ,Overall_status_data_packed &overallStatusDatapPacked);
void ledRGB();

void updateOverallStatusData(Overall_status_data_packed &overallStatusDatapPacked);

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LSM6DS3TRC &lsm6ds2,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_LIS3MDL &lis3mdl2,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fillsion1,
                 Adafruit_NXPSensorFusion &fillsion2,
                 IMU_data_Raw_packed ImuArray[NUM_IMUS], 
                 IMU1_euler_calib_status_packed &imu1EulerCalibration,
                 IMU2_euler_calib_status_packed &imu2EulerCalibration);

#endif