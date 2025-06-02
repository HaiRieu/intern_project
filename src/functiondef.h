#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

#include "app.h"

void appsetup();
void appprocess();

void SystemInit();

bool restoreSettings(IMU_config_data_anJoystick_Union &configData);

void saveSetting(IMU_config_data_anJoystick_Union &configData);

bool validateSettings(IMU_config_data_anJoystick_Union &configData);

void setDefaultSettings(IMU_config_data_anJoystick_Union &configData);

bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
             Adafruit_LSM6DS3TRC &lsm6ds2,
             Adafruit_LIS3MDL &lis3mdl1,
             Adafruit_LIS3MDL &lis3mdl2,
             Adafruit_NXPSensorFusion &fillsion1,
             Adafruit_NXPSensorFusion &fillsion2,
             Overall_status_data_Union &overallStatusDatapPacked);

void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds1,
                      Adafruit_LSM6DS3TRC &lsm6ds2,
                      Adafruit_LIS3MDL &lis3mdl1,
                      Adafruit_LIS3MDL &lis3mdl2);

void setupIMUInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1,
                        Adafruit_LSM6DS3TRC &lsm6ds2,
                        Adafruit_LIS3MDL &lis3mdl1,
                        Adafruit_LIS3MDL &lis3mdl2);

void IRAM_ATTR imu1InterruptHandler();

void IRAM_ATTR imu2InterruptHandler();

bool loadCalibration(Adafruit_Sensor_Calibration_EEPROM &cal, IMU1_euler_calib_status_Union &imu1EulerCalibration, IMU2_euler_calib_status_Union &imu2EulerCalibration);

void setupBLEGamepad(BleGamepad &bleGamepad,
                     BleGamepadConfiguration &bleGamepadConfig);


void senDataBLE(BleGamepad &bleGamepad,
                IMU_data_Raw_packed ImuArray[],
                IMU1_euler_calib_status_Union &imu1EulerCalibration,
                IMU2_euler_calib_status_Union &imu2EulerCalibration);                     

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, Overall_status_data_Union &overallStatusDatapPacked);
void ledRGB();

void updateOverallStatusData(Overall_status_data_Union &overallStatusDatapPacked);

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LSM6DS3TRC &lsm6ds2,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_LIS3MDL &lis3mdl2,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fillsion1,
                 Adafruit_NXPSensorFusion &fillsion2,
                 IMU_data_Raw_packed ImuArray[NUM_IMUS],
                 IMU1_euler_calib_status_Union &imu1EulerCalibration,
                 IMU2_euler_calib_status_Union &imu2EulerCalibration);



float mapFlexValue(int adcValue);                 
void ReadAnnalogSensor(Flex_sensor_data_Union &flexSensorDataUnion);
void checkbuttons(Button_data_Union &buttonDataUnion) ; 
void readJoystick(Joystick_data_Union &joystickDataUnion) ; 

#endif