#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

#include "app.h"

void appsetup();
void appprocess();

void SystemInit();

bool restoreSettings(EEPROMDataCheckUnion &configData);

void saveSetting(EEPROMDataCheckUnion &configData);

void resetEEPROMDefaul(EEPROMDataCheckUnion &configData);


bool initIMU1(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              Adafruit_NXPSensorFusion &fusion,
              OverallStatusDataUnion &overallStatusDatapPacked);

bool initIMU2(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              Adafruit_NXPSensorFusion &fusion,
              OverallStatusDataUnion &overallStatusDatapPacked);

bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds1,
              Adafruit_LSM6DS3TRC &lsm6ds2,
              Adafruit_LIS3MDL &lis3mdl1,
              Adafruit_LIS3MDL &lis3mdl2,
              Adafruit_NXPSensorFusion &fillsion1,
              Adafruit_NXPSensorFusion &fillsion2,
              OverallStatusDataUnion &overallStatusDatapPacked);                

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

bool loadCalibration(Adafruit_Sensor_Calibration_EEPROM &calIMU1,
                     Adafruit_Sensor_Calibration_EEPROM &CalIMU2,
                     IMUEulernUnion imuEulerCalibration[NUM_IMUS]);

void setupBLEGamepad(BleGamepad &bleGamepad,
                     BleGamepadConfiguration &bleGamepadConfig);

void senDataBLE(BleGamepad &bleGamepad,
                JoystickDataUnion &joystickDataUnion,
                IMUDataRawUnion ImuArray[],
                IMUEulernUnion IMUeurle[]);

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, OverallStatusDataUnion &overallStatusDatapPacked);
void cycleRGBOnce();
void processMotor();

void updateOverallStatusData(OverallStatusDataUnion &overallStatusDatapPacked);

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LSM6DS3TRC &lsm6ds2,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_LIS3MDL &lis3mdl2,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fillsion1,
                 Adafruit_NXPSensorFusion &fillsion2,
                 IMUDataRawUnion ImuArray[NUM_IMUS],
                 IMUEulernUnion IMUeurle[NUM_IMUS]);

float mapFlexValue(int adcValue);
void ReadAnnalogSensor(FlexDataUnion &flexSensorDataUnion);
void checkbuttons(ButtonDataUnion &buttonDataUnion);
void readJoystick(JoystickDataUnion &joystickDataUnion);

#endif