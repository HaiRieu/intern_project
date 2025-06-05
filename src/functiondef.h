#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

#include "app.h"

void appsetup();
void appprocess();

void systemInit();

bool restoreSettings(EEPROMDataCheckUnion &configData);

void saveSetting(EEPROMDataCheckUnion &configData);

void resetEEPROMDefaul(EEPROMDataCheckUnion &configData);


bool imu1Start(Adafruit_LSM6DS3TRC &lsm6ds,
                  Adafruit_LIS3MDL &lis3mdl,
                  uint8_t addressLsm6ds,
                  uint8_t addressLis3mdl,
                  OverallStatusDataUnion &overallDataUnion,
                  Adafruit_NXPSensorFusion &fusionImu);     
                  
bool imu2Start(Adafruit_LSM6DS3TRC &lsm6ds,
                  Adafruit_LIS3MDL &lis3mdl,
                  uint8_t addressLsm6ds,
                  uint8_t addressLis3mdl,
                  OverallStatusDataUnion &overallDataUnion,
                  Adafruit_NXPSensorFusion &fusionImu);                    
             
void setupIMU1DataRate(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LIS3MDL &lis3mdl1);
void setupIMU2DataRate(Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl2);

void setupIMU1Interrupts(Adafruit_LSM6DS3TRC &lsm6ds1,Adafruit_LIS3MDL &lis3mdl1);
void setupIMU2Interrupts(Adafruit_LSM6DS3TRC &lsm6ds2,Adafruit_LIS3MDL &lis3mdl2);

void IRAM_ATTR imu1InterruptHandler();

void IRAM_ATTR imu2InterruptHandler();

bool loadCalibrationImu1(Adafruit_Sensor_Calibration_EEPROM &calIMU,
                     uint8_t addressEEPROM ,
                     IMUEulernUnion &imuEulerCalibration);

bool loadCalibrationImu2(Adafruit_Sensor_Calibration_EEPROM &calIMU,
                     uint8_t addressEEPROM ,
                     IMUEulernUnion imuEulerCalibration);

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
                 Adafruit_Sensor_Calibration_EEPROM &cal1,
                 Adafruit_Sensor_Calibration_EEPROM &cal2,   
                 Adafruit_NXPSensorFusion &fusion1,
                 Adafruit_NXPSensorFusion &fusion2,
                 IMUDataRawUnion ImuArray[NUM_IMUS],
                 IMUEulernUnion &IMUeurle1,
                 IMUEulernUnion &IMUeurle2);

float mapFlexValue(int adcValue);
void readAnnalogSensor(FlexDataUnion &flexSensorDataUnion);
void checkbuttons(ButtonDataUnion &buttonDataUnion);
void readJoystick(JoystickDataUnion &joystickDataUnion);

#endif