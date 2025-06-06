#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

#include "app.h"

void appsetup();

void appprocess();

void systemInit();

bool restoreSettings(EEPROMDataCheckUnion &configData);

void saveSetting(EEPROMDataCheckUnion &configData);

void resetEEPROMDefaul(EEPROMDataCheckUnion &configData);
   
bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds1,
               Adafruit_LIS3MDL &lis3mdl1,
               Adafruit_LSM6DS3TRC &lsm6ds2,
               Adafruit_LIS3MDL &lis3mdl2,
               uint8_t addressLsm6ds1,
               uint8_t addressLis3mdl1,
               uint8_t addressLsm6ds2,
               uint8_t addressLis3mdl2,
               OverallStatusDataUnion &overallDataUnion,
               Adafruit_NXPSensorFusion &fusionImu1,
               Adafruit_NXPSensorFusion &fusionImu2);
               
void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LIS3MDL &lis3mdl1,
                       Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl2);

void setupImuInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1,Adafruit_LIS3MDL &lis3mdl1,
                         Adafruit_LSM6DS3TRC &lsm6ds2,Adafruit_LIS3MDL &lis3mdl2);



void IRAM_ATTR imu1InterruptHandler();

void IRAM_ATTR imu2InterruptHandler();

bool loadCalibrationImu(Adafruit_Sensor_Calibration_EEPROM &calIMU1,
                        Adafruit_Sensor_Calibration_EEPROM &calIMU2,
                        uint8_t addressEEPROMImu1, uint8_t addressEEPROMimu2,
                        IMUEulernUnion &imu1EulerCalibration,
                        IMUEulernUnion &imu2EulerCalibration);

bool getImu1Status();

bool getImu2Status();

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

float mapForceValue(int adcValue);

void readFlexSensor(FlexDataUnion &flexSensorDataUnion);

void readForceSensor(ForceData &forceSensorData);

void checkbuttons(ButtonDataUnion &buttonDataUnion);

void readJoystick(JoystickDataUnion &joystickDataUnion);

#endif