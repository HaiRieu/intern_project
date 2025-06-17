#ifndef FUNCTIONDEF_H
#define FUNCTIONDEF_H

#include "app.h"

void appsetup();

void appprocess();

void systemInit();

bool restoreSettings(EEPROMDataCheckUnion &configData);

void saveSetting(EEPROMDataCheckUnion &configData);

void resetEEPROMDefaul(EEPROMDataCheckUnion &configData);

bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              uint8_t addressLsm6ds,
              uint8_t addressLis3mdl,
              OverallStatusDataUnion &overallDataUnion,
              Adafruit_NXPSensorFusion &fusionImu, uint32_t &timestamp);

void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl);

void setupImuInterrupts(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl, uint8_t pin, uint8_t imuId);

void IRAM_ATTR imuInterruptHandlerImu1();

void IRAM_ATTR imuInterruptHandlerImu2();

bool loadCalibrationImu(Adafruit_Sensor_Calibration_EEPROM &calIMU,
                        uint8_t addressEEPROMImu,
                        IMUEulernUnion &imuEulerCalibration);

void setupBLEGamepad(BleGamepad &bleGamepad,
                     BleGamepadConfiguration &bleGamepadConfig);

bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds,
                 Adafruit_LIS3MDL &lis3mdl,
                 uint8_t addressLsm6ds,
                 uint8_t addressLis3mdl,
                 OverallStatusDataUnion &overallDataUnion,
                 Adafruit_NXPSensorFusion &fusion,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 uint8_t addressEEPROMImu,
                 IMUEulernUnion &imuEulerCalibration,
                 uint8_t pinInterrup, uint8_t imuId, uint32_t &timestamp);


bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, OverallStatusDataUnion &overallStatusDatapPacked);

void cycleRGBOnce();

void processMotor();

void updateOverallStatusData(OverallStatusDataUnion &overallStatusDatapPacked);
void updateOverallStatus(BleGamepad &bleGamepad, OverallStatusDataUnion &overallStatusData, bool &imu1Status, bool &imu2Status, bool &fuelGauge);

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds,
                 Adafruit_LIS3MDL &lis3mdl,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fusion,
                 IMUDataRawUnion &imuData,
                 IMUEulernUnion &IMUeurle, BleGamepad &bleGamepad, bool &imuStatus, uint32_t &timestamp);

float mapFlexValue(int adcValue);

float mapForceValue(int adcValue);

void readFlexSensor(FlexDataUnion &flexSensorDataUnion, BleGamepad &bleGamepad);

void readForceSensor(ForceData &forceSensorData, BleGamepad &bleGamepad);

void checkbuttons(ButtonDataUnion &buttonDataUnion, BleGamepad &bleGamepad);

void readJoystick(JoystickDataUnion &joystickDataUnion);

void updateJoystickBle(BleGamepad &bleGamepad, JoystickDataUnion &joystickDataUnion);

void calibrateJoystick();

int16_t smoothJoystick(int16_t newValue, int16_t &preValue);

int16_t processJoystick(uint16_t adcValue, bool isXaxis);

void readBatteryData(BatteryData &batteryData, Adafruit_MAX17048 &maxlipo);

void upBatteryBLE(BatteryData &batteryData, BleGamepad &bleGamepad);

void IRAM_ATTR switchChangeISR();

void setPowerDown();

void processSwithChange();

void onwriteJoystic(EEPROMDataCheckUnion &joystickDataUnion, BleGamepad &bleGamepad);

bool onWriteconfig(EEPROMDataCheckUnion &configData, BleGamepad &bleGamepad,
                   ImuJoystickUnion &imuJoystickUnion);

void bleCalibration(ImuJoystickUnion &imuJoystickUnion,
                    IMUEulernUnion &imu1EulernUnion,
                    IMUEulernUnion &imu2EulernUnion,
                    BleGamepad &bleGamepad,
                    Adafruit_Sensor_Calibration_EEPROM &cal1,
                    Adafruit_Sensor_Calibration_EEPROM &cal2);

void updateLed(int r, int g, int b, int times, int delayTime);

void sendDataBLE(BleGamepad &bleGamepad,
                 IMUDataRawUnion &imuData,
                 IMUEulernUnion &IMUeurle,
                 bool isIMU);

void handleBLEConfig(EEPROMDataCheckUnion &configDataCheckUnion, BleGamepad &bleGamepad,
                     ImuJoystickUnion &imuJoystickUnion);


bool loadconfigEEProm(EEPROMDataCheckUnion &configData);                     

#endif