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
                IMUDataRawUnion &imuData1,
                IMUDataRawUnion &imuData2,
                IMUEulernUnion &IMU1eurle,
                IMUEulernUnion &IMU2eurle);

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, OverallStatusDataUnion &overallStatusDatapPacked);

void cycleRGBOnce();

void processMotor();

void updateOverallStatusData(OverallStatusDataUnion &overallStatusDatapPacked);

void readDataIMU2(Adafruit_LSM6DS3TRC &lsm6ds2,
                  Adafruit_LIS3MDL &lis3mdl2,
                  Adafruit_Sensor_Calibration_EEPROM &cal2,
                  Adafruit_NXPSensorFusion &fusion2,
                  IMUDataRawUnion &imuData,
                  IMUEulernUnion &IMUeurle2);

void readDataIMU1(Adafruit_LSM6DS3TRC &lsm6ds1,
                 Adafruit_LIS3MDL &lis3mdl1,
                 Adafruit_Sensor_Calibration_EEPROM &cal1,
                 Adafruit_NXPSensorFusion &fusion1,
                 IMUDataRawUnion &imuData,
                 IMUEulernUnion &IMUeurle1);

float mapFlexValue(int adcValue);

float mapForceValue(int adcValue);

void readFlexSensor(FlexDataUnion &flexSensorDataUnion);

void readForceSensor(ForceData &forceSensorData);

void checkbuttons(ButtonDataUnion &buttonDataUnion , BleGamepad &bleGamepad);

void readJoystick(JoystickDataUnion &joystickDataUnion);

void updateJoystickBle(BleGamepad &bleGamepad, JoystickDataUnion &joystickDataUnion) ;

void calibrateJoystick() ; 

int16_t smoothJoystick(int16_t newValue, int16_t &preValue) ; 

int16_t processJoystick(uint16_t adcValue, bool isXaxis) ; 

void readBatteryData(BatteryData &batteryData, Adafruit_MAX17048 &maxlipo);

void upBatteryBLE(BatteryData &batteryData, BleGamepad &bleGamepad); 

void IRAM_ATTR switchChangeISR() ; 

void setPowerDown() ;

void processSwithChange();

void onwriteJoystic(EEPROMDataCheckUnion &joystickDataUnion, BleGamepad &bleGamepad);

void onWrieconfig(EEPROMDataCheckUnion &configData, ImuJoystickUnion &imuJoystickUnion, BleGamepad &bleGamepad);

void bleCalibration(ImuJoystickUnion &imuJoystickUnion, 
                    IMUEulernUnion &imu1EulernUnion,
                    IMUEulernUnion &imu2EulernUnion, 
                    BleGamepad &bleGamepad,
                    Adafruit_Sensor_Calibration_EEPROM &cal1, 
                    Adafruit_Sensor_Calibration_EEPROM &cal2);

#endif