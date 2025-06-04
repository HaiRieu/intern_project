
#pragma once

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>
#include <Adafruit_MAX1704X.h>
#include <BleGamepad.h>
#include <stdint.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Calibration.h>


// Define constants for the application
#define FIRMWARE_VERSION "00.00.01"
#define MODEL_NUMBER "DegapVrGlove"
#define MANUFACTURER "NUS/Seamless"
#define HARDWARE_REVISION "00.00.01"
#define IMU1_INT_PIN 40
#define IMU2_INT_PIN 38
#define I2C_SDA_PIN 34
#define I2C_SCL_PIN 33
#define VSVY_EN_PIN 18
#define LED_GREEN_PIN 37
#define LED_RED_PIN 35
#define LED_BLUE_PIN 36
#define MOTOR_EN_PIN 13
#define NUM_FLEX 5
#define FORECE_SENSOR_PIN 11
#define FILTER_UPDATE_RATE_HZ 100
#define NUM_IMUS 2 
#define NUM_BUTTONS 4
#define BYTE_CHECK1 0x75 
#define BYTE_CHECK2 0x54 
#define SETTING_ADDR 0x00
#define IMU1_CAL_EEPROM_ADDR  0x50   
#define IMU2_CAL_EEPROM_ADDR  0x51  
#define EEPROM_TOTAL_SIZE (sizeof(IMUAndJoystickCheck))
#define XAXIS_PIN 3 
#define YAXIS_PIN 4
#define JOYSTICK_BUTTON_PIN 5
#define FLEX_MIN 200 
#define FLEX_MAX 3500 
#define FORCE_MIN 
#define FORCE_MAX 
#define FG_CHECK_INTERVAL 1000 


enum CMDStatusCode {
  IDLE = 0 ,
  RUN = 1 , 
  StartCalibrationIMU1 = 2,
  StartCalibrationIMU2 = 3, 
};


enum BatteryChargeStatus {
  BATTERY_CHARGE_STATUS_NOT_CHARGING =  0,
  BATTERY_CHARGE_STATUS_CHARGING = 1,
  BATTERY_CHARGE_STATUS_FULLY_CHARGED = 2
};


enum statusCodeSensor
{
  NOT_DETECT = 0,
  FAILED = 1,
  IDEL = 2,
  RUNNING = 3

};


enum statusCode
{
  NO_ERROR = 0,
  GENERAL_ERROR = 1
};


typedef struct __attribute__((packed)) {
  uint8_t checksum1; 
  uint8_t checksum2; 
  uint8_t IMU1AccelYyroRateHz;
  uint8_t IMU1MagFreqHz;
  uint8_t IMU2AccelGyroFreqHz;
  uint8_t IMU2MagFreqHz;
  uint8_t IMU1AccelRangeG;
  uint8_t IMU1GyroRangeDps;
  uint8_t IMU1MagRangeGaus;
  uint8_t IMU2AccelRangeG;
  uint8_t IMU2GyroRangeDps;
  uint8_t IMU2MagRangeGauss;
  uint16_t JoystickFlexSensorRate;
  uint16_t crc;

  
} IMUAndJoystickCheck;
typedef union __attribute__((packed)) {
  IMUAndJoystickCheck EEPROMDataCheck;
  uint8_t rawData[16];
} EEPROMDataCheckUnion;


/*
brief Structure to hold configuration data for IMU and Joystick
*/
typedef struct __attribute__((packed)) 
{
  uint8_t CMD ;
  uint8_t IMU1AccelYyroRateHz;
  uint8_t IMU1MagFreqHz;
  uint8_t IMU2AccelGyroFreqHz;
  uint8_t IMU2MagFreqHz;
  uint8_t IMU1AccelRangeG;
  uint8_t IMU1GyroRangeDps;
  uint8_t IMU1MagRangeGaus;
  uint8_t IMU2AccelRangeG;
  uint8_t IMU2GyroRangeDps;
  uint8_t IMU2MagRangeGauss;
  uint16_t JoystickFlexSensorRate;
} IMUAndJoystickConfig;

typedef union __attribute__((packed))
{
  IMUAndJoystickConfig configDataImuJOTISK;
  uint8_t rawData[13];
} ImuJoystickUnion;


/*
brief Structure to hold overall status of the system
*/
typedef struct __attribute__((packed)) Overall_status
{
  statusCode statusode;      
  statusCodeSensor FuelgauseStatus; 
  statusCodeSensor Imu1Status;     
  statusCodeSensor Imu2Status;     

} Overall_status_data;

typedef union __attribute__((packed))
{
  Overall_status_data overallStatusData;
  uint8_t rawData[4];

}  OverallStatusDataUnion;
 
/*
brief Structure to hold raw IMU data
*/
typedef struct __attribute__((packed))
{

  int16_t accelXmg;
  int16_t accelYmg;
  int16_t accelZmg;
  int16_t GyroXrads;
  int16_t GyroYrads;
  int16_t GyroZrads;
  int16_t MagXuT;
  int16_t MagYuT;
  int16_t MagZuT;
 
} IMUDataRaw;

typedef union __attribute__((packed))
{
  IMUDataRaw dataImu;
  uint8_t rawData[18];
} IMUDataRawUnion;


/*
brief Structure to hold Euler angles and calibration status for IMU1
*/
typedef struct __attribute__((packed))
{

  float EulerYawdeg;
  float EulerPitchdeg;
  float EulerRolldeg;
  uint8_t calibation;

} IMUEulerCalibStatus;

typedef union __attribute__((packed))
{
  IMUEulerCalibStatus eulerCalibStatus;
  uint8_t rawData[13];

} IMUEulernUnion;

/*
brief Structure to hold flex sensor data
*/

typedef struct __attribute__((packed))
{

 float FlexSensorData[NUM_FLEX] ; 

} FlexData;

typedef union __attribute__((packed))
{
  FlexData flexSensorData;
  uint8_t rawData[20];

} FlexDataUnion;

/*
brief Structure to hold force sensor data
*/
typedef struct __attribute__((packed)) {
  float ForceDataKOhm ;
} ForceData;

typedef union __attribute__((packed)) {
  ForceData forceData;
  uint8_t rawData[4];
} ForceDataUnion;

/*
brief Structure to hold button data for the gamepad
*/
typedef struct __attribute__((packed)) {
   
  uint8_t buttons[NUM_BUTTONS] ; 

} ButtonData;

typedef union __attribute__((packed)) {
  ButtonData buttonData;
  uint8_t rawData[4];
} ButtonDataUnion;

/*
brief Structure to hold joystick data
*/
typedef struct __attribute__((packed)) {
  int16_t Xaxis ; 
  int16_t Yaxis ;
  uint8_t joystickButton  ; 
}  JoystickData;
typedef union __attribute__((packed)) {
  JoystickData joystickData;
  uint8_t rawData[5];
} JoystickDataUnion;


typedef struct __attribute__((packed)){
  uint8_t batteryLevel;
  BatteryChargeStatus batteryChargeStatus;  
} BatteryData;




