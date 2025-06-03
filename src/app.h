
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

#define SETTINGS_MAGIC1 0x75 
#define SETTING_MAGIC2 0x54 
#define SETTING_ADDR 0x00
#define SETTING_SIZE sizeof(IMU_config_data_and_Joystick_Check)

#define  XAXIS_PIN 3 
#define  YAXIS_PIN 4
#define  JOYSTICK_BUTTON_PIN 5

#define FLEX_MIN 200 
#define FLEX_MAX 3500

#define EEPROM_TOTAL_SIZE (SETTING_SIZE + 4) 

#define FIRMWARE_VERSION "00.00.01"
#define MODEL_NUMBER "DegapVrGlove"
#define MANUFACTURER "NUS/Seamless"
#define HARDWARE_REVISION "00.00.01"



typedef struct __attribute__((packed)) {
   uint8_t IMU1_accel_gyro_rate;
  uint8_t IMU1_mag_freq;
  uint8_t IMU2_accel_gyro_freq;
  uint8_t IMU2_mag_freq;
  uint8_t IMU1_accel_range;
  uint8_t IMU1_gyro_range;
  uint8_t IMU1_mag_range;
  uint8_t IMU2_accel_range;
  uint8_t IMU2_gyro_range;
  uint8_t IMU2_mag_range;
  uint16_t Joystick_flex_sensor_rate;

} IMU_config_data_and_Joystick_Check;
typedef union __attribute__((packed)) {
  IMU_config_data_and_Joystick_Check configDataIMUJOTISK; // Configuration data for IMU and Joystick
  uint8_t rawData[12];
} IMU_config_data_and_Joystick_Check_Union;




/*
brief Structure to hold configuration data for IMU and Joystick
*/
typedef struct __attribute__((packed)) 
{
  uint8_t CMD ;
  uint8_t IMU1_accel_gyro_rate;
  uint8_t IMU1_mag_freq;
  uint8_t IMU2_accel_gyro_freq;
  uint8_t IMU2_mag_freq;
  uint8_t IMU1_accel_range;
  uint8_t IMU1_gyro_range;
  uint8_t IMU1_mag_range;
  uint8_t IMU2_accel_range;
  uint8_t IMU2_gyro_range;
  uint8_t IMU2_mag_range;
  uint16_t Joystick_flex_sensor_rate;
} IMU_config_data_and_Joystick;

typedef union __attribute__((packed))
{
  IMU_config_data_and_Joystick configDataIMUJOTISK;
  uint8_t rawData[13];
} IMU_config_data_and_Joystick_Union;


enum CMD_status_code {
  IDLE = 0 ,
  RUN = 1 , 
  StartCalibrationIMU1 = 2,
  StartCalibrationIMU2 = 3, 
};


/*
brief Structure to hold overall status of the system
*/
typedef struct __attribute__((packed)) Overall_status
{
  uint8_t status_code;      
  uint8_t Fuelgause_status; 
  uint8_t Imu1_status;     
  uint8_t Imu2_status;     

} Overall_status_data;

typedef union __attribute__((packed))
{
  Overall_status_data overallStatusData;
  uint8_t rawData[4];

}  Overall_status_data_Union;
 

enum statuscode_sensor
{
  NOT_DETECT = 0,
  FAILED = 1,
  IDEL = 2,
  RUNNING = 3

};

enum statuscode
{
  NO_ERROR = 0,
  GENERAL_ERROR = 1
};

/*
brief Structure to hold raw IMU data
*/
typedef struct __attribute__((packed))
{

  int16_t accelX_mg;
  int16_t accelY_mg;
  int16_t accelZ_mg;
  int16_t GyroX_rads;
  int16_t GyroY_rads;
  int16_t GyroZ_rads;
  int16_t MagX_uT;
  int16_t MagY_uT;
  int16_t MagZ_uT;

} IMU_data_Raw;

typedef union __attribute__((packed))
{
  IMU_data_Raw data_Imu;
  uint8_t rawData[18];
} IMU_data_Raw_packed;


/*
brief Structure to hold Euler angles and calibration status for IMU1
*/
typedef struct __attribute__((packed))
{

  float EulerYaw_deg;
  float EulerPitch_deg;
  float EulerRoll_deg;
  uint8_t calibation;

} IMU_euler_calib_status;

typedef union __attribute__((packed))
{
  IMU_euler_calib_status eulerCalibStatus;
  uint8_t rawData[13];

} IMU_euler_calib_status_Union;


/*
brief Structure to hold flex sensor data
*/

typedef struct __attribute__((packed))
{

 float Flex_Sensor_Data[NUM_FLEX] ; 

} Flex_sensor_data;

typedef union __attribute__((packed))
{
  Flex_sensor_data flexSensorData;
  uint8_t rawData[20];

} Flex_sensor_data_Union;


/*
brief Structure to hold force sensor data
*/
typedef struct __attribute__((packed)) {
  float Force_sensor_data_kOhm ;
} Force_sensor_data;

typedef union __attribute__((packed)) {
  Force_sensor_data forceSensorData;
  uint8_t rawData[4];
} Force_sensor_data_Union;



/*
brief Structure to hold button data for the gamepad
*/
typedef struct __attribute__((packed)) {
   
  uint8_t buttons[NUM_BUTTONS] ; 

} Button_data;

typedef union __attribute__((packed)) {
  Button_data buttonData;
  uint8_t rawData[4];
} Button_data_Union;


/*
brief Structure to hold joystick data
*/
typedef struct __attribute__((packed)) {
  int16_t Xaxis ; 
  int16_t Yaxis ;
  uint8_t joystickButton  ; 
}  Joystick_data;
typedef union __attribute__((packed)) {
  Joystick_data joystickData;
  uint8_t rawData[5];
} Joystick_data_Union;


