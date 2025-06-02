#include "functiondef.h"

BleGamepad bleGamepad;
BleGamepadConfiguration bleGamepadConfig;
Adafruit_LSM6DS3TRC lsm6ds1;
Adafruit_LSM6DS3TRC lsm6ds2;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LIS3MDL lis3mdl2;
Adafruit_Sensor_Calibration_EEPROM cal;
Adafruit_NXPSensorFusion fillsion1, fillsion2;
Adafruit_MAX17048 fuelGauge;

bool imuDataIMU1Ready = false;
bool imuDataIMU2Ready = false;

static IMU_config_data_anJoystick_Union configData;
static Overall_status_data_Union overallStatusDataUnion;
static IMU1_euler_calib_status_Union imu1EulerCalibration;
static IMU2_euler_calib_status_Union imu2EulerCalibration;
static Flex_sensor_data_Union flexSensorDataUnion;
static Force_sensor_data_Union forceSensorDataUnion;

unsigned long lastFlexSensorReadTime = 0;
unsigned long readFlexSensorInterval = 100;

void appsetup()
{

  Serial.begin(115200);
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  SystemInit();

  if (!restoreSettings(configData))
  {
    Serial.println("Failed to restore settings from EEPROM");
  }
  else
  {
    setDefaultSettings(configData);
  }

  if (!initFuelGauge(fuelGauge, overallStatusDataUnion))
  {
    Serial.println("Fuel gauge initialization failed");
  }

  if (!initIMU(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2, fillsion1, fillsion2, overallStatusDataUnion))
  {

    Serial.println("IMU initialization failed");
  }
  else
  {
    setupIMUDataRate(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2);
    setupIMUInterrupts(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2);
  }

  if (loadCalibration(cal, imu1EulerCalibration, imu2EulerCalibration))
  {

    Serial.println("Calibration loaded successfully");
  }
  else
  {
    Serial.println("Failed to load calibration");
  }

  setupBLEGamepad(bleGamepad, bleGamepadConfig);
  ledRGB();
  updateOverallStatusData(overallStatusDataUnion);
}

void appprocess()
{
  if (millis() - lastFlexSensorReadTime >= readFlexSensorInterval)
  {
    ReadAnnalogSensor(flexSensorDataUnion);
    lastFlexSensorReadTime = millis();
  }
}

/*
brief Calculate CRC16 checksum

 * This function calculates the CRC16 checksum for a given data buffer.
 * It uses the polynomial 0xA001 and initializes the CRC with 0xFFFF.
 * The function iterates through each byte of the data, updating the CRC accordingly.
 *
 * @param data Pointer to the data buffer
 * @param length Length of the data buffer
 * @return Calculated CRC16 checksum
*/
static uint16_t CRC16(const uint8_t *data, size_t length)
{

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/*
brief Restore settings from EEPROM

 * This function reads the settings from EEPROM and restores them into the provided configData structure.
 * It checks for a magic number to validate the data and calculates the CRC to ensure data integrity.
 * If the data is invalid or corrupted, it sets default settings.
 *
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure to store restored settings
 * @return true if settings were successfully restored, false otherwise
*/
bool restoreSettings(IMU_config_data_anJoystick_Union &configData)
{
  uint8_t buffer[sizeof(configData.rawData) + 4];

  for (size_t i = 0; i < sizeof(buffer); i++)
  {
    buffer[i] = EEPROM.read(SETTING_ADDR + i);
  }

  if (buffer[0] != SETTINGS_MAGIC1 || buffer[1] != SETTING_MAGIC2)
  {
    setDefaultSettings(configData);
    return false;
  }

  uint16_t storeCRC = (buffer[sizeof(configData.rawData) + 3] << 8) | buffer[sizeof(configData.rawData) + 2];
  uint16_t calulateCRC = CRC16(buffer, sizeof(configData.rawData) + 2);
  if (calulateCRC != storeCRC)
  {
    setDefaultSettings(configData);
    return false;
  }
  memcpy(&configData, buffer + 2, sizeof(configData.rawData));
  if (!validateSettings(configData))
  {
    setDefaultSettings(configData);
    return false;
  }

  return true;
}

/*
brief Save settings to EEPROM

 * This function saves the provided configData into EEPROM.
 * It first writes a magic number to identify the settings, then copies the raw data.
 * Finally, it calculates and appends a CRC16 checksum for data integrity.
 *
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure containing settings to save
*/
void saveSetting(IMU_config_data_anJoystick_Union &configData)
{

  uint8_t buffer[SETTING_SIZE + 4];
  buffer[0] = SETTINGS_MAGIC1;
  buffer[1] = SETTING_MAGIC2;

  memcpy(buffer + 2, configData.rawData, SETTING_SIZE);

  uint16_t crc = CRC16(buffer, SETTING_SIZE + 2);
  buffer[SETTING_SIZE + 2] = crc & 0xFF;
  buffer[SETTING_SIZE + 3] = (crc >> 8) & 0xFF;
  for (size_t i = 0; i < sizeof(buffer); i++)
  {
    EEPROM.write(SETTING_ADDR + i, buffer[i]);
  }
  EEPROM.commit();
}

bool validateSettings(IMU_config_data_anJoystick_Union &configData)
{
  if (configData.configDataIMUJOTISK.Joystick_flex_sensor_rate < 100 ||
      configData.configDataIMUJOTISK.Joystick_flex_sensor_rate > 1000)
  {
    return false;
  }
  return true;
}

/*
brief Set default settings for IMU and Joystick

 * This function initializes the configData structure with default settings for IMU and Joystick.
 * It sets the data rates, ranges, and other parameters to predefined values.
 * The settings are then saved to EEPROM.
 *
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure to set default settings
*/
void setDefaultSettings(IMU_config_data_anJoystick_Union &configData)
{

  configData.configDataIMUJOTISK.IMU1_accel_gyro_rate = LSM6DS_RATE_12_5_HZ;
  configData.configDataIMUJOTISK.IMU1_mag_freq = LIS3MDL_DATARATE_155_HZ;
  configData.configDataIMUJOTISK.IMU1_accel_range = LSM6DS_ACCEL_RANGE_2_G;
  configData.configDataIMUJOTISK.IMU1_gyro_range = LSM6DS_GYRO_RANGE_250_DPS;
  configData.configDataIMUJOTISK.IMU1_mag_range = LIS3MDL_RANGE_4_GAUSS;

  configData.configDataIMUJOTISK.IMU2_accel_gyro_freq = LSM6DS_RATE_12_5_HZ;
  configData.configDataIMUJOTISK.IMU2_mag_freq = LIS3MDL_DATARATE_155_HZ;
  configData.configDataIMUJOTISK.IMU2_accel_range = LSM6DS_ACCEL_RANGE_2_G;
  configData.configDataIMUJOTISK.IMU2_gyro_range = LSM6DS_GYRO_RANGE_250_DPS;
  configData.configDataIMUJOTISK.IMU2_mag_range = LIS3MDL_RANGE_4_GAUSS;

  configData.configDataIMUJOTISK.Joystick_flex_sensor_rate = 100;
  saveSetting(configData);
}

/*
brief Update the calibration status for IMU1 and IMU2

 * This function updates the calibration status in the provided IMU1_euler_calib_status_Union and IMU2_euler_calib_status_Union structures.
 * It sets the calibration status to 0x01 if calibration is successful, otherwise to 0x00.
 *
 * @param imu1Config Reference to the IMU1_euler_calib_status_Union structure
 * @param imu2Config Reference to the IMU2_euler_calib_status_Union structure
 * @param calibrationstatus Boolean indicating whether calibration was successful
*/
void IMU1updateCalibrationStatus(IMU1_euler_calib_status_Union &imu1Config, bool calibrationstatus)
{
  imu1Config.eulerCalibStatus.calibation = calibrationstatus ? 0x01 : 0x00;
}

void IMU2updateCalibrationStatus(IMU2_euler_calib_status_Union &imu2Config, bool calibrationstatus)
{
  imu2Config.eulerCalibStatus.calibation = calibrationstatus ? 0x01 : 0x00;
}

/*
brief Load calibration data from EEPROM

 * This function initializes the Adafruit_Sensor_Calibration_EEPROM object and attempts to load calibration data.
 * If successful, it updates the calibration status for both IMU1 and IMU2.
 * If initialization or loading fails, it sets the calibration status to false.
 *
 * @param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object
 * @param imu1EulerCalibration Reference to the IMU1_euler_calib_status_Union structure
 * @param imu2EulerCalibration Reference to the IMU2_euler_calib_status_Union structure
 * @return true if calibration was loaded successfully, false otherwise
*/
bool loadCalibration(Adafruit_Sensor_Calibration_EEPROM &cal, IMU1_euler_calib_status_Union &imu1EulerCalibration, IMU2_euler_calib_status_Union &imu2EulerCalibration)
{

  if (!cal.begin())
  {
    Serial.println("Failed to initialize calibration");
    IMU1updateCalibrationStatus(imu1EulerCalibration, false);
    IMU2updateCalibrationStatus(imu2EulerCalibration, false);
    return false;
  }
  if (!cal.loadCalibration())
  {

    IMU1updateCalibrationStatus(imu1EulerCalibration, false);
    IMU2updateCalibrationStatus(imu2EulerCalibration, false);
    return false;
  }

  IMU1updateCalibrationStatus(imu1EulerCalibration, true);
  IMU2updateCalibrationStatus(imu2EulerCalibration, true);
  return true;
}

/*
brief Setup BLE Gamepad

 * This function initializes the BLE Gamepad with the specified configuration.

*/
void setupBLEGamepad(BleGamepad &bleGamepad, BleGamepadConfiguration &bleGamepadConfig)
{
  Serial.println("Starting BLE work!");
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
  bleGamepadConfig.setVid(0xe502);
  bleGamepadConfig.setPid(0xabcd);

  bleGamepadConfig.setModelNumber(const_cast<char *>("ESP32-G1"));
  bleGamepadConfig.setSoftwareRevision(const_cast<char *>("v1.0.0"));
  bleGamepadConfig.setSerialNumber(const_cast<char *>("SN001"));
  bleGamepadConfig.setFirmwareRevision(const_cast<char *>("FW1.0"));
  bleGamepadConfig.setHardwareRevision(const_cast<char *>("HW1.0"));

  bleGamepadConfig.setAxesMin(0x0000);
  bleGamepadConfig.setAxesMax(0x7FFF);
  bleGamepad.begin(&bleGamepadConfig);
}

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, Overall_status_data_Union &overallStatusDatapPacked)
{
  if (!fuelGauge.begin())
  {
    Serial.println("Failed to find MAX17048 chip");
    overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::FAILED;
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR;
    return false;
  }
  Serial.println("Fuel gauge found");
  fuelGauge.setActivityThreshold(10);
  overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::RUNNING;
  return true;
}

/*
brief Update the overall status data

 * This function updates the overall status data structure with the current status of the fuel gauge and IMU sensors.
 * It sets the status code and sensor statuses based on their readiness.
*/

void updateOverallStatusData(Overall_status_data_Union &overallStatusDatapPacked)
{
  overallStatusDatapPacked.overallStatusData.status_code = statuscode::NO_ERROR;
  overallStatusDatapPacked.overallStatusData.Fuelgause_status = fuelGauge.isDeviceReady() ? statuscode_sensor::RUNNING : statuscode_sensor::FAILED;

  if (imuDataIMU1Ready && imuDataIMU2Ready)
  {
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
  }
  else
  {
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
  }
}

/*
brief Setup IMU interrupts
 * @param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
*/
void setupIMUInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl1, Adafruit_LIS3MDL &lis3mdl2)
{
  pinMode(IMU1_INT_PIN, INPUT);
  pinMode(IMU2_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1InterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU2_INT_PIN), imu1InterruptHandler, RISING);
  lsm6ds1.configInt1(false, false, true);
  lsm6ds2.configInt1(false, false, true);

  Serial.println("IMU interrupts configured");
}

/*
brief Interrupt handler for IMU
*/
void IRAM_ATTR imu1InterruptHandler()
{
  imuDataIMU1Ready = true;
}

void IRAM_ATTR imu2InterruptHandler()
{
  imuDataIMU2Ready = true;
}
