#include "functiondef.h"

BleGamepad bleGamepad;
BleGamepadConfiguration bleGamepadConfig;
Adafruit_LSM6DS3TRC lsm6ds1;
Adafruit_LSM6DS3TRC lsm6ds2;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LIS3MDL lis3mdl2;
Adafruit_Sensor_Calibration_EEPROM calibrationImu1, calibrationImu2;
Adafruit_NXPSensorFusion fusion1, fusion2;
Adafruit_MAX17048 fuelGauge;

bool imuDataIMU1Ready = false;
bool imuDataIMU2Ready = false;

static ImuJoystickUnion configData;
static EEPROMDataCheckUnion configDataCheckUnion;
static OverallStatusDataUnion overallStatusDataUnion;
static IMUEulernUnion imuEulerCalibration[NUM_IMUS];
static FlexDataUnion flexSensorDataUnion;
static ForceDataUnion forceSensorDataUnion;

unsigned long lastFlexSensorReadTime = 0;
unsigned long readFlexSensorInterval = 100;

unsigned long lastJoystickReadTime = 0;
unsigned long JoystickReadInterval = 50;

void appsetup()
{

  Serial.begin(115200);
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  SystemInit();

  if (!restoreSettings(configDataCheckUnion))
  {
    Serial.println("Failed to restore settings from EEPROM");
  }
  else
  {
    resetEEPROMDefaul(configDataCheckUnion);
  }

  if (!initFuelGauge(fuelGauge, overallStatusDataUnion))
  {
    Serial.println("Fuel gauge initialization failed");
  }

  if (imuStart(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2, fusion1, fusion2, overallStatusDataUnion))
  {
    Serial.println("IMU initialization failed");
    setupIMUDataRate(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2);
    setupIMUInterrupts(lsm6ds1, lsm6ds2, lis3mdl1, lis3mdl2);
  }

  if (loadCalibration(calibrationImu1, calibrationImu2, imuEulerCalibration))
  {
    Serial.println("Calibration loaded successfully");
  }
  setupBLEGamepad(bleGamepad, bleGamepadConfig);
  processMotor();
  cycleRGBOnce();
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
@brief Calculate CRC16 checksum
 * This function calculates the CRC16 checksum for a given byte.
 * It uses a polynomial division method to compute the checksum.
 * @param crc Initial CRC value
 * @param a Byte to calculate CRC for
 * @return Calculated CRC16 value
*/
uint16_t CRC16(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++)
  {
    if (crc & 1)
    {
      crc = (crc >> 1) ^ 0xA001;
    }
    else
    {
      crc = (crc >> 1);
    }
  }
  return crc;
}

/*
@brief Restore settings from EEPROM
 * This function reads the settings from EEPROM and restores them into the provided configData structure.
 * It checks for a magic number to validate the data and calculates the CRC to ensure data integrity.
 * If the data is invalid or corrupted, it sets default settings.
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure to store restored settings
 * @return true if settings were successfully restored, false otherwise
*/
bool restoreSettings(EEPROMDataCheckUnion &configData)
{
  uint8_t buffer[EEPROM_TOTAL_SIZE];

  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < sizeof(buffer); i++)
  {
    buffer[i] = EEPROM.read(SETTING_ADDR + i);
    crc = CRC16(crc, buffer[i]);
  }

  if (crc != 0 || buffer[0] != BYTE_CHECK1 || buffer[1] != BYTE_CHECK2)
  {
    return false;
  }
  memcpy(configData.rawData, buffer, sizeof(configData.rawData));

  if (configData.EEPROMDataCheck.JoystickFlexSensorRate < 100 ||
      configData.EEPROMDataCheck.JoystickFlexSensorRate > 1000)
  {
    return false;
  }

  return true;
}

/*
@brief Save settings to EEPROM
 * This function saves the provided configData into EEPROM.
 * It first writes a magic number to identify the settings, then copies the raw data.
 * Finally, it calculates and appends a CRC16 checksum for data integrity.
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure containing settings to save
*/
void saveSetting(EEPROMDataCheckUnion &configData)
{

  uint8_t buffer[EEPROM_TOTAL_SIZE];
  buffer[0] = BYTE_CHECK1;
  buffer[1] = BYTE_CHECK2;

  memcpy(buffer, configData.rawData, EEPROM_TOTAL_SIZE);

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < EEPROM_TOTAL_SIZE - 2; i++)
  {
    crc = CRC16(crc, buffer[i]);
  }

  buffer[EEPROM_TOTAL_SIZE - 2] = crc & 0xFF;
  buffer[EEPROM_TOTAL_SIZE - 1] = crc >> 8;

  for (uint16_t i = 0; i < EEPROM_TOTAL_SIZE; i++)
  {
    EEPROM.write(SETTING_ADDR + i, buffer[i]);
  }
  EEPROM.commit();
}

/*
@brief Set default settings for IMU and Joystick

 * This function initializes the configData structure with default settings for IMU and Joystick.
 * It sets the data rates, ranges, and other parameters to predefined values.
 * The settings are then saved to EEPROM.
 *
 * @param configData Reference to the IMU_config_data_anJoystick_Union structure to set default settings
*/
void resetEEPROMDefaul(EEPROMDataCheckUnion &configData)
{

  configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_104_HZ;
  configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_155_HZ;
  configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
  configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
  configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_4_GAUSS;

  configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_104_HZ;
  configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_155_HZ;
  configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
  configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
  configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_4_GAUSS;

  configData.EEPROMDataCheck.JoystickFlexSensorRate = 100;
  saveSetting(configData);
}

/*
@brief Update the calibration status for IMU1 and IMU2

 * This function updates the calibration status in the provided IMU1_euler_calib_status_Union and IMU2_euler_calib_status_Union structures.
 * It sets the calibration status to 0x01 if calibration is successful, otherwise to 0x00.
 *
 * @param imu1Config Reference to the IMU1_euler_calib_status_Union structure
 * @param imu2Config Reference to the IMU2_euler_calib_status_Union structure
 * @param calibrationstatus Boolean indicating whether calibration was successful
*/
void IMU1updateCalibrationStatus(IMUEulernUnion imuEulerCalibration[NUM_IMUS], bool calibrationstatus)
{
  imuEulerCalibration[0].eulerCalibStatus.calibation = calibrationstatus ? 0x01 : 0x00;
}

void IMU2updateCalibrationStatus(IMUEulernUnion imuEulerCalibration[NUM_IMUS], bool calibrationstatus)
{
  imuEulerCalibration[1].eulerCalibStatus.calibation = calibrationstatus ? 0x01 : 0x00;
}

/*
@brief Load calibration data from EEPROM

 * This function initializes the Adafruit_Sensor_Calibration_EEPROM object and attempts to load calibration data.
 * If successful, it updates the calibration status for both IMU1 and IMU2.
 * If initialization or loading fails, it sets the calibration status to false.
 *
 * @param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object
 * @param imu1EulerCalibration Reference to the IMU1_euler_calib_status_Union structure
 * @param imu2EulerCalibration Reference to the IMU2_euler_calib_status_Union structure
 * @return true if calibration was loaded successfully, false otherwise
*/
bool loadCalibration(Adafruit_Sensor_Calibration_EEPROM &calIMU1,
                     Adafruit_Sensor_Calibration_EEPROM &CalIMU2,
                     IMUEulernUnion imuEulerCalibration[NUM_IMUS])
{

  if (!calIMU1.begin(IMU1_CAL_EEPROM_ADDR))
  {
    Serial.println("Failed to initialize calibration");
    IMU1updateCalibrationStatus(imuEulerCalibration, false);
    return false;
  }

  if (!CalIMU2.begin(IMU2_CAL_EEPROM_ADDR))
  {
    Serial.println("Failed to initialize calibration for IMU2");
    IMU2updateCalibrationStatus(imuEulerCalibration, false);
    return false;
  }
  if (!calIMU1.loadCalibration())
  {

    IMU1updateCalibrationStatus(imuEulerCalibration, false);
    return false;
  }

  if (!CalIMU2.loadCalibration())
  {

    IMU2updateCalibrationStatus(imuEulerCalibration, false);
    return false;
  }

  IMU1updateCalibrationStatus(imuEulerCalibration, true);
  IMU2updateCalibrationStatus(imuEulerCalibration, true);
  return true;
}

bool initFuelGauge(Adafruit_MAX17048 &fuelGauge, OverallStatusDataUnion &overallStatusDatapPacked)
{
  if (!fuelGauge.begin())
  {
    Serial.println("Failed to find MAX17048 chip");
    overallStatusDatapPacked.overallStatusData.FuelgauseStatus = statusCodeSensor::FAILED;
    overallStatusDatapPacked.overallStatusData.statusode = statusCode::GENERAL_ERROR;
    return false;
  }
  Serial.println("Fuel gauge found");
  overallStatusDatapPacked.overallStatusData.FuelgauseStatus = statusCodeSensor::RUNNING;
  return true;
}

/*
@brief Update the overall status data

 * This function updates the overall status data structure with the current status of the fuel gauge and IMU sensors.
 * It sets the status code and sensor statuses based on their readiness.
*/

void updateOverallStatusData(OverallStatusDataUnion &overallStatusDatapPacked)
{
  overallStatusDatapPacked.overallStatusData.statusode = statusCode::NO_ERROR;
  overallStatusDatapPacked.overallStatusData.FuelgauseStatus = fuelGauge.isDeviceReady() ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;

  if (imuDataIMU1Ready && imuDataIMU2Ready)
  {
    overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    overallStatusDatapPacked.overallStatusData.Imu2Status = statusCodeSensor::RUNNING;
  }
  else
  {
    overallStatusDatapPacked.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
    overallStatusDatapPacked.overallStatusData.Imu2Status = statusCodeSensor::FAILED;
  }
}

/*
@brief Setup IMU interrupts
 * @param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
*/
void setupIMUInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl1, Adafruit_LIS3MDL &lis3mdl2)
{
  pinMode(IMU1_INT_PIN, INPUT);
  pinMode(IMU2_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1InterruptHandler, LOW);
  attachInterrupt(digitalPinToInterrupt(IMU2_INT_PIN), imu2InterruptHandler, LOW);
  lsm6ds1.configInt1(false, false, true);
  lsm6ds2.configInt1(false, false, true);

  Serial.println("IMU interrupts configured");
}

/*
@brief Interrupt handler for IMU
*/
void IRAM_ATTR imu1InterruptHandler()
{
  imuDataIMU1Ready = true;
}

void IRAM_ATTR imu2InterruptHandler()
{
  imuDataIMU2Ready = true;
}
