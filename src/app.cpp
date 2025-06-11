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
static IMUEulernUnion imuEulerCalibration1, imuEulerCalibration2;
static FlexDataUnion flexSensorDataUnion;
static ForceData forceSensorData;
static IMUDataRawUnion imuDataRawUnion1, imuDataRawUnion2;
static ButtonDataUnion buttonDataUnion;
static JoystickDataUnion joystickDataUnion;
static BatteryData batteryData;

unsigned long lastFlexSensorReadTime = 0;
unsigned long readFlexSensorInterval = 100;

unsigned long lastJoysticReadTime = 0;
unsigned long JoystickReadInterval = 50;

unsigned long lastBatteryCheckTime = 0;
unsigned long batteryCheckInterval = 1000;

void appsetup()
{

  Serial.begin(115200);
  Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
  systemInit();

  if (!restoreSettings(configDataCheckUnion))
  {
    resetEEPROMDefaul(configDataCheckUnion);
  }
  else
  {
    Serial.println("to restore settings from EEPROM");
  }

  if (!initFuelGauge(fuelGauge, overallStatusDataUnion))
  {
    Serial.println("Fuel gauge initialization failed");
  }

  if (imuStart(lsm6ds1, lis3mdl1, lsm6ds2, lis3mdl2,
               ADDRESS_LSM6DS1, ADDRESS_LIS3MDL1, ADDRESS_LSM6DS2, ADDRESS_LIS3MDL2,
               overallStatusDataUnion, fusion1, fusion2))
  {
    Serial.println("IMU initialized successfully");
    setupIMUDataRate(lsm6ds1, lis3mdl1, lsm6ds2, lis3mdl2);
    loadCalibrationImu(calibrationImu1, calibrationImu2, IMU1_CAL_EEPROM_ADDR, IMU2_CAL_EEPROM_ADDR,
                       imuEulerCalibration1, imuEulerCalibration2);
    setupImuInterrupts(lsm6ds1, lis3mdl1, lsm6ds2, lis3mdl2);
  }

  setupBLEGamepad(bleGamepad, bleGamepadConfig);
  processMotor();
  cycleRGBOnce();
  updateOverallStatusData(overallStatusDataUnion);
}

void appprocess()
{

  if (getImu1Status() || getImu2Status())
  {

    if (imuDataIMU1Ready )
    {
      readDataIMU1(lsm6ds1, lis3mdl1, calibrationImu1, fusion1, imuDataRawUnion1, imuEulerCalibration1);
      imuDataIMU1Ready = false;
    
    }

    if (imuDataIMU2Ready)
    {

      readDataIMU2(lsm6ds2, lis3mdl2, calibrationImu2, fusion2, imuDataRawUnion2, imuEulerCalibration2);
      imuDataIMU2Ready = false;
   
    }
    else
    {

      readDataIMU1(lsm6ds1, lis3mdl1, calibrationImu1, fusion1, imuDataRawUnion1, imuEulerCalibration1);
      readDataIMU2(lsm6ds2, lis3mdl2, calibrationImu2, fusion2, imuDataRawUnion2, imuEulerCalibration2);
    }
    senDataBLE(bleGamepad, joystickDataUnion, imuDataRawUnion1, imuDataRawUnion2,
               imuEulerCalibration1, imuEulerCalibration2);
    if (bleGamepad.isConnected())
    {
      bleCalibration(configData, imuEulerCalibration1,
                     imuEulerCalibration2, bleGamepad, calibrationImu1, calibrationImu2);
    }
  }

  if (millis() - lastFlexSensorReadTime >= readFlexSensorInterval)
  {
    readFlexSensor(flexSensorDataUnion);
    readForceSensor(forceSensorData);
    checkbuttons(buttonDataUnion, bleGamepad);
    lastFlexSensorReadTime = millis();
  }

  if (millis() - lastJoysticReadTime >= JoystickReadInterval)
  {
    readJoystick(joystickDataUnion);
    updateJoystickBle(bleGamepad, joystickDataUnion);
    lastJoysticReadTime = millis();
  }

  if (millis() - lastBatteryCheckTime >= batteryCheckInterval)
  {
    readBatteryData(batteryData, fuelGauge);
    upBatteryBLE(batteryData, bleGamepad);
    lastBatteryCheckTime = millis();
  }
  if (bleGamepad.isOnWriteConfig && bleGamepad.isRightSize)
  {
    onwriteJoystic(configDataCheckUnion, bleGamepad);
  }

  if (bleGamepad.isOnWriteConfig && bleGamepad.isRightSize)
  {
    onWrieconfig(configDataCheckUnion, configData, bleGamepad);
  }

  processSwithChange();
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
      configData.EEPROMDataCheck.JoystickFlexSensorRate > 99999)
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
@brief Initialize the fuel gauge (MAX17048)

 * This function initializes the MAX17048 fuel gauge sensor.
 * It checks if the sensor is present and ready, and updates the overall status data accordingly.
 * If the sensor initialization fails, it sets the status code to indicate a general error.
 *
 * @param fuelGauge Reference to the Adafruit_MAX17048 object for the fuel gauge
 * @param overallStatusDatapPacked Reference to the OverallStatusDataUnion structure to update status
 * @return true if the fuel gauge is initialized successfully, false otherwise
*/
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

void updateOverallStatusData(OverallStatusDataUnion &overallStatusData)
{
  overallStatusData.overallStatusData.statusode = statusCode::NO_ERROR;
  overallStatusData.overallStatusData.FuelgauseStatus = fuelGauge.isDeviceReady() ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;

  if (imuDataIMU1Ready && imuDataIMU2Ready)
  {
    overallStatusData.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    overallStatusData.overallStatusData.Imu2Status = statusCodeSensor::RUNNING;
  }
  else
  {
    overallStatusData.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
    overallStatusData.overallStatusData.Imu2Status = statusCodeSensor::FAILED;
  }
}

/*
@brief Setup interrupts for IMU1 and IMU2

 * This function configures the interrupt pins for IMU1 and IMU2.
 * It sets the pin modes, attaches interrupt handlers, and configures the interrupt settings for both IMUs.
 * If the IMUs are not enabled, it does not configure their interrupts.
 *
 * @param lsm6ds1 Reference to the first LSM6DS3TRC sensor
 * @param lis3mdl1 Reference to the first LIS3MDL sensor
 * @param lsm6ds2 Reference to the second LSM6DS3TRC sensor
 * @param lis3mdl2 Reference to the second LIS3MDL sensor
*/
void setupImuInterrupts(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LIS3MDL &lis3mdl1,
                        Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl2)
{

  if (getImu1Status())
  {

    pinMode(IMU1_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1InterruptHandler, LOW);
    lsm6ds1.configInt1(false, false, true);
    lis3mdl1.configInterrupt(true, true, true, false, false, true);
    Serial.println("IMU1 interrupts configured");
  }

  if (getImu2Status())
  {

    pinMode(IMU2_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU2_INT_PIN), imu2InterruptHandler, LOW);
    lsm6ds2.configInt1(false, false, true);
    lis3mdl2.configInterrupt(true, true, true, false, false, true);
    Serial.println("IMU2 interrupts configured");
  }
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
