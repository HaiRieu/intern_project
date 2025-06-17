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

volatile bool imuDataIMUReady1 = false;
volatile bool imuDataIMUReady2 = false;
bool fugelGaugeStatus = false;
bool imu1Status = false;
bool imu2Status = false;

uint32_t timestamp1 = 0;
uint32_t timestamp2 = 0;

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

unsigned long lastIMUReadTime = 0;
unsigned long imuReadInterval = 100;

unsigned long lastFlexSensorReadTime = 0;
unsigned long readFlexSensorInterval = 100;

unsigned long lastJoysticReadTime = 0;
unsigned long JoystickReadInterval = 200;

unsigned long lastBatteryCheckTime = 0;
unsigned long batteryCheckInterval = 2000;

unsigned long lastOverallStatusUpdateTime = 0;
unsigned long overallStatusUpdateInterval = 100;

unsigned long lastOnwrite = 0;
unsigned long onwriteInterval = 200;


void appsetup()
{

  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  systemInit();
  if (!restoreSettings(configDataCheckUnion))
  {
    resetEEPROMDefaul(configDataCheckUnion);
  }
  else
  {
    Serial.println("to restore settings from EEPROM");
  }

  fugelGaugeStatus = initFuelGauge(fuelGauge, overallStatusDataUnion);
  Serial.println("Fuel gauge initialized");

  imu1Status = initIMU(lsm6ds1, lis3mdl1, ADDRESS_LSM6DS1, ADDRESS_LIS3MDL1,
                       overallStatusDataUnion, fusion1, calibrationImu1, IMU1_CAL_EEPROM_ADDR,
                       imuEulerCalibration1, IMU1_INT_PIN, IMU1ID, timestamp1);

  imu2Status = initIMU(lsm6ds2, lis3mdl2, ADDRESS_LSM6DS2, ADDRESS_LIS3MDL2,
                       overallStatusDataUnion, fusion2, calibrationImu2, IMU2_CAL_EEPROM_ADDR,
                       imuEulerCalibration2, IMU2_INT_PIN, IMU2ID, timestamp2);

  setupBLEGamepad(bleGamepad, bleGamepadConfig);

  processMotor();

  cycleRGBOnce();

  updateOverallStatusData(overallStatusDataUnion);
}

void appprocess()
{
  if (imuDataIMUReady1)
  {
    if (imu1Status)
    {
      readDataIMU(lsm6ds1, lis3mdl1, calibrationImu1, fusion1,
                  imuDataRawUnion1, imuEulerCalibration1, bleGamepad, imu1Status, timestamp1);
      sendDataBLE(bleGamepad, imuDataRawUnion1, imuEulerCalibration1, true);
    }
    imuDataIMUReady1 = false;
  }
  else if (imu1Status && (millis() - timestamp1 >= 200))
  {
    readDataIMU(lsm6ds1, lis3mdl1, calibrationImu1, fusion1,
                imuDataRawUnion1, imuEulerCalibration1, bleGamepad, imu1Status, timestamp1);
    sendDataBLE(bleGamepad, imuDataRawUnion1, imuEulerCalibration1, true);
  }

  if (imuDataIMUReady2)
  {
    if (imu2Status)
    {
      readDataIMU(lsm6ds2, lis3mdl2, calibrationImu2, fusion2,
                  imuDataRawUnion2, imuEulerCalibration2, bleGamepad, imu2Status, timestamp2);
      sendDataBLE(bleGamepad, imuDataRawUnion2, imuEulerCalibration2, false);
    }
    imuDataIMUReady2 = false;
  }
  else if (imu2Status && (millis() - timestamp2 >= 200))
  {
    readDataIMU(lsm6ds2, lis3mdl2, calibrationImu2, fusion2,
                imuDataRawUnion2, imuEulerCalibration2, bleGamepad, imu2Status, timestamp2);
    sendDataBLE(bleGamepad, imuDataRawUnion2, imuEulerCalibration2, false);
  }

  if (millis() - lastFlexSensorReadTime >= readFlexSensorInterval)
  {
    readFlexSensor(flexSensorDataUnion, bleGamepad);
    readForceSensor(forceSensorData, bleGamepad);
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

  if (millis() - lastOverallStatusUpdateTime >= overallStatusUpdateInterval)
  {
     updateOverallStatus(bleGamepad, overallStatusDataUnion, imu1Status, imu2Status,fugelGaugeStatus );
     lastOverallStatusUpdateTime = millis();
  }

  
  if (bleGamepad.isOnWriteConfig == 1)
  {
    onwriteJoystic(configDataCheckUnion, bleGamepad);
  //  bleGamepad.isOnWriteConfig = 0;
  }
/*
  if(bleGamepad.isConnected()) {
    bleGamepad.setterCharacterData(bleGamepad.OverallStatus, overallStatusDataUnion.rawData, sizeof(overallStatusDataUnion.rawData));
  }
  */


      onWriteconfig(configDataCheckUnion, bleGamepad, configData);
   

  // processSwithChange();
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

void saveSetting(EEPROMDataCheckUnion &configData)
{



  configData.EEPROMDataCheck.checksum1 = BYTE_CHECK1;
  configData.EEPROMDataCheck.checksum2 = BYTE_CHECK2;

  uint8_t buffer[EEPROM_TOTAL_SIZE];

  memcpy(buffer, configData.rawData, EEPROM_TOTAL_SIZE - 2);

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < EEPROM_TOTAL_SIZE - 2; i++)
  {
    crc = CRC16(crc, buffer[i]);
  }

  buffer[EEPROM_TOTAL_SIZE - 2] = crc & 0xFF;
  buffer[EEPROM_TOTAL_SIZE - 1] = (crc >> 8) & 0xFF;

  configData.EEPROMDataCheck.crc = crc;

  for (uint16_t i = 0; i < EEPROM_TOTAL_SIZE; i++)
  {
    EEPROM.write(SETTING_ADDR + i, buffer[i]);
  }
  EEPROM.commit();
}

bool restoreSettings(EEPROMDataCheckUnion &configData)
{

  uint8_t buffer[EEPROM_TOTAL_SIZE];

  for (size_t i = 0; i < EEPROM_TOTAL_SIZE; i++)
  {
    buffer[i] = EEPROM.read(SETTING_ADDR + i);
  }

  if (buffer[0] != BYTE_CHECK1 || buffer[1] != BYTE_CHECK2)
  {
    Serial.println("Invalid magic bytes");
    return false;
  }

  uint16_t calculatedCrc = 0xFFFF;
  for (size_t i = 0; i < EEPROM_TOTAL_SIZE - 2; i++)
  {
    calculatedCrc = CRC16(calculatedCrc, buffer[i]);
  }

  uint16_t storedCrc = (buffer[EEPROM_TOTAL_SIZE - 1] << 8) | buffer[EEPROM_TOTAL_SIZE - 2];

  if (calculatedCrc != storedCrc)
  {
    return false;
  }

  memcpy(configData.rawData, buffer, EEPROM_TOTAL_SIZE);

  if (configData.EEPROMDataCheck.JoystickFlexSensorRate < 100 ||
      configData.EEPROMDataCheck.JoystickFlexSensorRate > 99999)
  {
    return false;
  }
  return true;
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

  configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_52_HZ;
  configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_80_HZ;
  configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
  configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
  configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_4_GAUSS;

  configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_52_HZ;
  configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_80_HZ;
  configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
  configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
  configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_4_GAUSS;

  configData.EEPROMDataCheck.JoystickFlexSensorRate = 1000;
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
  overallStatusData.overallStatusData.Imu1Status = imu1Status ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;
  overallStatusData.overallStatusData.Imu2Status = imu2Status ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;
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
void setupImuInterrupts(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl, uint8_t pin, uint8_t imuId)
{
  pinMode(pin, INPUT);
  if (imuId == 1)
  {
    attachInterrupt(digitalPinToInterrupt(pin), imuInterruptHandlerImu1, LOW);
  }
  else if (imuId == 2)
  {
    attachInterrupt(digitalPinToInterrupt(pin), imuInterruptHandlerImu2, LOW);
  }
  lsm6ds.configInt1(false, false, true);
  lis3mdl.configInterrupt(true, true, true, false, false, true);
  Serial.println("IMU interrupts configured");
}

/*
@brief Interrupt handler for IMU
*/
void IRAM_ATTR imuInterruptHandlerImu1()
{

  imuDataIMUReady1 = true;
}

void IRAM_ATTR imuInterruptHandlerImu2()
{
  imuDataIMUReady2 = true;
}
