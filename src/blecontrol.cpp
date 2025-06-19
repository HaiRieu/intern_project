
#include "functiondef.h"

unsigned long lastWriteTime = 0;
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
    bleGamepadConfig.setButtonCount(NUM_BUTTONS);
    // bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
    bleGamepadConfig.setVid(0xe502);
    bleGamepadConfig.setPid(0xabcd);

    bleGamepadConfig.setModelNumber((char *)MODEL_NUMBER);
    bleGamepadConfig.setFirmwareRevision((char *)FIRMWARE_VERSION);
    bleGamepadConfig.setHardwareRevision((char *)HARDWARE_REVISION);
    bleGamepad.deviceManufacturer = (char *)MANUFACTURER;

    bleGamepadConfig.setAxesMin(0x0000);
    bleGamepadConfig.setAxesMax(0x7FFF);
    bleGamepad.begin(&bleGamepadConfig);
}

/*
@brief Function to handle writing joystick data to BLE Gamepad and saving it to EEPROM.
@param joystickDataUnion Reference to the JoystickDataUnion containing joystick data
@param bleGamepad Reference to the BleGamepad object
This function retrieves joystick data from the BLE Gamepad, updates the joystickDataUnion with the current joystick flex sensor rate, and saves the updated data to EEPROM.
*/
void onwriteJoystic(EEPROMDataCheckUnion &joystickDataUnion, BleGamepad &bleGamepad)
{
    bleGamepad.getcharacterData(bleGamepad.Config, joystickDataUnion.rawData);
    uint16_t value = joystickDataUnion.EEPROMDataCheck.JoystickFlexSensorRate;
    saveSetting(joystickDataUnion);
}

/*
@brief Function to configure the IMU1 accelerometer and gyroscope rate based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU1 accelerometer and gyroscope
This function sets the IMU1 accelerometer and gyroscope rate based on the provided rate value. It maps the rate value to a specific LSM6DS rate constant.
*/
void configureIMUAccelGyroRate(EEPROMDataCheckUnion &configData, uint8_t rateValue, uint8_t idIMU)
{
    uint8_t value;
    switch (rateValue)
    {
    case 0:
        value = LSM6DS_RATE_SHUTDOWN;
        break;

    case 1:
        value = LSM6DS_RATE_12_5_HZ;
        break;

    case 2:
        value = LSM6DS_RATE_26_HZ;
        break;

    case 3:
        value = LSM6DS_RATE_52_HZ;
        break;

    case 4:
        value = LSM6DS_RATE_104_HZ;
        break;

    case 5:
        value = LSM6DS_RATE_208_HZ;
        break;

    case 6:
        value = LSM6DS_RATE_416_HZ;
        break;

    default:
        value = LSM6DS_RATE_52_HZ;
        break;
    }

    if (idIMU == 1)
    {
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = value;
    }
    else if (idIMU == 2)
    {
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = value;
    }
}

void configureImuMagFreqHz(EEPROMDataCheckUnion &configData, uint8_t rateValue, uint8_t idIMU)
{
    uint8_t value;
    switch (rateValue)
    {
    case 0:
        value = LIS3MDL_DATARATE_0_625_HZ;
        break;

    case 1:
        value = LIS3MDL_DATARATE_1_25_HZ;
        break;

    case 2:
        value = LIS3MDL_DATARATE_2_5_HZ;
        break;

    case 3:
        value = LIS3MDL_DATARATE_5_HZ;
        break;

    case 4:
        value = LIS3MDL_DATARATE_10_HZ;
        break;

    case 5:
        value = LIS3MDL_DATARATE_20_HZ;
        break;

    case 6:
        value = LIS3MDL_DATARATE_40_HZ;
        break;

    case 7:
        value = LIS3MDL_DATARATE_80_HZ;
        break;

    default:
        break;
    }

    if (idIMU == 1)
    {

        configData.EEPROMDataCheck.IMU1MagFreqHz = value;
    }
    else if (idIMU == 2)
    {

        configData.EEPROMDataCheck.IMU2MagFreqHz = value;
    }
}

void configureIMUMagRange(EEPROMDataCheckUnion &configData, uint8_t rangeValue, uint8_t idIMU)
{
    uint8_t value;
    switch (rangeValue)
    {
    case 0:
        value = LIS3MDL_RANGE_4_GAUSS;
        break;

    case 1:
        value = LIS3MDL_RANGE_8_GAUSS;
        break;

    case 2:
        value = LIS3MDL_RANGE_12_GAUSS;
        break;

    case 3:
        value = LIS3MDL_RANGE_16_GAUSS;
        break;

    default:
        break;
    }

    if (idIMU == 1)
    {
        configData.EEPROMDataCheck.IMU1MagRangeGaus = value;
    }
    else if (idIMU == 2)
    {
        configData.EEPROMDataCheck.IMU2MagRangeGauss = value;
    }
}

void configureIMUAccelRange(EEPROMDataCheckUnion &configData, uint8_t rangeValue, uint8_t idIMU)
{
    uint8_t value;

    switch (rangeValue)
    {
    case 0:
        value = LSM6DS_ACCEL_RANGE_2_G;
        break;

    case 2:
        value = LSM6DS_ACCEL_RANGE_4_G;
        break;

    case 3:
        value = LSM6DS_ACCEL_RANGE_8_G;
        break;

    case 1:
        value = LSM6DS_ACCEL_RANGE_16_G;
        break;

    default:
        break;
    }

    if (idIMU == 1)
    {

        configData.EEPROMDataCheck.IMU1AccelRangeG = value;
    }
    else if (idIMU == 2)
    {

        configData.EEPROMDataCheck.IMU2AccelRangeG = value;
    }
}

void configureIMUGyroRange(EEPROMDataCheckUnion &configData, uint8_t rangeValue, uint8_t idIMU)
{
    uint8_t value;
    switch (rangeValue)
    {
    case 0:
        value = LSM6DS_GYRO_RANGE_125_DPS;
        break;

    case 1:
        value = LSM6DS_GYRO_RANGE_250_DPS;
        break;

    case 2:
        value = LSM6DS_GYRO_RANGE_500_DPS;
        break;

    case 3:
        value = LSM6DS_GYRO_RANGE_1000_DPS;
        break;

    case 4:
        value = LSM6DS_GYRO_RANGE_2000_DPS;
        break;

    default:
        break;
    }

    if (idIMU == 1)
    {

        configData.EEPROMDataCheck.IMU1GyroRangeDps = value;
    }
    else if (idIMU == 2)
    {

        configData.EEPROMDataCheck.IMU2GyroRangeDps = value;
    }
}

uint8_t convertImuAccelGyroRate(uint8_t rate)
{
    switch (rate)
    {
    case LSM6DS_RATE_SHUTDOWN:
        return 0;
    case LSM6DS_RATE_12_5_HZ:
        return 1;

    case LSM6DS_RATE_26_HZ:
        return 2;

    case LSM6DS_RATE_52_HZ:
        return 3;

    case LSM6DS_RATE_104_HZ:
        return 4;

    case LSM6DS_RATE_208_HZ:
        return 5;

    case LSM6DS_RATE_416_HZ:
        return 6;

    default:
        return 0;
    }
}

uint8_t convertImuMagRate(uint8_t rate)
{
    switch (rate)
    {
    case LIS3MDL_DATARATE_0_625_HZ:
        return 0;
    case LIS3MDL_DATARATE_1_25_HZ:
        return 1;

    case LIS3MDL_DATARATE_2_5_HZ:
        return 2;

    case LIS3MDL_DATARATE_5_HZ:
        return 3;

    case LIS3MDL_DATARATE_10_HZ:
        return 4;

    case LIS3MDL_DATARATE_20_HZ:
        return 5;

    case LIS3MDL_DATARATE_40_HZ:
        return 6;

    case LIS3MDL_DATARATE_80_HZ:
        return 7;

    default:
        return 0;
    }
}

uint8_t convertImuMagRangeGaus(uint8_t range)
{
    switch (range)
    {
    case LIS3MDL_RANGE_4_GAUSS:
        return 0;
    case LIS3MDL_RANGE_8_GAUSS:
        return 1;

    case LIS3MDL_RANGE_12_GAUSS:
        return 2;

    case LIS3MDL_RANGE_16_GAUSS:
        return 3;

    default:
        return 0;
    }
}

uint8_t convertImuAccelRangeG(uint8_t range)
{
    switch (range)
    {
    case LSM6DS_ACCEL_RANGE_2_G:
        return 0;
    case LSM6DS_ACCEL_RANGE_4_G:
        return 2;

    case LSM6DS_ACCEL_RANGE_8_G:
        return 3;

    case LSM6DS_ACCEL_RANGE_16_G:
        return 1;

    default:
        return 0;
    }
}

uint8_t convertImuGyroRangeDps(uint8_t range)
{
    switch (range)
    {
    case LSM6DS_GYRO_RANGE_125_DPS:
        return 0;
    case LSM6DS_GYRO_RANGE_250_DPS:
        return 1;

    case LSM6DS_GYRO_RANGE_500_DPS:
        return 2;

    case LSM6DS_GYRO_RANGE_1000_DPS:
        return 3;

    case LSM6DS_GYRO_RANGE_2000_DPS:
        return 4;

    default:
        return 0;
    }
}

bool loadconfigEEProm(EEPROMDataCheckUnion &configData)
{
    if (restoreSettings(configData))
    {
        return true;
    }

    return false;
}

/*
@brief Function to handle writing configuration data to EEPROM and updating IMU settings.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param bleGamepad Reference to the BleGamepad object
@param imuJoystickUnion Reference to the ImuJoystickUnion containing IMU and joystick configuration data
@param lsm6ds1 Reference to the first Adafruit_LSM6DS3TRC IMU object
@param lis3mdl1 Reference to the first Adafruit_LIS3MDL IMU object
@param lsm6ds2 Reference to the second Adafruit_LSM6DS3TRC IMU object
@param lis3mdl2 Reference to the second Adafruit_LIS3MDL IMU object
This function checks if the BLE Gamepad is connected and if it is in read or write configuration mode. If in read mode, it loads the configuration from EEPROM and updates the IMU settings accordingly. If in write mode, it retrieves the configuration data from the BLE Gamepad, updates the EEPROM data, and saves it to EEPROM.
*/
bool onWriteconfig(EEPROMDataCheckUnion &configData, 
                   BleGamepad &bleGamepad, 
                   ImuJoystickUnion &imuJoystickUnion,
                   Adafruit_LSM6DS3TRC &lsm6ds1,
                   Adafruit_LIS3MDL &lis3mdl1,
                   Adafruit_LSM6DS3TRC &lsm6ds2,
                   Adafruit_LIS3MDL &lis3mdl2)
{
    if (!bleGamepad.isConnected())
        return false;

    if (bleGamepad.isOnReadConfig == 1)
    {
        bleGamepad.isOnReadConfig = 0;
        loadconfigEEProm(configData);

        imuJoystickUnion.configDataImuJOTISK.IMU1AccelYyroRateHz = convertImuAccelGyroRate(configData.EEPROMDataCheck.IMU1AccelYyroRateHz);
        imuJoystickUnion.configDataImuJOTISK.IMU1MagFreqHz = convertImuMagRate(configData.EEPROMDataCheck.IMU1MagFreqHz);
        imuJoystickUnion.configDataImuJOTISK.IMU1AccelRangeG = convertImuAccelRangeG(configData.EEPROMDataCheck.IMU1AccelRangeG);
        imuJoystickUnion.configDataImuJOTISK.IMU1MagRangeGaus = convertImuMagRangeGaus(configData.EEPROMDataCheck.IMU1MagRangeGaus);
        imuJoystickUnion.configDataImuJOTISK.IMU1GyroRangeDps = convertImuGyroRangeDps(configData.EEPROMDataCheck.IMU1GyroRangeDps);

        imuJoystickUnion.configDataImuJOTISK.IMU2AccelGyroFreqHz = convertImuAccelGyroRate(configData.EEPROMDataCheck.IMU2AccelGyroFreqHz);
        imuJoystickUnion.configDataImuJOTISK.IMU2MagFreqHz = convertImuMagRate(configData.EEPROMDataCheck.IMU2MagFreqHz);
        imuJoystickUnion.configDataImuJOTISK.IMU2AccelRangeG = convertImuAccelRangeG(configData.EEPROMDataCheck.IMU2AccelRangeG);
        imuJoystickUnion.configDataImuJOTISK.IMU2MagRangeGauss = convertImuMagRangeGaus(configData.EEPROMDataCheck.IMU2MagRangeGauss);
        imuJoystickUnion.configDataImuJOTISK.IMU2GyroRangeDps = convertImuGyroRangeDps(configData.EEPROMDataCheck.IMU2GyroRangeDps);
        imuJoystickUnion.configDataImuJOTISK.JoystickFlexSensorRate = configData.EEPROMDataCheck.JoystickFlexSensorRate;
        imuJoystickUnion.configDataImuJOTISK.CMD =  configData.EEPROMDataCheck.CMD; 

        bleGamepad.setterCharacterData(bleGamepad.Config, imuJoystickUnion.rawData, sizeof(imuJoystickUnion.rawData));
    }

    if (bleGamepad.isOnWriteConfig == 1 && bleGamepad.isRightSize == 1)
    {
        Serial.println("Writing configuration to EEPROM...");

        bleGamepad.isOnWriteConfig = 0;
        bleGamepad.isRightSize = 0;

        bleGamepad.getcharacterData(bleGamepad.Config, imuJoystickUnion.rawData);

        configureIMUAccelGyroRate(configData, imuJoystickUnion.configDataImuJOTISK.IMU1AccelYyroRateHz, 1);
        configureImuMagFreqHz(configData, imuJoystickUnion.configDataImuJOTISK.IMU1MagFreqHz, 1);
        configureIMUAccelRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU1AccelRangeG, 1);
        configureIMUMagRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU1MagRangeGaus, 1);
        configureIMUGyroRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU1GyroRangeDps, 1);

        configureIMUAccelGyroRate(configData, imuJoystickUnion.configDataImuJOTISK.IMU2AccelGyroFreqHz, 2);
        configureImuMagFreqHz(configData, imuJoystickUnion.configDataImuJOTISK.IMU2MagFreqHz, 2);
        configureIMUAccelRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU2AccelRangeG, 2);
        configureIMUMagRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU2MagRangeGauss, 2);
        configureIMUGyroRange(configData, imuJoystickUnion.configDataImuJOTISK.IMU2GyroRangeDps, 2);

        configData.EEPROMDataCheck.CMD = imuJoystickUnion.configDataImuJOTISK.CMD;
        configData.EEPROMDataCheck.JoystickFlexSensorRate = imuJoystickUnion.configDataImuJOTISK.JoystickFlexSensorRate;

        setupIMUDataRateConfig(lsm6ds1, lis3mdl1, configData, 1);
        setupIMUDataRateConfig(lsm6ds2, lis3mdl2, configData, 2);

        saveSetting(configData);
        return true;
    }

    return false;
}

/*
brief Sends IMU data over BLE if the gamepad is connected.
@param bleGamepad Reference to the BleGamepad object
@param imu1DataRawPacked Reference to the packed data structure for IMU 1
@param imu2DataRawPacked Reference to the packed data structure for IMU 2
@param imu1EulerCalibration Reference to the packed data structure for IMU 1 Euler calibration status
@param imu2EulerCalibration Reference to the packed data structure for IMU 2 Euler calibration status
*/
void sendDataBLE(BleGamepad &bleGamepad,
                 IMUDataRawUnion &imuData,
                 IMUEulernUnion &IMUeurle,
                 bool isIMU)
{
    if (bleGamepad.isConnected())
    {
        if (isIMU)
        {
            bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, imuData.rawData, sizeof(imuData.rawData));
            bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, IMUeurle.rawData, sizeof(IMUeurle.rawData));
      
        }
        else
        {
            bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, imuData.rawData, sizeof(imuData.rawData));
            bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, IMUeurle.rawData, sizeof(IMUeurle.rawData));
         
        }
    }
    else
    {
        Serial.println("BLE Gamepad is not connected, cannot send data.");
    }
}

void updateOverallStatus(BleGamepad &bleGamepad, OverallStatusDataUnion &overallStatusData, bool &imu1Status, bool &imu2Status, bool &fuelGauge)
{

    if (bleGamepad.isConnected())
    {
        overallStatusData.overallStatusData.statusode = statusCode::NO_ERROR;
        overallStatusData.overallStatusData.FuelgauseStatus = fuelGauge ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;
        overallStatusData.overallStatusData.Imu1Status = imu1Status ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;
        overallStatusData.overallStatusData.Imu2Status = imu2Status ? statusCodeSensor::RUNNING : statusCodeSensor::FAILED;
        bleGamepad.setterCharacterData(bleGamepad.OverallStatus, overallStatusData.rawData, sizeof(overallStatusData.rawData));
    }
}