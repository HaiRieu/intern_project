
#include "functiondef.h"




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
void configureIMU1AccelGyroRate(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_SHUTDOWN;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_12_5_HZ;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_26_HZ;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_52_HZ;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_104_HZ;
        break;

    case 5:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_208_HZ;
        break;

    case 6:
        configData.EEPROMDataCheck.IMU1AccelYyroRateHz = LSM6DS_RATE_416_HZ;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU1 magnetometer frequency based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU1 magnetometer frequency
This function sets the IMU1 magnetometer frequency based on the provided rate value. It maps the rate value to a specific LIS3MDL data rate constant.
*/
void configureImu1MagFreqHz(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_0_625_HZ;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_1_25_HZ;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_2_5_HZ;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_5_HZ;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_10_HZ;
        break;

    case 5:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_20_HZ;
        break;

    case 6:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_40_HZ;
        break;

    case 7:
        configData.EEPROMDataCheck.IMU1MagFreqHz = LIS3MDL_DATARATE_80_HZ;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU1 accelerometer range in G based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU1 accelerometer range in G
This function sets the IMU1 accelerometer range in G based on the provided rate value. It maps the rate value to a specific LSM6DS accelerometer range constant.
*/
void configureImu1AccelRangeG(EEPROMDataCheckUnion &configData, uint8_t rateValue) {
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_4_G;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_8_G;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU1AccelRangeG = LSM6DS_ACCEL_RANGE_16_G;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU1 gyroscope range in degrees per second (dps) based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU1 gyroscope range in dps
This function sets the IMU1 gyroscope range in dps based on the provided rate value. It maps the rate value to a specific LSM6DS gyroscope range constant.
*/
void configureImu1GyroRangeDps(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_125_DPS;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_500_DPS;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_1000_DPS;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU1GyroRangeDps = LSM6DS_GYRO_RANGE_2000_DPS;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU1 magnetometer range in Gauss based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU1 magnetometer range in Gauss
This function sets the IMU1 magnetometer range in Gauss based on the provided rate value. It maps the rate value to a specific LIS3MDL range constant.
*/
void  configureImu1MagRangeGaus(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_4_GAUSS;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_8_GAUSS;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_12_GAUSS;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU1MagRangeGaus = LIS3MDL_RANGE_16_GAUSS;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU2 accelerometer and gyroscope rate based on the provided rate value.
*/
void configureIMU2AccelGyroRate(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_SHUTDOWN;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_12_5_HZ;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_26_HZ;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_52_HZ;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_104_HZ;
        break;

    case 5:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_208_HZ;
        break;

    case 6:
        configData.EEPROMDataCheck.IMU2AccelGyroFreqHz = LSM6DS_RATE_416_HZ;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU2 magnetometer frequency based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU2 magnetometer frequency
This function sets the IMU2 magnetometer frequency based on the provided rate value. It maps the rate value to a specific LIS3MDL data rate constant.
*/
void configureImu2MagFreqHz(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_0_625_HZ;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_1_25_HZ;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_2_5_HZ;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_5_HZ;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_10_HZ;
        break;

    case 5:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_20_HZ;
        break;

    case 6:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_40_HZ;
        break;

    case 7:
        configData.EEPROMDataCheck.IMU2MagFreqHz = LIS3MDL_DATARATE_80_HZ;
        break;

    default:
        break;
    }
}


/*
@brief Function to configure the IMU2 accelerometer range in G based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU2 accelerometer range in G
This function sets the IMU2 accelerometer range in G based on the provided rate value. It maps the rate value to a specific LSM6DS accelerometer range constant.
*/
void configureImu2AccelRangeG(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_2_G;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_4_G;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_8_G;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU2AccelRangeG = LSM6DS_ACCEL_RANGE_16_G;
        break;

    default:
        break;
    }
}

/*
@brief Function to configure the IMU2 gyroscope range in degrees per second (dps) based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU2 gyroscope range in dps
This function sets the IMU2 gyroscope range in dps based on the provided rate value. It maps the rate value to a specific LSM6DS gyroscope range constant.
*/

void configureImu2GyroRangeDps(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_125_DPS;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_250_DPS;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_500_DPS;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_1000_DPS;
        break;

    case 4:
        configData.EEPROMDataCheck.IMU2GyroRangeDps = LSM6DS_GYRO_RANGE_2000_DPS;
        break;

    default:
        break;
    }
}
/*
@brief Function to configure the IMU2 magnetometer range in Gauss based on the provided rate value.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param rateValue The rate value to configure the IMU2 magnetometer range in Gauss
This function sets the IMU2 magnetometer range in Gauss based on the provided rate value. It maps the rate value to a specific LIS3MDL range constant.

*/

void configureImu2MagRangeGauss(EEPROMDataCheckUnion &configData, uint8_t rateValue)
{
    switch (rateValue)
    {
    case 0:
        configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_4_GAUSS;
        break;

    case 1:
        configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_8_GAUSS;
        break;

    case 2:
        configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_12_GAUSS;
        break;

    case 3:
        configData.EEPROMDataCheck.IMU2MagRangeGauss = LIS3MDL_RANGE_16_GAUSS;
        break;

    default:
        break;
    }
}

/*
@brief Function to handle writing configuration data for IMU and joystick settings to the BLE Gamepad.
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param imuJoystickUnion Reference to the ImuJoystickUnion containing IMU and joystick configuration data
@param bleGamepad Reference to the BleGamepad object
This function retrieves the configuration data from the BLE Gamepad, updates the IMU and
 joystick settings based on the provided configuration, and saves the updated settings to EEPROM.
*/
void onWrieconfig(EEPROMDataCheckUnion &configData, ImuJoystickUnion &imuJoystickUnion, BleGamepad &bleGamepad)
{
    bleGamepad.getcharacterData(bleGamepad.Config, imuJoystickUnion.rawData);

    //  uint8_t cmd = imuJoystickUnion.configDataImuJOTISK.CMD;

    configureIMU1AccelGyroRate(configData, imuJoystickUnion.configDataImuJOTISK.IMU1AccelYyroRateHz);
    configureImu1MagFreqHz(configData, imuJoystickUnion.configDataImuJOTISK.IMU1MagFreqHz);
    configureImu1AccelRangeG(configData, imuJoystickUnion.configDataImuJOTISK.IMU1AccelRangeG);
    configureImu1GyroRangeDps(configData, imuJoystickUnion.configDataImuJOTISK.IMU1GyroRangeDps);
    configureImu1MagRangeGaus(configData, imuJoystickUnion.configDataImuJOTISK.IMU1MagRangeGaus); 

    configureIMU2AccelGyroRate(configData, imuJoystickUnion.configDataImuJOTISK.IMU2AccelGyroFreqHz);
    configureImu2MagFreqHz(configData, imuJoystickUnion.configDataImuJOTISK.IMU2MagFreqHz);
    configureImu2AccelRangeG(configData, imuJoystickUnion.configDataImuJOTISK.IMU2AccelRangeG);
    configureImu2GyroRangeDps(configData, imuJoystickUnion.configDataImuJOTISK.IMU2GyroRangeDps);
    configureImu2MagRangeGauss(configData, imuJoystickUnion.configDataImuJOTISK.IMU2MagRangeGauss);

    saveSetting(configData);

}

/*
brief Sends IMU data over BLE if the gamepad is connected.
@param bleGamepad Reference to the BleGamepad object
@param imu1DataRawPacked Reference to the packed data structure for IMU 1
@param imu2DataRawPacked Reference to the packed data structure for IMU 2
@param imu1EulerCalibration Reference to the packed data structure for IMU 1 Euler calibration status
@param imu2EulerCalibration Reference to the packed data structure for IMU 2 Euler calibration status
*/
void senDataBLE(BleGamepad &bleGamepad,
                JoystickDataUnion &joystickDataUnion,
                IMUDataRawUnion ImuArray[],
                IMUEulernUnion &IMU1eurle,
                IMUEulernUnion &IMU2eurle)
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, ImuArray[0].rawData, sizeof(ImuArray[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, ImuArray[1].rawData, sizeof(ImuArray[1].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, IMU1eurle.rawData, sizeof(IMU1eurle.rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, IMU2eurle.rawData, sizeof(IMU2eurle.rawData));
    }else {
        Serial.println("BLE Gamepad is not connected, cannot send data.");
    }
}

void upBatteryBLE(BatteryData &batteryData, BleGamepad &bleGamepad)
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setBatteryLevel(batteryData.batteryLevel);
        bleGamepad.setBatteryChargeStatus(batteryData.batteryChargeStatus);
    }
}

void bleCalibration(EEPROMDataCheckUnion &eepromData , ImuJoystickUnion &imuJoystickUnion, BleGamepad &bleGamepad)
{
   if(bleGamepad.isOnWriteConfig && bleGamepad.isRightSize) {
      bleGamepad.getcharacterData(bleGamepad.Config, imuJoystickUnion.rawData); 
   }
} 
