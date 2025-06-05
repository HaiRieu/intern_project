
#include "functiondef.h"

void setCalibrartionToEEPROM(Adafruit_Sensor_Calibration_EEPROM &calibratinImu1,
                             Adafruit_Sensor_Calibration_EEPROM &calibratinImu2,
                             IMUEulernUnion &imu1EulerCalibration,
                             IMUEulernUnion &imu2EulerCalibration)
{
    calibratinImu1.saveCalibration() ; 
}

bool onWriteforCalibration(BleGamepad &bleGamepad, IMUEulernUnion &imu1Calibtartion, IMUEulernUnion &imu2Calibtartion)
{
    if (bleGamepad.isOnWriteConfig && bleGamepad.isRightSize)
    {
        Serial.println("BLE calibration write event detected");
        uint8_t recevedData[3];
        bleGamepad.getcharacterData(bleGamepad.Config, recevedData);
    }
    return  false;
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
    bleGamepadConfig.setButtonCount(NUM_BUTTONS);
    // bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
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
                IMUEulernUnion IMUeurle[])
{
    if (bleGamepad.isConnected())
    {

        int16_t Xaxis = map(joystickDataUnion.joystickData.Xaxis, 0, 4095, -32768, 32767);
        int16_t Yaxis = map(joystickDataUnion.joystickData.Yaxis, 0, 4095, -32768, 32767);
        bleGamepad.setAxes(Xaxis, Y_AXIS);
        if (joystickDataUnion.joystickData.joystickButton)
        {
            bleGamepad.press(1);
        }
        else
        {
            bleGamepad.release(1);
        }
        bleGamepad.sendReport();

        bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, ImuArray[0].rawData, sizeof(ImuArray[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, ImuArray[1].rawData, sizeof(ImuArray[1].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, IMUeurle[0].rawData, sizeof(IMUeurle[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, IMUeurle[1].rawData, sizeof(IMUeurle[1].rawData));
    }
}
