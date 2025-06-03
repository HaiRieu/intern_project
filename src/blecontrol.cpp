
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
  //bleGamepadConfig.setButtonCount(numOfButtons);
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
                IMU_data_Raw_packed ImuArray[],
               IMU_euler_calib_status_Union IMUeurle[NUM_IMUS])
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, ImuArray[0].rawData, sizeof(ImuArray[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, ImuArray[1].rawData, sizeof(ImuArray[1].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, IMUeurle[0].rawData, sizeof(IMUeurle[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, IMUeurle[1].rawData, sizeof(IMUeurle[1].rawData));
        
   
   
    }
}


