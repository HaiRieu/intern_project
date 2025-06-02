
#include "functiondef.h" 


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
                IMU1_euler_calib_status_Union &imu1EulerCalibration,
                IMU2_euler_calib_status_Union &imu2EulerCalibration)
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setterCharacterData(bleGamepad.IMU1RawData, ImuArray[0].rawData, sizeof(ImuArray[0].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2RawData, ImuArray[1].rawData, sizeof(ImuArray[1].rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, imu1EulerCalibration.rawData, sizeof(imu1EulerCalibration.rawData));
        bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, imu2EulerCalibration.rawData, sizeof(imu2EulerCalibration.rawData));
        
   
   
    }
}


