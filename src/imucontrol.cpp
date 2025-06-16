#include "functiondef.h"



bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds,
              Adafruit_LIS3MDL &lis3mdl,
              uint8_t addressLsm6ds,
              uint8_t addressLis3mdl,
              OverallStatusDataUnion &overallDataUnion,
              Adafruit_NXPSensorFusion &fusionImu, uint32_t &timestamp)
{

    bool imuStatusLsm6ds = false;
    bool imuStatusLis3mdl = false;

    if (lsm6ds.begin_I2C(addressLsm6ds))
    {
        switch (addressLsm6ds)
        {
        case ADDRESS_LSM6DS1:
            Serial.println("LSM6DS1 initialized successfully");
            imuStatusLsm6ds = true;
            break;
        case ADDRESS_LSM6DS2:
            Serial.println("LSM6DS2 initialized successfully");
            imuStatusLsm6ds = true;
            break;

        default:
            break;
        }
    }

    if (lis3mdl.begin_I2C(addressLis3mdl))
    {

        switch (addressLis3mdl)
        {
        case ADDRESS_LIS3MDL1:
            Serial.println("LIS3MDL1 initialized successfully");
            imuStatusLis3mdl = true;
            break;

        case ADDRESS_LIS3MDL2:
            Serial.println("LIS3MDL2 initialized successfully");
            imuStatusLis3mdl = true;
            break;

        default:
            break;
        }
    }

    if (imuStatusLsm6ds && imuStatusLis3mdl)
    {
        fusionImu.begin(FILTER_UPDATE_RATE_HZ);
        timestamp = millis();
    }

    return imuStatusLsm6ds && imuStatusLis3mdl;
}

void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl , lsm6ds_data_rate_t )
{

    lsm6ds.setAccelDataRate(LSM6DS_RATE_52_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_52_HZ);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
}

bool loadCalibrationImu(Adafruit_Sensor_Calibration_EEPROM &calIMU,
                        uint8_t addressEEPROMImu,
                        IMUEulernUnion &imuEulerCalibration)
{
    bool calibrationStatusImu = false;

    if (calIMU.begin(addressEEPROMImu))
    {
        switch (addressEEPROMImu)
        {
        case IMU1_CAL_EEPROM_ADDR:
            Serial.println("IMU1 calibration EEPROM initialized successfully");
            calibrationStatusImu = true;
            imuEulerCalibration.eulerCalibStatus.calibation = true;
            break;

        case IMU2_CAL_EEPROM_ADDR:
            Serial.println("IMU2 calibration EEPROM initialized successfully");
            calibrationStatusImu = true;
            imuEulerCalibration.eulerCalibStatus.calibation = true;
            break;

        default:
            Serial.println("Unknown EEPROM address for IMU1 calibration");
            calibrationStatusImu = false;
            imuEulerCalibration.eulerCalibStatus.calibation = false;
            break;
        }
    }

    return calibrationStatusImu;
}

bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds,
             Adafruit_LIS3MDL &lis3mdl,
             uint8_t addressLsm6ds,
             uint8_t addressLis3mdl,
             OverallStatusDataUnion &overallDataUnion,
             Adafruit_NXPSensorFusion &fusion,
             Adafruit_Sensor_Calibration_EEPROM &cal,
             uint8_t addressEEPROMImu,
             IMUEulernUnion &imuEulerCalibration,
             uint8_t pinInterrup, uint8_t imuId , uint32_t &timestamp)
{

    if (imuStart(lsm6ds, lis3mdl, addressLsm6ds, addressLis3mdl,
                 overallDataUnion, fusion, timestamp))
    {
        Serial.println("IMU initialized successfully");
        setupIMUDataRate(lsm6ds, lis3mdl);
        loadCalibrationImu(cal, addressEEPROMImu, imuEulerCalibration);
        setupImuInterrupts(lsm6ds, lis3mdl, pinInterrup, imuId);
        return true;
    }
    return false;
}

/*
@brief Reads data from the IMU sensors and updates the IMUDataRawUnion and IMUEulernUnion arrays.
@param lsm6ds1 Reference to the first LSM6DS3TRC sensor
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl1 Reference to the first LIS3MDL sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object for calibration
@param fusion1 Reference to the first Adafruit_NXPSensorFusion object for sensor fusion
@param fusion2 Reference to the second Adafruit_NXPSensorFusion object for sensor fusion
@param ImuArray Array of IMUDataRawUnion structures to store raw IMU data
@param IMUeurle Array of IMUEulernUnion structures to store Euler angles
*/

void readDataIMU(Adafruit_LSM6DS3TRC &lsm6ds,
                 Adafruit_LIS3MDL &lis3mdl,
                 Adafruit_Sensor_Calibration_EEPROM &cal,
                 Adafruit_NXPSensorFusion &fusion,
                 IMUDataRawUnion &imuData,
                 IMUEulernUnion &IMUeurle,
                 BleGamepad &bleGamepad, bool &imuStatus, uint32_t &timestamp)
{
    if (imuStatus)
    {
        sensors_event_t accel, gyro, mag, temp;
        lsm6ds.getEvent(&accel, &gyro, &temp);
        lis3mdl.getEvent(&mag);

     //   if (bleGamepad.isOnWriteConfig == 1)
     //   {
            cal.calibrate(accel);
            cal.calibrate(gyro);
            cal.calibrate(mag);
            cal.saveCalibration();
            bleGamepad.isOnWriteConfig = 0;
  //      }

         if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ) )
          {
              return;
          }
          timestamp = millis();

        imuData.dataImu.accelXmg = accel.acceleration.x;
        imuData.dataImu.accelYmg = accel.acceleration.y;
        imuData.dataImu.accelZmg = accel.acceleration.z;

        imuData.dataImu.GyroXrads = gyro.gyro.x;
        imuData.dataImu.GyroYrads = gyro.gyro.y;
        imuData.dataImu.GyroZrads = gyro.gyro.z;

        imuData.dataImu.MagXuT = mag.magnetic.x;
        imuData.dataImu.MagYuT = mag.magnetic.y;
        imuData.dataImu.MagZuT = mag.magnetic.z;

        fusion.update(imuData.dataImu.GyroXrads,
                      imuData.dataImu.GyroYrads,
                      imuData.dataImu.GyroZrads,
                      imuData.dataImu.accelXmg,
                      imuData.dataImu.accelYmg,
                      imuData.dataImu.accelZmg,
                      imuData.dataImu.MagXuT,
                      imuData.dataImu.MagYuT,
                      imuData.dataImu.MagZuT);

        IMUeurle.eulerCalibStatus.EulerRolldeg = fusion.getRoll();
        IMUeurle.eulerCalibStatus.EulerPitchdeg = fusion.getPitch();
        IMUeurle.eulerCalibStatus.EulerYawdeg = fusion.getYaw();
    }
    else
    {
        Serial.println("IMU1 is not initialized or not connected");
    }
}

/*
@brief Updates the battery level and charge status in the BLE Gamepad.
@param batteryData Reference to the BatteryData structure containing battery information
@param bleGamepad Reference to the BleGamepad object
This function checks if the BLE Gamepad is connected and updates the battery level and charge status accordingly.
*/
void upBatteryBLE(BatteryData &batteryData, BleGamepad &bleGamepad)
{
    if (bleGamepad.isConnected())
    {
        bleGamepad.setBatteryLevel(batteryData.batteryLevel);
        bleGamepad.setBatteryChargeStatus(batteryData.batteryChargeStatus);
    }
}
/*
@brief Parses motion calibration data from a string and stores it in an array of offsets.
@param data The string containing the calibration data in the format "CalibrationData <offset1> <offset2> ... <offset12>"
@param offsets Pointer to an array of floats where the parsed calibration data will be stored
@return True if the parsing was successful, false otherwise
This function checks if the input string starts with "CalibrationData", extracts the calibration values, and stores them in the provided offsets array. It returns true if successful, or false if the format is invalid or no values are found.
*/

bool parseMotionCalData(const String &data, float *offsets)
{
    if (!data.startsWith("CalibrationData"))
    {
        Serial.println("Error: Invalid calibration data format");
        return false;
    }

    int startIdx = data.indexOf(' ') + 1;
    if (startIdx == 0)
    {
        Serial.println("Error: No calibration values found");
        return false;
    }
    int tokenCount = 0;
    String remainingData = data.substring(startIdx);
    for (int i = 0; i < 12; i++)
    {
        int spaceIdx = remainingData.indexOf(' ');
        String token;

        if (spaceIdx == -1)
        {
            token = remainingData;
            remainingData = "";
        }
        else
        {
            token = remainingData.substring(0, spaceIdx);
            remainingData = remainingData.substring(spaceIdx + 1);
        }

        token.trim();
        if (token.length() == 0)
        {
            Serial.println("Error: Empty token found");
            return false;
        }

        offsets[i] = token.toFloat();
        tokenCount++;

        if (remainingData.length() == 0 && i < 11)
        {
            Serial.println("Error: Not enough calibration values");
            return false;
        }
    }
    return true;
}

void saveMotionCal(Adafruit_Sensor_Calibration_EEPROM &cal, float *offsets)
{
    cal.mag_hardiron[0] = offsets[0];
    cal.mag_hardiron[1] = offsets[1];
    cal.mag_hardiron[2] = offsets[2];

    cal.mag_softiron[0] = offsets[3];
    cal.mag_softiron[1] = offsets[4];
    cal.mag_softiron[2] = offsets[5];
    cal.mag_softiron[3] = offsets[6];
    cal.mag_softiron[4] = offsets[7];
    cal.mag_softiron[5] = offsets[8];
    cal.mag_softiron[6] = offsets[9];
    cal.mag_softiron[7] = offsets[10];
    cal.mag_softiron[8] = offsets[11];
    cal.saveCalibration();
}

void handleSerial(Adafruit_Sensor_Calibration_EEPROM &cal)
{
    if (Serial.available())
    {
        String value = Serial.readStringUntil('\n').c_str();
        if (value == "CalibrationData")
        {
            float offser[12];
            parseMotionCalData(value, offser);
            saveMotionCal(cal, offser);
        }
    }
}

/*
@brief Sets the IMU calibration for the specified IMU union and BLE Gamepad.
@param ImuEulernUnion Reference to the IMUEulernUnion containing IMU calibration data
@param bleGamepad Reference to the BleGamepad object
@param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object for calibration
@return True if calibration was successfully loaded, false otherwise

*/
bool setImuCalibration(IMUEulernUnion &ImuEulernUnion,
                       BleGamepad &bleGamepad,
                       Adafruit_Sensor_Calibration_EEPROM &cal)

{
    unsigned long startTime = millis();
    bool isCalibrated = false;
    if (cal.loadCalibration())
    {
        isCalibrated = true;
    }
    else
    {
        handleSerial(cal);
        isCalibrated = true;
        Serial.println("Failed to load calibration data for IMU1.");
    }
    unsigned long loadTime = millis() - startTime;
    return isCalibrated;
}

/*
@brief Updates the calibration status of the specified IMU in the BLE Gamepad.
@param ImuEulernUnion Reference to the IMUEulernUnion containing IMU calibration data
@param bleGamepad Reference to the BleGamepad object
@param imuNumber The IMU number (1 or 2) to update the calibration status for
@param isCalibrated Boolean indicating whether the IMU is calibrated or not
This function checks if the BLE Gamepad is connected and updates the calibration status of the specified IMU. It sends the updated calibration data to the appropriate characteristic based on the IMU number.
*/

void updateCalibrationIMU(IMUEulernUnion &ImuEulernUnion,
                          BleGamepad &bleGamepad,
                          uint8_t imuNumber,
                          bool isCalibrated)
{
    if (bleGamepad.isConnected())
    {
        ImuEulernUnion.eulerCalibStatus.calibation = isCalibrated ? 1 : 0;

        if (imuNumber == 1)
        {
            bleGamepad.setterCharacterData(bleGamepad.IMU1FuseDataCaliStatus, ImuEulernUnion.rawData, sizeof(ImuEulernUnion.rawData));
        }
        else if (imuNumber == 2)
        {
            bleGamepad.setterCharacterData(bleGamepad.IMU2FuseDataCaliStatus, ImuEulernUnion.rawData, sizeof(ImuEulernUnion.rawData));
        }
        else
        {
            Serial.println("Invalid IMU number for calibration update.");
        }
    }
}

/*
@brief Handles BLE calibration for IMU and joystick settings.
@param imuJoystickUnion Reference to the ImuJoystickUnion containing IMU and joystick configuration data
@param imu1EulernUnion Reference to the IMUEulernUnion for IMU 1 calibration data
@param imu2EulernUnion Reference to the IMUEulernUnion for IMU 2 calibration data
@param bleGamepad Reference to the BleGamepad object
@param cal1 Reference to the Adafruit_Sensor_Calibration_EEPROM object for IMU 1 calibration
@param cal2 Reference to the Adafruit_Sensor_Calibration_EEPROM object for IMU 2 calibration
This function checks if the BLE Gamepad is in write configuration mode and processes calibration commands for IMU 1 and IMU 2. It updates the calibration status and sends the updated data to the BLE Gamepad.
*/

void bleCalibration(ImuJoystickUnion &imuJoystickUnion, IMUEulernUnion &imu1EulernUnion,
                    IMUEulernUnion &imu2EulernUnion, BleGamepad &bleGamepad,
                    Adafruit_Sensor_Calibration_EEPROM &cal1, Adafruit_Sensor_Calibration_EEPROM &cal2)
{
    if (bleGamepad.isOnWriteConfig == 1 && bleGamepad.isRightSize == 1)
    {
        bleGamepad.getcharacterData(bleGamepad.Config, imuJoystickUnion.rawData);
        if (imuJoystickUnion.configDataImuJOTISK.CMD == 2)
        {
            bool isCalibrated = setImuCalibration(imu1EulernUnion, bleGamepad, cal1);
            updateCalibrationIMU(imu1EulernUnion, bleGamepad, 1, isCalibrated);
        }
        else if (imuJoystickUnion.configDataImuJOTISK.CMD == 3)
        {
            bool isCalibrated = setImuCalibration(imu2EulernUnion, bleGamepad, cal2);
            updateCalibrationIMU(imu2EulernUnion, bleGamepad, 2, isCalibrated);
        }

        bleGamepad.isOnWriteConfig = 0;
        bleGamepad.isRightSize = 0;
    }
}
