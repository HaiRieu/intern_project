#include "functiondef.h"

uint32_t timestamp;
// bool imu1Status = true;
//  bool imu2Status = true;
bool imu1Status = false;
bool imu2Status = false;
bool imuStattus = false;
/*
@brief Starts the IMU sensors and initializes the sensor fusion.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param addressLsm6ds I2C address for the LSM6DS3TRC sensor
@param addressLis3mdl I2C address for the LIS3MDL sensor
@param overallDataUnion Reference to the OverallStatusDataUnion structure to store status data
@param fusionImu Reference to the Adafruit_NXPSensorFusion object for sensor fusion
@return True if both IMU sensors are initialized successfully, false otherwise
*/
bool imuStart(Adafruit_LSM6DS3TRC &lsm6ds1,
              Adafruit_LIS3MDL &lis3mdl1,
              Adafruit_LSM6DS3TRC &lsm6ds2,
              Adafruit_LIS3MDL &lis3mdl2,
              uint8_t addressLsm6ds1,
              uint8_t addressLis3mdl1,
              uint8_t addressLsm6ds2,
              uint8_t addressLis3mdl2,
              OverallStatusDataUnion &overallDataUnion,
              Adafruit_NXPSensorFusion &fusionImu1,
              Adafruit_NXPSensorFusion &fusionImu2)
{
    bool imu1StatusLsm6ds = false;
    bool imu1StatusLis3mdl = false;
    bool imu2StatusLsm6ds = false;
    bool imu2StatusLis3mdl = false;

    if (lsm6ds1.begin_I2C(addressLsm6ds1))
    {
        switch (addressLsm6ds1)
        {
        case ADDRESS_LSM6DS1:
            Serial.println("LSM6DS1 initialized successfully");
            imu1StatusLsm6ds = true;
            break;

        case ADDRESS_LSM6DS2:
            Serial.println("LSM6DS2 initialized successfully");
            imu1StatusLsm6ds = true;
            break;
        default:
            Serial.println("Unknown LSM6DS3TRC address");
            imu1StatusLsm6ds = false;
            break;
        }
    }

    if (lis3mdl1.begin_I2C(addressLis3mdl1))
    {

        switch (addressLis3mdl1)
        {
        case ADDRESS_LIS3MDL1:
            Serial.println("LIS3MDL1 initialized successfully");
            imu1StatusLis3mdl = true;
            break;

        case ADDRESS_LIS3MDL2:
            Serial.println("LIS3MDL2 initialized successfully");
            imu1StatusLis3mdl = true;
            break;
        default:
            Serial.println("Unknown LIS3MDL address");
            imu1StatusLis3mdl = false;
            break;
        }
    }

    imu1Status = imu1StatusLsm6ds && imu1StatusLis3mdl;
    if (imu1Status)
    {
        fusionImu1.begin(FILTER_UPDATE_RATE_HZ);
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::RUNNING;
    }
    else
    {
        overallDataUnion.overallStatusData.Imu1Status = statusCodeSensor::FAILED;
    }

    if (lsm6ds2.begin_I2C(addressLsm6ds2))
    {
        switch (addressLsm6ds2)
        {
        case ADDRESS_LSM6DS1:
            Serial.println("LSM6DS1 initialized successfully");
            imu2StatusLsm6ds = true;
            break;

        case ADDRESS_LSM6DS2:
            Serial.println("LSM6DS2 initialized successfully");
            imu2StatusLsm6ds = true;
            break;
        default:
            Serial.println("Unknown LSM6DS3TRC address");
            imu2StatusLsm6ds = false;
            break;
        }
    }

    if (lis3mdl2.begin_I2C(addressLis3mdl2))
    {

        switch (addressLis3mdl2)
        {
        case ADDRESS_LIS3MDL1:
            Serial.println("LIS3MDL1 initialized successfully");
            imu2StatusLis3mdl = true;
            break;

        case ADDRESS_LIS3MDL2:
            Serial.println("LIS3MDL2 initialized successfully");
            imu2StatusLis3mdl = true;
            break;
        default:
            Serial.println("Unknown LIS3MDL address");
            imu2StatusLis3mdl = false;
            break;
        }
    }

    imu2Status = imu2StatusLsm6ds && imu2StatusLis3mdl;
    if (imu2Status)
    {
        fusionImu2.begin(FILTER_UPDATE_RATE_HZ);
        overallDataUnion.overallStatusData.Imu2Status = statusCodeSensor::RUNNING;
    }
    else
    {
        overallDataUnion.overallStatusData.Imu2Status = statusCodeSensor::FAILED;
    }

    imuStattus = imu1Status || imu2Status;

    return imuStattus;
}

/*
brief Sets the data rate and range for the IMU sensors.
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
*/
void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds1, Adafruit_LIS3MDL &lis3mdl1,
                      Adafruit_LSM6DS3TRC &lsm6ds2, Adafruit_LIS3MDL &lis3mdl2)
{

    if (imu1Status)
    {
        lsm6ds1.setAccelDataRate(LSM6DS_RATE_104_HZ);
        lsm6ds1.setGyroDataRate(LSM6DS_RATE_104_HZ);
        lsm6ds1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        lsm6ds1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
        lis3mdl1.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);
    }

    if (imu2Status)
    {

        lsm6ds2.setAccelDataRate(LSM6DS_RATE_104_HZ);
        lsm6ds2.setGyroDataRate(LSM6DS_RATE_104_HZ);
        lsm6ds2.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        lsm6ds2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
        lis3mdl2.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl2.setRange(LIS3MDL_RANGE_4_GAUSS);
    }
}

/*
@brief Initialize IMU1 with LSM6DS3TRC and LIS3MDL sensors

 * This function initializes the LSM6DS3TRC and LIS3MDL sensors for IMU1.
 * It sets the data rates, ranges, and other parameters based on the provided configuration.
 * The function also checks if the sensors are ready and updates the overall status data.
 *
 * @param lsm6ds Reference to the LSM6DS3TRC sensor object
 * @param lis3mdl Reference to the LIS3MDL sensor object
 * @param addressLSM6DS Address of the LSM6DS3TRC sensor
 * @param addressLIS3MD Address of the LIS3MDL sensor
 * @param overallStatusData Reference to the OverallStatusDataUnion structure to update status
 * @param fusion Reference to the Adafruit_NXPSensorFusion object for sensor fusion
 * @return true if IMU1 is initialized successfully, false otherwise
*/
bool loadCalibrationImu(Adafruit_Sensor_Calibration_EEPROM &calIMU1,
                        Adafruit_Sensor_Calibration_EEPROM &calIMU2,
                        uint8_t addressEEPROMImu1, uint8_t addressEEPROMimu2,
                        IMUEulernUnion &imu1EulerCalibration,
                        IMUEulernUnion &imu2EulerCalibration)
{
    bool calibrationStatusImu1 = false;
    if (imu1Status)
    {
        if (calIMU1.begin(addressEEPROMImu1))
        {
            switch (addressEEPROMImu1)
            {
            case IMU1_CAL_EEPROM_ADDR:
                Serial.println("IMU1 calibration EEPROM initialized successfully");
                calibrationStatusImu1 = true;
                imu1EulerCalibration.eulerCalibStatus.calibation = true;
                break;
            default:
                Serial.println("Unknown EEPROM address for IMU1 calibration");
                calibrationStatusImu1 = false;
                imu1EulerCalibration.eulerCalibStatus.calibation = false;
                break;
            }
        }
    }

    bool calibrationStatusImu2 = false;

    if (imu2Status)
    {

        if (calIMU2.begin(addressEEPROMimu2))
        {
            switch (addressEEPROMimu2)
            {
            case IMU2_CAL_EEPROM_ADDR:
                Serial.println("IMU2 calibration EEPROM initialized successfully");
                calibrationStatusImu2 = true;
                imu2EulerCalibration.eulerCalibStatus.calibation = true;
                break;
            default:
                Serial.println("Unknown EEPROM address for IMU2 calibration");
                calibrationStatusImu2 = false;
                imu2EulerCalibration.eulerCalibStatus.calibation = false;
                break;
            }
        }
    }

    return true;
}

bool getImu1Status()
{
    return imu1Status;
}
bool getImu2Status()
{
    return imu2Status;
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

void readDataIMU1(Adafruit_LSM6DS3TRC &lsm6ds1,
                  Adafruit_LIS3MDL &lis3mdl1,
                  Adafruit_Sensor_Calibration_EEPROM &cal1,
                  Adafruit_NXPSensorFusion &fusion1,
                  IMUDataRawUnion &imuData,
                  IMUEulernUnion &IMUeurle1)
{
    if (imu1Status)
    {

        sensors_event_t accel1, gyro1, mag1;

        lsm6ds1.getEvent(&accel1, &gyro1, NULL);
        lis3mdl1.getEvent(&mag1);

        cal1.calibrate(accel1);
        cal1.calibrate(gyro1);
        cal1.calibrate(mag1);
        cal1.saveCalibration();

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
        {
            return;
        }
        timestamp = millis();
        imuData.dataImu.accelXmg = accel1.acceleration.x;
        imuData.dataImu.accelYmg = accel1.acceleration.y;
        imuData.dataImu.accelZmg = accel1.acceleration.z;

        imuData.dataImu.GyroXrads = gyro1.gyro.x;
        imuData.dataImu.GyroYrads = gyro1.gyro.y;
        imuData.dataImu.GyroZrads = gyro1.gyro.z;

        imuData.dataImu.MagXuT = mag1.magnetic.x;
        imuData.dataImu.MagYuT = mag1.magnetic.y;
        imuData.dataImu.MagZuT = mag1.magnetic.z;

        fusion1.update(imuData.dataImu.accelXmg, imuData.dataImu.accelYmg, imuData.dataImu.accelZmg,
                       imuData.dataImu.GyroXrads, imuData.dataImu.GyroYrads, imuData.dataImu.GyroZrads,
                       imuData.dataImu.MagXuT, imuData.dataImu.MagYuT, imuData.dataImu.MagZuT);

        IMUeurle1.eulerCalibStatus.EulerRolldeg = fusion1.getRoll();
        IMUeurle1.eulerCalibStatus.EulerPitchdeg = fusion1.getPitch();
        IMUeurle1.eulerCalibStatus.EulerYawdeg = fusion1.getYaw();
    }
    else
    {
        IMUeurle1.eulerCalibStatus.calibation = false;
    }
}

/*
@brief Reads data from the second IMU sensors and updates the IMUDataRawUnion and IMUEulernUnion arrays.
@param lsm6ds2 Reference to the second LSM6DS3TRC sensor
@param lis3mdl2 Reference to the second LIS3MDL sensor
@param cal2 Reference to the Adafruit_Sensor_Calibration_EEPROM object for calibration
@param fusion2 Reference to the second Adafruit_NXPSensorFusion object for sensor fusion
@param imuData Reference to the IMUDataRawUnion structure to store raw IMU data
@param IMUeurle2 Reference to the IMUEulernUnion structure to store Euler angles
*/
void readDataIMU2(Adafruit_LSM6DS3TRC &lsm6ds2,
                  Adafruit_LIS3MDL &lis3mdl2,
                  Adafruit_Sensor_Calibration_EEPROM &cal2,
                  Adafruit_NXPSensorFusion &fusion2,
                  IMUDataRawUnion &imuData,
                  IMUEulernUnion &IMUeurle2)
{

    if (imu2Status)
    {
        sensors_event_t accel2, gyro2, mag2;
        lsm6ds2.getEvent(&accel2, &gyro2, NULL);

        lis3mdl2.getEvent(&mag2);

        cal2.calibrate(accel2);
        cal2.calibrate(gyro2);
        cal2.calibrate(mag2);
        cal2.saveCalibration();

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
        {
            return;
        }
        timestamp = millis();

        imuData.dataImu.accelXmg = accel2.acceleration.x;
        imuData.dataImu.accelYmg = accel2.acceleration.y;
        imuData.dataImu.accelZmg = accel2.acceleration.z;

        imuData.dataImu.GyroXrads = gyro2.gyro.x;
        imuData.dataImu.GyroYrads = gyro2.gyro.y;
        imuData.dataImu.GyroZrads = gyro2.gyro.z;

        imuData.dataImu.MagXuT = mag2.magnetic.x;
        imuData.dataImu.MagYuT = mag2.magnetic.y;
        imuData.dataImu.MagZuT = mag2.magnetic.z;

        fusion2.update(imuData.dataImu.accelXmg, imuData.dataImu.accelYmg, imuData.dataImu.accelZmg,
                       imuData.dataImu.GyroXrads, imuData.dataImu.GyroYrads, imuData.dataImu.GyroZrads,
                       imuData.dataImu.MagXuT, imuData.dataImu.MagYuT, imuData.dataImu.MagZuT);

        IMUeurle2.eulerCalibStatus.EulerRolldeg = fusion2.getRoll();
        IMUeurle2.eulerCalibStatus.EulerPitchdeg = fusion2.getPitch();
        IMUeurle2.eulerCalibStatus.EulerYawdeg = fusion2.getYaw();
    }
    else
    {
        IMUeurle2.eulerCalibStatus.calibation = false;
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
    if (bleGamepad.isOnWriteConfig && bleGamepad.isRightSize)
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
