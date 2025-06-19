#include "functiondef.h"

byte caldata[68];
byte calcount = 0;


/*
@brief Initializes the IMU sensors and starts the sensor fusion process.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param addressLsm6ds I2C address for the LSM6DS3TRC
@param addressLis3mdl I2C address for the LIS3MDL
@param overallDataUnion Reference to the OverallStatusDataUnion for storing status data
@param fusionImu Reference to the Adafruit_NXPSensorFusion object for sensor fusion
@param timestamp Reference to the timestamp variable to store the start time
@return Returns true if both IMU sensors are initialized successfully, false otherwise
*/
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

void setupIMUDataRate(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl)
{

    lsm6ds.setAccelDataRate(LSM6DS_RATE_52_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_52_HZ);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
}

/*
@brief Sets up the IMU data rate and range configurations based on the provided configuration data.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param configData Reference to the EEPROMDataCheckUnion containing configuration data
@param idIMU Identifier for the IMU (1 or 2)
*/
void setupIMUDataRateConfig(Adafruit_LSM6DS3TRC &lsm6ds, Adafruit_LIS3MDL &lis3mdl, EEPROMDataCheckUnion &configData, uint8_t idIMU)
{

    if (idIMU == 1)
    {
        lsm6ds.setAccelDataRate((lsm6ds_data_rate_t)configData.EEPROMDataCheck.IMU1AccelYyroRateHz);
        lsm6ds.setGyroDataRate((lsm6ds_data_rate_t)configData.EEPROMDataCheck.IMU1AccelYyroRateHz);
        lsm6ds.setAccelRange((lsm6ds_accel_range_t)configData.EEPROMDataCheck.IMU1AccelRangeG);
        lsm6ds.setGyroRange((lsm6ds_gyro_range_t)configData.EEPROMDataCheck.IMU1GyroRangeDps);
        lis3mdl.setDataRate((lis3mdl_dataRate_t)configData.EEPROMDataCheck.IMU1MagFreqHz);
        lis3mdl.setRange((lis3mdl_range_t)configData.EEPROMDataCheck.IMU1MagRangeGaus);
    }
    else if (idIMU == 2)
    {
        lsm6ds.setAccelDataRate((lsm6ds_data_rate_t)configData.EEPROMDataCheck.IMU2AccelGyroFreqHz);
        lsm6ds.setGyroDataRate((lsm6ds_data_rate_t)configData.EEPROMDataCheck.IMU2AccelGyroFreqHz);
        lsm6ds.setAccelRange((lsm6ds_accel_range_t)configData.EEPROMDataCheck.IMU2AccelRangeG);
        lsm6ds.setGyroRange((lsm6ds_gyro_range_t)configData.EEPROMDataCheck.IMU2GyroRangeDps);
        lis3mdl.setDataRate((lis3mdl_dataRate_t)configData.EEPROMDataCheck.IMU2MagFreqHz);
        lis3mdl.setRange((lis3mdl_range_t)configData.EEPROMDataCheck.IMU2MagRangeGauss);
    }
}

/*
@brief Sets up the IMU interrupts for the specified sensors.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param pinInterrup Pin number for the interrupt
@param imuId Identifier for the IMU (1 or 2)
*/
bool loadCalibrationImu(Adafruit_Sensor_Calibration_EEPROM &calIMU,
                        uint8_t addressEEPROMImu,
                        IMUEulernUnion &imuEulerCalibration)
{
    if (!calIMU.begin(addressEEPROMImu))
    {
        Serial.println("Failed to initialize calibration EEPROM");
        imuEulerCalibration.eulerCalibStatus.calibation = false;
        return false;
    }
    if (!calIMU.loadCalibration())
    {
        Serial.println("Calibration data not found or invalid");
        imuEulerCalibration.eulerCalibStatus.calibation = false;
        return false;
    }

    imuEulerCalibration.eulerCalibStatus.calibation = true ;
    return true;
}

/*
@brief Initializes the IMU sensors and sets up the data rate, interrupts, and calibration.
@param lsm6ds Reference to the LSM6DS3TRC sensor
@param lis3mdl Reference to the LIS3MDL sensor
@param addressLsm6ds I2C address for the LSM6DS3TRC
@param addressLis3mdl I2C address for the LIS3MDL
@param overallDataUnion Reference to the OverallStatusDataUnion for storing status data
@param fusion Reference to the Adafruit_NXPSensorFusion object for sensor fusion
@param cal Reference to the Adafruit_Sensor_Calibration_EEPROM object for calibration
@param addressEEPROMImu I2C address for the EEPROM storing IMU calibration data
@param imuEulerCalibration Reference to the IMUEulernUnion for storing Euler angles
@param pinInterrup Pin number for the interrupt
@param imuId Identifier for the IMU (1 or 2)
@param timestamp Reference to the timestamp variable to store the start time
@param eepromDataCheck Reference to the EEPROMDataCheckUnion for storing EEPROM data check
@return Returns true if the IMU sensors are initialized successfully, false otherwise
*/
bool initIMU(Adafruit_LSM6DS3TRC &lsm6ds,
             Adafruit_LIS3MDL &lis3mdl,
             uint8_t addressLsm6ds,
             uint8_t addressLis3mdl,
             OverallStatusDataUnion &overallDataUnion,
             Adafruit_NXPSensorFusion &fusion,
             Adafruit_Sensor_Calibration_EEPROM &cal,
             uint8_t addressEEPROMImu,
             IMUEulernUnion &imuEulerCalibration,
             uint8_t pinInterrup, uint8_t imuId, uint32_t &timestamp, EEPROMDataCheckUnion &eepromDataCheck)
{

    if (imuStart(lsm6ds, lis3mdl, addressLsm6ds, addressLis3mdl,
                 overallDataUnion, fusion, timestamp))
    {
        Serial.println("IMU initialized successfully");
        // setupIMUDataRate(lsm6ds, lis3mdl);
        setupIMUDataRateConfig(lsm6ds, lis3mdl, eepromDataCheck, imuId);
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

        //  if (bleGamepad.isCalibration == 1)
        //   {
        //   Serial.println("Calibrating IMU...");
        //  bleGamepad.isCalibration = 0;
        cal.calibrate(accel);
        cal.calibrate(gyro);
        cal.calibrate(mag);
       // cal.printSavedCalibration();
      //  cal.saveCalibration();
        //    }

        if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
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
@brief Prints the raw IMU data to the serial monitor.
@param imuData Reference to the IMUDataRawUnion containing raw IMU data
This function prints the raw accelerometer, gyroscope, and magnetometer data in a comma-separated
format to the serial monitor. The accelerometer data is scaled to mg, gyroscope data to degrees per second, and magnetometer data to microteslas.
*/
void serialMonitor(IMUDataRawUnion &imuData)
{
    Serial.print("Raw:");
    Serial.print(int(imuData.dataImu.accelXmg * 8192 / 9.8));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.accelYmg * 8192 / 9.8));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.accelZmg * 8192 / 9.8));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.GyroXrads * DEGREES_PER_RADIAN * 16));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.GyroYrads * DEGREES_PER_RADIAN * 16));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.GyroZrads * DEGREES_PER_RADIAN * 16));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.MagXuT * 10));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.MagYuT * 10));
    Serial.print(",");
    Serial.print(int(imuData.dataImu.MagZuT * 10));
    Serial.println("");
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


void receiveCalibration(Adafruit_Sensor_Calibration_EEPROM &cal)
{
    uint16_t crc;
    byte b, i;

    while (Serial.available())
    {
        b = Serial.read();
        if (calcount == 0 && b != 117)
        {

            return;
        }
        if (calcount == 1 && b != 84)
        {

            calcount = 0;
            return;
        }

        caldata[calcount++] = b;
        if (calcount < 68)
        {

            return;
        }

        crc = 0xFFFF;
        for (i = 0; i < 68; i++)
        {
            crc = CRC16(crc, caldata[i]);
        }
        if (crc == 0)
        {

            float offsets[16];
            memcpy(offsets, caldata + 2, 16 * 4);
            cal.accel_zerog[0] = offsets[0];
            cal.accel_zerog[1] = offsets[1];
            cal.accel_zerog[2] = offsets[2];

            cal.gyro_zerorate[0] = offsets[3];
            cal.gyro_zerorate[1] = offsets[4];
            cal.gyro_zerorate[2] = offsets[5];

            cal.mag_hardiron[0] = offsets[6];
            cal.mag_hardiron[1] = offsets[7];
            cal.mag_hardiron[2] = offsets[8];

            cal.mag_field = offsets[9];

            cal.mag_softiron[0] = offsets[10];
            cal.mag_softiron[1] = offsets[13];
            cal.mag_softiron[2] = offsets[14];
            cal.mag_softiron[3] = offsets[13];
            cal.mag_softiron[4] = offsets[11];
            cal.mag_softiron[5] = offsets[15];
            cal.mag_softiron[6] = offsets[14];
            cal.mag_softiron[7] = offsets[15];
            cal.mag_softiron[8] = offsets[12];

            if (!cal.saveCalibration())
            {
              
                Serial.println("Couldn't save calibration");
            }
            else
            {
              
                Serial.println("Wrote calibration");
                
            }
            calcount = 0;
            return;
        }

        for (i = 2; i < 67; i++)
        {
            if (caldata[i] == 117 && caldata[i + 1] == 84)
            {

                calcount = 68 - i;
                memmove(caldata, caldata + i, calcount);
                return;
            }
        }

        if (caldata[67] == 117)
        {
            caldata[0] = 117;
            calcount = 1;
        }
        else
        {
            calcount = 0;
        }
    }


   
}

