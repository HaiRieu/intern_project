#include "functiondef.h"

const int FLEX_SENSOR_PINS[NUM_FLEX] = {6, 7, 8, 9, 10};
const int BUTTON_PINS[NUM_BUTTONS] = {0, 1, 2, 3};

const FlexDataPoint FLEX_POINTS[] = {
    FLEX_POINT_1,
    FLEX_POINT_2,
    FLEX_POINT_3,
    FLEX_POINT_4,
    FLEX_POINT_5,
    FLEX_POINT_6,
    FLEX_POINT_7,
    FLEX_POINT_8,
    FLEX_POINT_9};

const int size = sizeof(FLEX_POINTS) / sizeof(FlexDataPoint);

static joystickCali joyCali = {2048, 2048, 100, 4095, false};

volatile unsigned long pressStartTime = 0;
volatile bool buttonPressed = false;
volatile bool eventHandled = false;
bool longPrinted = false;
bool isLedOn = false;
unsigned long lastLed = 0;

/*
brief Initialize the system by setting up pin modes and initial states.
*/
void systemInit()
{
  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, HIGH);
  pinMode(SW_DE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SW_DE_PIN), switchChangeISR, CHANGE);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);

  for (int i = 0; i < NUM_FLEX; i++)
  {
    pinMode(FLEX_SENSOR_PINS[i], INPUT);
  }

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  pinMode(FORECE_SENSOR_PIN, INPUT);

  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
}

/*
@brief Maps the ADC value from a flex sensor to a corresponding degree value.
  * @param adcValue The ADC value read from the flex sensor
  * @return The mapped degree value based on predefined flex points
*/
float mapFlexValue(int adcValue)
{
  adcValue = constrain(adcValue, FLEX_POINTS[0].adcValue, FLEX_POINTS[NUM_FLEX - 1].adcValue);

  for (int i = 0; i <= size - 1; i++)
  {
    if (adcValue <= FLEX_POINTS[i + 1].adcValue)
    {

      float x1 = FLEX_POINTS[i].adcValue;
      float y1 = FLEX_POINTS[i].degrees;
      float x2 = FLEX_POINTS[i + 1].adcValue;
      float y2 = FLEX_POINTS[i + 1].degrees;

      return y1 + (adcValue - x1) * (y2 - y1) / (x2 - x1);
    }
  }
  return FLEX_POINTS[size - 1].degrees;
}

/*
@brief Maps the ADC value from a force sensor to a corresponding force value.
  * @param adcValue The ADC value read from the force sensor
  * @return The mapped force value based on a linear mapping
*/

float mapForceValue(int adcValue)
{
  return (float)adcValue * 4096 / 3.3;
}

/*
brief Read analog values from flex sensors and store them in the provided union.
  * @param flexSensorDataUnion Reference to the Flex_sensor_data_Union structure to store sensor data

*/
void readFlexSensor(FlexDataUnion &flexSensorDataUnion, BleGamepad &bleGamepad)
{
  for (int i = 0; i < NUM_FLEX; i++)
  {
    int adcValue = analogRead(FLEX_SENSOR_PINS[i]);
    flexSensorDataUnion.flexSensorData.FlexSensorData[i] = mapFlexValue(adcValue);
  }

  if (bleGamepad.isConnected())
  {
    bleGamepad.setterCharacterData(bleGamepad.FlexSensorData, flexSensorDataUnion.rawData, sizeof(flexSensorDataUnion));
  }
}

void readForceSensor(ForceData &forceData, BleGamepad &bleGamepad)
{
  int adcValue = analogRead(FORECE_SENSOR_PIN);
  forceData.ForceDataKOhm = mapForceValue(adcValue);
  if (bleGamepad.isConnected())
  {
    bleGamepad.setterCharacterData(bleGamepad.ForceSensorData, (uint8_t *)&forceData, sizeof(forceData));
  }
}

/*
@Brief Reads the force sensor value and stores it in the provided union.
  * @param forceDataUnion Reference to the ForceDataUnion structure to store force sensor data
*/
void checkbuttons(ButtonDataUnion &buttonDataUnion, BleGamepad &bleGamepad)
{
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    if (digitalRead(BUTTON_PINS[i]) == HIGH)
    {
      buttonDataUnion.buttonData.buttons[i] = 1;
    }
    else
    {
      buttonDataUnion.buttonData.buttons[i] = 0;
    }
  }
  if (bleGamepad.isConnected())
  {
    bleGamepad.setterCharacterData(bleGamepad.Buttons, buttonDataUnion.rawData, sizeof(buttonDataUnion.rawData));
  }
}

/*
@brief Processes the joystick ADC value and returns a scaled value based on calibration data.
  * @param adcValue The ADC value read from the joystick
  * @param isXaxis True if processing the X-axis, false for Y-axis
  * @return Scaled joystick value in the range of -32768 to 32767
  * This function applies dead zone and scaling based on the joystick calibration data.
  * It calculates the distance from the center position and scales it to fit within the range of a signed 16-bit integer.
  * If the distance is within the dead zone, it returns 0.
*/
int16_t processJoystick(uint16_t adcValue, bool isXaxis)
{
  uint16_t center = isXaxis ? joyCali.centerX : joyCali.xenterY;
  int32_t distance = (int32_t)adcValue - (int32_t)center;

  if (abs(distance) < joyCali.deadZone)
  {
    return 0;
  }
  int32_t maxDistance = joyCali.maxRange - joyCali.deadZone;

  int32_t scaledValue;

  if (distance > 0)
  {
    scaledValue = (distance - joyCali.deadZone) * 32767 / maxDistance;
  }
  else
  {
    scaledValue = (distance + joyCali.deadZone) * -32768 / maxDistance;
  }

  return (int16_t)constrain(scaledValue, -32768, 32767);
}

/*
@brief Smooths the joystick value using a simple low-pass filter.
  * @param newValue The new joystick value to be smoothed
  * @param preValue Reference to the previous smoothed value
  * @return The updated smoothed value
  * This function applies a smoothing factor (alpha) to the new value and the previous value.
  * It returns the new smoothed value, which is a weighted average of the new and previous values.
*/
int16_t smoothJoystick(int16_t newValue, int16_t &preValue)
{
  const float alpha = 0.3;
  preValue = (int16_t)(alpha * newValue + (1 - alpha) * preValue);
  return preValue;
}

/*
@brief Calibrates the joystick by averaging multiple readings.
  * This function reads the X and Y axis values from the joystick multiple times
  * and calculates the average to determine the center position for calibration.
  * The results are stored in the joyCali structure.
*/
void calibrateJoystick()
{
  uint32_t sumX = 0, sumY = 0;
  for (int i = 0; i < 100; i++)
  {
    sumX += analogRead(XAXIS_PIN);
    sumY += analogRead(YAXIS_PIN);
    delay(10);
  }
  joyCali.centerX = sumX / 100;
  joyCali.xenterY = sumY / 100;
}

/*
@brief Reads the joystick data and stores it in the provided union.
  * @param joystickDataUnion Reference to the JoystickDataUnion structure to store joystick data
*/
void readJoystick(JoystickDataUnion &joystickDataUnion)
{

  joystickDataUnion.joystickData.Xaxis = analogRead(XAXIS_PIN);
  joystickDataUnion.joystickData.Yaxis = analogRead(YAXIS_PIN);
  joystickDataUnion.joystickData.joystickButton = digitalRead(JOYSTICK_BUTTON_PIN) == HIGH ? 1 : 0;
}

/*
@brief Updates the joystick data in the BLE Gamepad.
  * This function processes the joystick data, smooths the values, and sends the updated data to the BLE Gamepad.
  * It uses a low-pass filter to smooth the joystick values and updates the BLE Gamepad with the new joystick data.
  * @param bleGamepad Reference to the BleGamepad object
  * @param joystickDataUnion Reference to the JoystickDataUnion structure containing joystick data
  * This function reads the joystick data, processes it to apply dead zones and scaling, smooths the values, and sends the updated joystick data to the BLE Gamepad if it is connected.
*/
void updateJoystickBle(BleGamepad &bleGamepad, JoystickDataUnion &joystickDataUnion)
{

  int16_t prevX = 0, prevY = 0;

  int16_t x = processJoystick(joystickDataUnion.joystickData.Xaxis, true);
  int16_t y = processJoystick(joystickDataUnion.joystickData.Yaxis, false);

  x = smoothJoystick(x, prevX);
  y = smoothJoystick(y, prevY);

  joystickDataUnion.joystickData.Xaxis = x;
  joystickDataUnion.joystickData.Yaxis = y;

  if (bleGamepad.isConnected())
  {

    bleGamepad.setterCharacterData(bleGamepad.Joystick, joystickDataUnion.rawData, sizeof(joystickDataUnion.rawData));
  }
}

/*
@brief Cycles through RGB colors once with a delay.
*/
void cycleRGBOnce()
{

  for (int i = 0; i <= 255; i += 5)
  {
    analogWrite(LED_RED_PIN, 255 - i);
    analogWrite(LED_GREEN_PIN, i);
    analogWrite(LED_BLUE_PIN, 0);
  }

  for (int i = 0; i <= 255; i += 5)
  {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 255 - i);
    analogWrite(LED_BLUE_PIN, i);
  }

  for (int i = 0; i <= 255; i += 5)
  {
    analogWrite(LED_RED_PIN, i);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 255 - i);
  }
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GREEN_PIN, 0);
  analogWrite(LED_BLUE_PIN, 0);
}

/*
@brief Processes the motor by enabling it for a short duration.
  * This function sets the MOTOR_EN_PIN to HIGH for 500 milliseconds and then sets it back to LOW.
  * It is used to control the motor's operation, such as starting or stopping it.
*/
void processMotor()
{
  digitalWrite(MOTOR_EN_PIN, HIGH);
  delay(500);
  digitalWrite(MOTOR_EN_PIN, LOW);
}

/*
@brief Reads battery data from the MAX17048 battery monitor and updates the provided BatteryData structure.
  * @param batteryData Reference to the BatteryData structure to store battery information
  * @param maxlipo Reference to the Adafruit_MAX17048 object for battery monitoring
  * This function reads the battery voltage, percentage, charge rate, and charging status,
  * and updates the BatteryData structure accordingly.
*/
void readBatteryData(BatteryData &batteryData, Adafruit_MAX17048 &maxlipo)
{
  float voltage = maxlipo.cellVoltage();
  if (isnan(voltage))
  {
    Serial.println(" to read battery voltage, check battery is connected!");
    return;
  }
  float percent = maxlipo.cellPercent();
  percent = constrain(percent, 0, 100);
  batteryData.batteryLevel = (uint8_t)percent;

  float chargeRate = maxlipo.chargeRate();

  bool isCharging = (voltage > 4.0);

  if (percent >= 100 && !isCharging)
  {
    batteryData.batteryChargeStatus = BatteryChargeStatus::BATTERY_CHARGE_STATUS_FULLY_CHARGED;
  }
  else if (isCharging)
  {
    batteryData.batteryChargeStatus = BatteryChargeStatus::BATTERY_CHARGE_STATUS_CHARGING;
  }
  else
  {
    batteryData.batteryChargeStatus = BatteryChargeStatus::BATTERY_CHARGE_STATUS_NOT_CHARGING;
  }
}

/*
@brief Powers down the system by disabling the VSVY_EN pin and entering an infinite loop.
  * This function is called when a long press on the switch is detected.
*/
void setPowerDown()
{
  digitalWrite(VSVY_EN_PIN, LOW);
  delay(100);
  while (true)
    ;
}

/*
@brief Processes the switch change event.
  * This function checks if a switch change has occurred and if it has been long enough to trigger a power down.
  * If a long press is detected, it calls the setPowerDown function to power down the system.
*/
void processSwithChange()
{

  if (buttonPressed && !longPrinted)
  {
    unsigned long pressDuration = millis() - pressStartTime;
    Serial.println(pressDuration);
    if (pressDuration >= 3000)
    {
      longPrinted = true;
      buttonPressed = false;
      setPowerDown();
    }
  }
  else if (eventHandled)
  {
    eventHandled = false;
    Serial.println("Short press detected");
  }
}

/*
@brief Interrupt Service Routine (ISR) for switch change detection.
  * This function is triggered on a change in the state of the switch connected to SW_DE_PIN.
  * It detects long presses and sets a flag for further processing.
*/
void IRAM_ATTR switchChangeISR()
{

  bool currentState;
  if (digitalRead(SW_DE_PIN) == HIGH)
  {
    currentState = true;
  }
  else
  {
    currentState = false;
  }

  if (currentState && !buttonPressed)
  {
    pressStartTime = millis();
    buttonPressed = true;
    longPrinted = false;
  }
  else if (!currentState && buttonPressed)
  {
    buttonPressed = false;
    eventHandled = true;
  }
}

/*
@brief Updates the LED color based on the provided RGB values and delay time.
  * This function sets the RGB LED pins to the specified color values and waits for the given delay time.
  * After the delay, it turns off the LED by setting all color pins to 0.
  * @param r The red component value (0-255)
  * @param g The green component value (0-255)
  * @param b The blue component value (0-255)
  * @param delayTime The delay time in milliseconds before turning off the LED
*/

void updateLed(int r, int g, int b, int delayTime)
{

  if (COMMON_ANODE)
  {
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  }

  if (!isLedOn)
  {
    analogWrite(LED_RED_PIN, r);
    analogWrite(LED_GREEN_PIN, g);
    analogWrite(LED_BLUE_PIN, b);
    lastLed = millis();
    isLedOn = true;
  }
}

void handleLedAutoOff()
{
  if (isLedOn)
  {
    Serial.println("LED is currently on, checking for auto-off condition");
    if (millis() - lastLed >= 100)
    {
      Serial.println("turning off LED");
      analogWrite(LED_RED_PIN, 0);
      analogWrite(LED_GREEN_PIN, 0);
      analogWrite(LED_BLUE_PIN, 0);
      isLedOn = false;
    }
  }
}