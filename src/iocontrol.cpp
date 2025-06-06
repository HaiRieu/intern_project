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

/*
brief Initialize the system by setting up pin modes and initial states.
*/
void systemInit()
{

  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, HIGH);
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
  return (float)adcValue * 4096 / 3.3 ;
}

/*
brief Read analog values from flex sensors and store them in the provided union.
  * @param flexSensorDataUnion Reference to the Flex_sensor_data_Union structure to store sensor data

*/
void readFlexSensor(FlexDataUnion &flexSensorDataUnion)
{
  for (int i = 0; i < NUM_FLEX; i++)
  {
    int adcValue = analogRead(FLEX_SENSOR_PINS[i]);
    flexSensorDataUnion.flexSensorData.FlexSensorData[i] = mapFlexValue(adcValue);
  }
}

void readForceSensor(ForceData &forceData)
{
  int adcValue = analogRead(FORECE_SENSOR_PIN);
  forceData.ForceDataKOhm = mapForceValue(adcValue);
}

/*
@Brief Reads the force sensor value and stores it in the provided union.
  * @param forceDataUnion Reference to the ForceDataUnion structure to store force sensor data
*/
void checkbuttons(ButtonDataUnion &buttonDataUnion)
{
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    if (digitalRead(BUTTON_PINS[i]) == LOW)
    {
      buttonDataUnion.buttonData.buttons[i] = 1;
    }
    else
    {
      buttonDataUnion.buttonData.buttons[i] = 0;
    }
  }
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

void processMotor()
{
  digitalWrite(MOTOR_EN_PIN, HIGH);
  delay(500);
  digitalWrite(MOTOR_EN_PIN, LOW);
}

void readBatteryData(BatteryData &batteryData, Adafruit_MAX17048 &maxlipo)
{

  batteryData.batteryLevel = maxlipo.cellPercent();
}
