#include "functiondef.h"

const int FLEX_SENSOR_PINS[NUM_FLEX] = {6, 7, 8, 9, 10};
const int BUTTON_PINS[NUM_BUTTONS] = {0, 1, 2, 3};

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

float mapFlexValue(int adcValue)
{
  adcValue = constrain(adcValue, FLEX_MIN, FLEX_MAX);
  return map(adcValue, FLEX_MIN, FLEX_MAX, 0, 100.0f);
}

/*
brief Read analog values from flex sensors and store them in the provided union.
  * @param flexSensorDataUnion Reference to the Flex_sensor_data_Union structure to store sensor data

*/
void readAnnalogSensor(FlexDataUnion &flexSensorDataUnion)
{
  for (int i = 0; i < NUM_FLEX; i++)
  {
    int adcValue = analogRead(FLEX_SENSOR_PINS[i]);
    flexSensorDataUnion.flexSensorData.FlexSensorData[i] = mapFlexValue(adcValue);
  }
}

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

void readJoystick(JoystickDataUnion &joystickDataUnion)
{

  joystickDataUnion.joystickData.Xaxis = analogRead(XAXIS_PIN);
  joystickDataUnion.joystickData.Yaxis = analogRead(YAXIS_PIN);
  joystickDataUnion.joystickData.joystickButton = digitalRead(JOYSTICK_BUTTON_PIN) == HIGH ? 1 : 0;
}

/*
brief Cycles through RGB colors once with a delay.
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

void readBatteryData(BatteryData &batteryData ,Adafruit_MAX17048 &maxlipo )
{
   
   batteryData.batteryLevel = maxlipo.cellPercent() ;

}

