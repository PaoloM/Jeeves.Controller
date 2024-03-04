// ==========================================================================================
// Project: Jeeves.Controller
// File:    main.cpp
// Version: 0.9
// Date:    2024-03-03
// License: MIT https://github.com/PaoloM/Jeeves.Controller/blob/main/LICENSE
// ==========================================================================================

#define SENSOR_TYPE "BoilerPlate" // type of sensor
#define VERSION     "0.9"         // firmware version
#define MAIN_TOPIC  "generic"     // default MQTT topic (can be empty)

#include "main.h"

// Global variables -------------------------------------------------------------------------
int counterValue = 0; // generic counter

// - KY-040 values
int KY040_MAX = 3;
int KY040_MIN = 1;
int KY040_CURRENT_VALUE = KY040_MIN;
boolean KY040_BUTTON_PRESSED = false;

// - DHTxx values
float DHT_TEMPERATURE;
float DHT_HUMIDITY;

// TODO: add global variables here

// MQTT sensor specific topics to report values ---------------------------------------------
char sensor_mqtt_topic[58]; // default MQTT location/topic, change as needed

// ==========================================================================================
// IMPLEMENTATION
// ==========================================================================================

// ------------------------------------------------------------------------------------------
// Step 1/7 - Add any sensor-specific initialization code
// ------------------------------------------------------------------------------------------

void sensorSetup()
{
  char s[80];
  IPAddress ip = WiFi.localIP();

  if (SENSOR_SSD1306) // - SSD1306 I2C OLED DISPLAY
  { 
    OLED_DISPLAY.begin();

    // Show splash screen
    OLED_DISPLAY.clearBuffer();
    OLED_DISPLAY.setFont(u8g2_font_helvB12_tr);
    sprintf(s, "%s", SENSOR_TYPE);
    OLED_DISPLAY.drawStr(0, 18, s);
    OLED_DISPLAY.setFont(u8g2_font_profont12_mf);
    sprintf(s, "ID :%06X", DEVICE_ID);
    OLED_DISPLAY.drawStr(0, 32, s);
    sprintf(s, "IP :%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    OLED_DISPLAY.drawStr(0, 42, s);
    sprintf(s, "Loc:%s", LOCATION);
    OLED_DISPLAY.drawStr(0, 52, s);
    sprintf(s, "Ver:%s", VERSION);
    OLED_DISPLAY.drawStr(0, 62, s);
    OLED_DISPLAY.sendBuffer();
    // End splash screen

    ON_SPLASH_SCREEN = true;
  }

  if (SENSOR_DHT) // - DHTxx TEMPERATURE AND HUMIDITY SENSOR
  {
    DHT_SENSOR.begin();
  }

  if (SENSOR_HD74480) // - HD77480 16x2 LCD DISPLAY
  {
    char client_id[20];

    LCD_DISPLAY.init();

    // Show splash screen
    LCD_DISPLAY.backlight();
    LCD_DISPLAY.setCursor(0, 0);
    sprintf(client_id, "Jeeves    %x", DEVICE_ID);
    LCD_DISPLAY.print(client_id);
    LCD_DISPLAY.setCursor(0, 1);
    LCD_DISPLAY.print(SENSOR_TYPE);
    LCD_DISPLAY.setCursor(13, 1);
    LCD_DISPLAY.print(VERSION);
    // End splash screen

    ON_SPLASH_SCREEN = true;
  }

  if (SENSOR_KY040) // - KY040 ROTARY ENCODER
  {
    pinMode(KY040_PIN_IN1, INPUT);
    pinMode(KY040_PIN_IN1, INPUT_PULLUP);
    pinMode(KY040_PIN_IN2, INPUT);
    pinMode(KY040_PIN_IN2, INPUT_PULLUP);
    pinMode(KY040_PIN_BUTTON, INPUT);
    pinMode(KY040_PIN_BUTTON, INPUT_PULLUP);
  }

  // TODO: Add other sensor-specific initialization code here
  /* code */
}

// ------------------------------------------------------------------------------------------
// Step 2/7 - Setup the MQTT topics
// ------------------------------------------------------------------------------------------
void sensorMqttSetup()
{
  if (USE_GENERIC)
  {
    sprintf(sensor_mqtt_topic, "%s/%s", LOCATION, MAIN_TOPIC); // default MQTT location/topic
  }

  // TODO: prepare the MQTT topics for this sensor
  /* code */
}

// ------------------------------------------------------------------------------------------
// Step 3a/7 - Read data from the sensor(s) timed every DELAY_MS milliseconds
// ------------------------------------------------------------------------------------------
void sensorUpdateReadings()
{
  if (USE_GENERIC)
  {
    counterValue++; // generic test counter
  }

  if (SENSOR_DHT) // - DHTxx temperature and humidity readings
  {
    sensors_event_t event;
    DHT_SENSOR.temperature().getEvent(&event);
    DHT_TEMPERATURE = (float)event.temperature; // in degrees Celsius
    if (DHT_TEMPERATURE_IN_FARENHEIT)
    {
      DHT_TEMPERATURE = (DHT_TEMPERATURE * 1.8) + 32;
    }
    DHT_SENSOR.humidity().getEvent(&event);
    DHT_HUMIDITY = (float)event.relative_humidity; // in percentage
  }

  // TODO: Perform measurements every DELAY_MS milliseconds
  /* code */
}

// ------------------------------------------------------------------------------------------
// Step 3b/7 - Read data from the sensor(s) on every loop
// ------------------------------------------------------------------------------------------
void sensorUpdateReadingsQuick()
{
  if (SENSOR_KY040) // - KY040 rotary encoder readings
  {
    switch (KY040_STATUS_CURRENT)
    {
    case KY040_STATUS_PRESSED:
      /* code */
      KY040_STATUS_CURRENT = KY040_STATUS_IDLE;
      break;

    case KY040_STATUS_GOINGUP:
      /* code */
      KY040_STATUS_CURRENT = KY040_STATUS_IDLE;
      break;

    case KY040_STATUS_GOINGDOWN:
      /* code */
      KY040_STATUS_CURRENT = KY040_STATUS_IDLE;
      break;

    default:
      break;
    }
  }

  // TODO: Perform measurements on every loop
  /* code */
}

// ------------------------------------------------------------------------------------------
// Step 4/7 - Send the values to the MQTT server
// ------------------------------------------------------------------------------------------
void sensorReportToMqtt()
{
  char t[255];
  bool emitTimestamp = false;

  if (USE_GENERIC)
  {
    sendToMqttTopicAndValue(sensor_mqtt_topic, String(counterValue)); // generic message
  }

  if (SENSOR_DHT) // - DHTxx temperature and humidity readings
  {
    sprintf(t, "%s/%s", LOCATION, STR_SENSOR_TOPIC_DHT_TEMPERATURE);
    sendToMqttTopicAndValue(t, String(DHT_TEMPERATURE));
    sprintf(t, "%s/%s", LOCATION, STR_SENSOR_TOPIC_DHT_HUMIDITY);
    sendToMqttTopicAndValue(t, String(DHT_HUMIDITY));
    emitTimestamp = true; // mark this measurement with a timestamp
  }

  // TODO: send all the required values to the MQTT broker
  /* code */

  if (emitTimestamp) // Common timestamp for all MQTT topics pub
  {
    time_t temp;
    struct tm *timeptr;
    char s[80];

    temp = time(NULL);
    timeptr = localtime(&temp);

    strftime(s, sizeof(s), "%Y-%m-%d %T", timeptr);
    sprintf(t, "%s/%s", LOCATION, STR_SENSOR_TOPIC_TIMESTAMP);
    sendToMqttTopicAndValue(t, s);
  }
}

// ------------------------------------------------------------------------------------------
// Step 5/7 - Report the latest values to the serial console
// ------------------------------------------------------------------------------------------
void sensorReportToSerial()
{
  if (USE_GENERIC)
  {
    printToSerialTopicAndValue(sensor_mqtt_topic, String(counterValue)); // generic message
  }

  // TODO: report required values to the console
  /* code */
}

// ------------------------------------------------------------------------------------------
// Step 6/7 - Update the local display
// ------------------------------------------------------------------------------------------
void sensorUpdateDisplay()
{
  if (!ON_SPLASH_SCREEN) // update the display only if the splash screen has been dismissed
  { 
    if (SENSOR_SSD1306)
    {
      OLED_DISPLAY.clearBuffer();
      /* code */

      OLED_DISPLAY.sendBuffer();
    }
    if (SENSOR_HD74480)
    {
      /* code */
    }
    /* code */ // <-- other indicators and annunciators
  }
}

// ------------------------------------------------------------------------------------------
// Step 7/7 (optional) - This callback is invoked when an MQTT message is received.
// ------------------------------------------------------------------------------------------
void mqttCallback(char *topic, byte *payload, uint8_t length)
{
  // Prepare message
  String message = "";
  for (int i = 0; i < int(length); i++)
  {
    message += (char)payload[i];
  }

  // log message
  char out[255];
  sprintf(out, STR_MESSAGE_RECEIVED_FORMAT, topic, message.c_str());
  log_out("MQTT", out);

  // TODO: Take action
  /* code */
}
