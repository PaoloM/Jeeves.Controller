// ==========================================================================================
// Project: Jeeves.BoilerPlate
// File:    main.h
// Version: 0.9
// Date:    2024-03-03
//
// MIT License
//
// Copyright (c) 2020-2024 Paolo Marcucci
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ==========================================================================================

#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <Arduino.h>

/*--------------------------- Configuration ------------------------------*/
#include "config.h"  // Jeeves server address, port
#include "sensor.h"  // Sensor-specific data
#include "strings.h" // Messages, etc

/*--------------------------- Libraries ----------------------------------*/
// Core libraries (included in framework)
#if (USING_ESP32_S2 || USING_ESP32_C3)
// --- Using an ESP32 platform ---

#else
// --- Using an ESP8266 platform ---
#include <ArduinoOTA.h> // Required for OTA updates
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>    // ESP8266 WiFi driver
#include <SoftwareSerial.h> // Allows sensors to avoid the USB serial port
#include <SPI.h>
#include <Wire.h>
#include <WiFiUdp.h>
#endif

// Project-specific libraries
#include <PubSubClient.h>     // MQTT PubSub support - knolleary/PubSubClient@^2.8
#include <Adafruit_Sensor.h>  // Universal sensor library - adafruit/Adafruit Unified Sensor@^1.1.4
#include "RestClient.h"       // ESP8266 REST client SSL by Hal9k https://github.com/Hal9k-dk/REST-test
#include <DNSServer.h>        // Required for AP support by https://github.com/tzapu/WiFiManager
#include <ESP8266WebServer.h> // Required for AP support by https://github.com/tzapu/WiFiManager
#include <WiFiManager.h>      // Access point capability by https://github.com/tzapu/WiFiManager

// #include <ESPAsync_WiFiManager.h> // https://github.com/khoih-prog/ESPAsync_WiFiManager
// #include "RestClient.h" // https://github.com/eduardomarcos/arduino-esp32-restclient

// AsyncWebServer webServer(80);
// #if !( USING_ESP32_S2 || USING_ESP32_C3 )
// DNSServer dnsServer;
// #endif

// // Framework dependencies
// // #include <ESP8266WiFi.h> // ESP8266 WiFi driver
// // #include <ESP8266mDNS.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h> // Required for OTA updates
// #include <Wire.h>
// #include <SPI.h>
// #include <SoftwareSerial.h> // Allows sensors to avoid the USB serial port

#include <time.h>

/*--------------------------- Global Variables ---------------------------*/
// MQTT general
const char *STATUS_TOPIC = "Events"; // MQTT topic to report startup events
char MQTT_BROKER[20];                // IP address of your MQTT broker
char MQTT_MESSAGE_BUFFER[150];       // General purpose buffer for MQTT messages
char CMD_TOPIC[55];                  // MQTT topic for receiving commands
char LOCATION[50];                   // Room/area where the sensor is located

// Wifi
#define WIFI_CONNECT_INTERVAL 500    // ms to wait for wifi connection
#define WIFI_CONNECT_MAX_ATTEMPTS 10 // Number of attempts/intervals to wait

// General
uint32_t DEVICE_ID;            // Unique ID from ESP chip ID
bool ON_SPLASH_SCREEN = false; // TBD
#define CMD_RESTART 1;         // TBD

// Time/NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -28800;   // Replace with your GMT offset (seconds) -28800 for PDT
const int daylightOffset_sec = 3600; // Replace with your daylight offset (seconds) 3600 for one hour

// Loop timer
unsigned int previousUpdateTime = millis();
unsigned int splashScreenTimer = 0;

/* ----------------- Hardware-specific config ---------------------- */
/* Serial */
#define SERIAL_BAUD_RATE 9600 // Speed for USB serial console
#define ESP_WAKEUP_PIN D0     // To reset ESP8266 after deep sleep

/*--------------------------- Function Signatures ---------------------------*/
void mqttCallback(char *topic, byte *payload, uint8_t length);
bool initWifi();
void reconnectMqtt();
void sensorReportToMqtt();
void sensorReportToSerial();
void sensorUpdateReadings();
void sensorUpdateReadingsQuick();
void sensorUpdateDisplay();
void sensorSetup();
void sensorMqttSetup();

/*--------------------------- Instantiate Global Objects --------------------*/
WiFiClient ESP_CLIENT;                                                  // WiFi
PubSubClient MQTT_CLIENT(ESP_CLIENT);                                   // MQTT
RestClient REST_CLIENT = RestClient(JEEVES_SERVER, JEEVES_SERVER_PORT); // Jeeves server connection
LiquidCrystal_I2C LCD_DISPLAY(HD44780_SCREEN_ADDRESS, 20, 4);           // set the LCD address to 0x27 for a 16 chars and 2 line display
DHT_Unified DHT_SENSOR(DHT_PIN, DHT_TYPE);                              // Set pin and type for DHT temp/humidity sensors
U8G2_SSD1306_128X64_NONAME_F_SW_I2C OLED_DISPLAY(U8G2_R0,
                                                 SSD1306_PIN_SCL, SSD1306_PIN_SDA, U8X8_PIN_NONE); // All Boards without Reset of the Display

void log_out(char *component, const char *value)
{
  char out[255];
  char s[100];
  time_t temp;
  struct tm *timeptr;

  temp = time(NULL);
  timeptr = localtime(&temp);

  strftime(s, sizeof(s), "%Y-%m-%d %T", timeptr);

  if (DEBUG)
  {
    sprintf(out, "%s | %s | %s", s, component, value);
    Serial.println(out);
  }
}

void printToSerialTopicAndValue(char *topic, String value)
{
  char out[80];
  sprintf(out, "%s = %s", topic, value.c_str());
  log_out(STR_MQTT_LOG_PREFIX, out);
}

void sendToMqttTopicAndValue(char *topic, String value)
{
  String _value;
  char buffer[255]; // General purpose buffer for MQTT messages

  _value = value;
  _value.toCharArray(buffer, _value.length() + 1);
  MQTT_CLIENT.publish(topic, buffer);
}

// KY-040 debouncing: A valid CW or CCW move returns 1, invalid returns 0.
int8_t read_rotary()
{
  static int8_t rot_enc_table[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

  KY040_PREV_NEXT_CODE <<= 2;
  if (digitalRead(KY040_PIN_IN2))
    KY040_PREV_NEXT_CODE |= 0x02;
  if (digitalRead(KY040_PIN_IN1))
    KY040_PREV_NEXT_CODE |= 0x01;
  KY040_PREV_NEXT_CODE &= 0x0f;

  // If valid then store as 16 bit data.
  if (rot_enc_table[KY040_PREV_NEXT_CODE])
  {
    KY040_STORE <<= 4;
    KY040_STORE |= KY040_PREV_NEXT_CODE;
    // if (KY040_STORE==0xd42b) return 1;
    // if (KY040_STORE==0xe817) return -1;
    if ((KY040_STORE & 0xff) == 0x2b)
      return -1;
    if ((KY040_STORE & 0xff) == 0x17)
      return 1;
  }
  return 0;
}

/*--------------------------- MQTT ---------------------------------------*/
void mqttSetup()
{
  String res = "";
  String res2 = "";
  char cmd[80];

  // get the location name from the Jeeves server
  sprintf(cmd, STR_GET_TAG_API_FORMAT, DEVICE_ID);
  int c = REST_CLIENT.get(cmd, &res);

  sprintf(cmd, "%s%d", STR_STATUS_MESSAGE, c);
  log_out(STR_MQTT_LOG_PREFIX, cmd);

  // prepare the location for all MQTT messages coming from this sensor
  sprintf(LOCATION, "%s", c != 200 ? STR_DEFAULT_LOCATION : res.c_str());

  // prepare the topic where default messages will be received
  sprintf(CMD_TOPIC, "%s/%s", LOCATION, STR_CMD_TOPIC);

  sprintf(cmd, STR_CMD_TOPIC_LOG_FORMAT, CMD_TOPIC);
  log_out(STR_MQTT_LOG_PREFIX, cmd);

  // get the address of the MQTT broker from the Jeeves server
  res2 = REST_CLIENT.get(STR_GET_MQTT_BROKER_IP_API);

  sprintf(cmd, STR_BROKER_LOG_FORMAT, res2.c_str());
  log_out(STR_MQTT_LOG_PREFIX, cmd);

  // define the location of the MQTT broker
  sprintf(MQTT_BROKER, "%s", c != 200 ? STR_STATUS_UNKNOWN : res2.c_str());

  // Set up the MQTT client and callback
  MQTT_CLIENT.setServer(MQTT_BROKER, 1883);
  MQTT_CLIENT.setCallback(mqttCallback);
}

/*--------------------------- WIFI ---------------------------------------*/
void wifiSetup()
{
  WiFiManager wifiManager;
  // wifiManager.resetSettings(); // uncomment this to reset the device EEPROM
  wifiManager.autoConnect("JeevesConnectAP");

  //   Serial.print("\nStarting Async_AutoConnect_ESP8266_minimal on " + String(ARDUINO_BOARD));
  //   Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);
  // #if (USING_ESP32_S2 || USING_ESP32_C3)
  //   ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, "Async_AutoConnect");
  // #else
  //   ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "Async_AutoConnect");
  // #endif

  //   // ESPAsync_wifiManager.resetSettings();   //reset saved settings
  //   // ESPAsync_wifiManager.setAPStaticIPConfig(IPAddress(192,168,186,1), IPAddress(192,168,186,1), IPAddress(255,255,255,0));
  //   ESPAsync_wifiManager.autoConnect("AutoConnectAP");
  //   if (WiFi.status() == WL_CONNECTED)
  //   {
  //     Serial.print(F("Connected. Local IP: "));
  //     Serial.println(WiFi.localIP());
  //   }
  //   else
  //   {
  //     Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
  //   }
}

/*--------------------------- SERIAL ---------------------------------------*/
void serialSetup()
{
  // Setup communication with the serial monitor
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println();
  Serial.println();

  char s[255];
  sprintf(s, STR_STARTUP_MESSAGE_FORMAT, SENSOR_TYPE);
  log_out(STR_STARTUP_LOG_PREFIX, s);
  sprintf(s, STR_STARTUP_VERSION_MESSAGE_FORMAT, VERSION);
  log_out(STR_STARTUP_LOG_PREFIX, s);
  sprintf(s, STR_STARTUP_DEVICE_MESSAGE_FORMAT, DEVICE_ID);
  log_out(STR_STARTUP_LOG_PREFIX, s);
}

/*--------------------------- OTA ---------------------------------------*/
void otaSetup()
{
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
                     {
      char s[255];

      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
      {
        type = "sketch";
      }
      else
      {
        type = "filesystem";
      }

      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      sprintf(s, STR_OTA_START_UPDATE_MESSAGE, type.c_str());
      log_out(STR_OTA_LOG_PREFIX, s); });

  ArduinoOTA.onEnd([]()
                   { log_out(STR_OTA_LOG_PREFIX, STR_OTA_END_UPDATE_MESSAGE); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
      char s[255];
      sprintf(s, STR_OTA_UPDATE_PROGRESS_FORMAT, (progress / (total / 100)));
      log_out(STR_OTA_LOG_PREFIX, s); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
      char se[255];
      if (error == OTA_AUTH_ERROR) 
        sprintf(se, STR_OTA_ERROR_MESSAGE_FORMAT, error, STR_OTA_ERROR_AUTH_FAILED);
      else if (error == OTA_BEGIN_ERROR)
        sprintf(se, STR_OTA_ERROR_MESSAGE_FORMAT, error, STR_OTA_ERROR_BEGIN_FAILED);
      else if (error == OTA_CONNECT_ERROR)
        sprintf(se, STR_OTA_ERROR_MESSAGE_FORMAT, error, STR_OTA_ERROR_CONNECT_FAILED);
      else if (error == OTA_RECEIVE_ERROR)
        sprintf(se, STR_OTA_ERROR_MESSAGE_FORMAT, error, STR_OTA_ERROR_RECEIVE_FAILED);
      else if (error == OTA_END_ERROR)
        sprintf(se, STR_OTA_ERROR_MESSAGE_FORMAT, error, STR_OTA_ERROR_END_FAILED);
      log_out(STR_OTA_LOG_PREFIX, se); });

  ArduinoOTA.begin();
}

/*--------------------------- TIME/NTP ---------------------------------------*/
void timeSetup()
{
  // init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

/*--------------------------- Program ---------------------------------------*/
void setup()
{
  DEVICE_ID = ESP.getChipId(); // Get the unique ID of the ESP8266 chip

  serialSetup();
  if (USE_WIFI)
  {
    wifiSetup();
    timeSetup();
    otaSetup();
    if (USE_MQTT)
      mqttSetup();
  }
  sensorSetup();
  if (USE_MQTT)
    sensorMqttSetup();
}

void loop()
{
  // Special code to handle KY-040 rotary encoder
  // from https://www.best-microcontroller-projects.com/rotary-encoder.html
  static int8_t c, val;

  if (SENSOR_KY040)
  {
    if ((val = read_rotary()))
    {
      c += val;

      if (KY040_PREV_NEXT_CODE == 0x0b)
      {
        KY040_STATUS_CURRENT = KY040_STATUS_GOINGUP;
      }

      if (KY040_PREV_NEXT_CODE == 0x07)
      {
        KY040_STATUS_CURRENT = KY040_STATUS_GOINGDOWN;
      }
    }

    if (digitalRead(KY040_PIN_BUTTON) == 0)
    {

      delay(10);
      if (digitalRead(KY040_PIN_BUTTON) == 0)
      {
        KY040_STATUS_CURRENT = KY040_STATUS_PRESSED;
        while (digitalRead(KY040_PIN_BUTTON) == 0)
          ;
      }
    }
    // - end of KY040 debounce code
  }

  if (USE_WIFI)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (USE_MQTT)
      {
        if (!MQTT_CLIENT.connected())
        {
          reconnectMqtt();
        }
      }
    }
  }

  if (USE_MQTT)
    MQTT_CLIENT.loop(); // process any outstanding MQTT messages

  sensorUpdateReadingsQuick(); // get the data from sensors at max speed

  if (millis() - previousUpdateTime >= DELAY_MS)
  {
    sensorUpdateReadings(); // get the data from sensors
    sensorReportToMqtt();   // send messages to the MQTT broker
    sensorReportToSerial(); // print the data on the serial port
    sensorUpdateDisplay();  // update the local display, if present
    previousUpdateTime = millis();
  }

  if (millis() - splashScreenTimer >= SPLASH_SCREEN_DELAY)
  {
    ON_SPLASH_SCREEN = false;
    splashScreenTimer = millis();
  }

  if (USE_WIFI)
    ArduinoOTA.handle(); // see if there is an OTA update request
}

void reconnectMqtt()
{
  char mqtt_client_id[20];
  sprintf(mqtt_client_id, "Jeeves-%06X", ESP.getChipId());

  // Loop until we're reconnected
  while (!MQTT_CLIENT.connected())
  {
    // Attempt to connect
    if (MQTT_CLIENT.connect(mqtt_client_id))
    {
      // Once connected, publish an announcement
      sprintf(MQTT_MESSAGE_BUFFER, STR_MQTT_STARTUP_MESSAGE_FORMAT, mqtt_client_id);
      MQTT_CLIENT.publish(STATUS_TOPIC, MQTT_MESSAGE_BUFFER);
      // Resubscribe
      MQTT_CLIENT.subscribe(CMD_TOPIC);
    }
    else
    {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
