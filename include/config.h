// ==========================================================================================
// Project: Jeeves.Controller
// File:    config.h
// Version: 0.9
// Date:    2024-03-03
// License: MIT https://github.com/PaoloM/Jeeves.Controller/blob/main/LICENSE
// ==========================================================================================

#define     DEBUG                    true                   // true to show messages on the Serial monitor
#define     USE_GENERIC              true                   // use the generic topic
#define     DELAY_MS                 10000                  // milliseconds between sensor readings
#define     SPLASH_SCREEN_DELAY      10000                  // milliseconds before splash screen dismissal

// Template info (do not change after creating the initial structure)
#define     BOILERPLATE_VERSION      "0.9"                  // version and date of the boilerplate template 
#define     BOILERPLATE_DATE         "2024-03-03"           // that this implementation is based on

// Jeeves server connection
const char* JEEVES_SERVER          = "jeeves";              // IP address of your Jeeves server
int         JEEVES_SERVER_PORT     = 8080;                  // Listening port of your Jeeves server

// Onboard sensors configuration
#define     SENSOR_SSD1306           false                  // use the SSD1306 124x64 OLED display
#define     SENSOR_HD74480           false                  // use the HD44780 16x2 LCD display
#define     SENSOR_KY040             false                  // use the KY-040 rotary encoder
#define     SENSOR_DHT               false                  // use the DHTxx temperature and humidity sensor
#define     SENSOR_PMS5003           false                  // TODO use the PMS5003 Digital Particle Concentration Laser Sensor 