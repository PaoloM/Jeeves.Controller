// ==========================================================================================
// Project: Jeeves.Controller
// File:    sensor.h
// Version: 0.9
// Date:    2024-03-03
// License: MIT https://github.com/PaoloM/Jeeves.Controller/blob/main/LICENSE
// ==========================================================================================


///////////////////////////////// SSD1306 I2C OLED DISPLAY //////////////////////////////////
// Default connections are SCL -> D1 and SDA -> D2, 3.3v to VCC
#include <U8g2lib.h> // olikraus/U8g2@^2.32.6
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define          SSD1306_SCREEN_WIDTH         128 // OLED display width, in pixels
#define          SSD1306_SCREEN_HEIGHT        64  // OLED display height, in pixels
#define          SSD1306_PIN_SCL              D1
#define          SSD1306_PIN_SDA              D2

///////////////////////////////// DHTxx TEMPERATURE AND HUMIDITY SENSOR //////////////////////
// (add some info)
#include <DHT.h> // adafruit/DHT sensor library@^1.4.3
#include <DHT_U.h>

#define DHT_PIN D3
#define          DHT_TYPE                     DHT11     // DHT 11
//#define          DHT_TYPE                     DHT22     // DHT 22 (AM2302)
//#define          DHT_TYPE                     DHT21     // DHT 21 (AM2301)
#define          DHT_TEMPERATURE_IN_FARENHEIT true

///////////////////////////////// HD44780 16x2 LCD DISPLAY ///////////////////////////////////
// (add some info)
#include <LiquidCrystal_I2C.h>  // marcoschwartz/LiquidCrystal_I2C@^1.1.4                

#define          HD44780_SCREEN_ADDRESS       0x27 // 0x27 for 16x2 characters

///////////////////////////////// KY-040 ROTARY ENCODER //////////////////////////////////////
// (add some info)
#define          KY040_PIN_IN1                D5   // CLK
#define          KY040_PIN_IN2                D6   // DT
#define          KY040_PIN_BUTTON             D7   // SW
#define          KY040_STATUS_IDLE            0
#define          KY040_STATUS_PRESSED         1
#define          KY040_STATUS_GOINGUP         2
#define          KY040_STATUS_GOINGDOWN       3
static  uint8_t  KY040_PREV_NEXT_CODE       = 0;
static  uint16_t KY040_STORE                = 0;
static  int      KY040_STATUS_CURRENT       = KY040_STATUS_IDLE;