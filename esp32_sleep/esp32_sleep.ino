/*
 * Install the following libraries: 
 * "Wifi" (builtin)
 * "HTTP Client" (builtin on some devices)
 * "Adafruit BMP085 Library"
 * "Adafruit BusIO"
 * "Adafruit Unified Sensor"
 * "DHT sensor library"
 * "ESP8266 and ESP32 OLED driver for SSD1306 displays"
 */

#include <Wire.h>  
#include "WiFi.h"
#include "DHT.h"
#include "HTTPClient.h"
#include "SSD1306Wire.h"
#include "time.h"
#include <Adafruit_BMP085.h>

// PINOUTS
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22
#define PIN_DHT11 4
#define DHTTYPE DHT11 // type of DHT device (DHT 11 comes with amazon weather station)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

void setup() {
  /*
   * Connect to wifi and start sensor modules. 
   * Network name and password should be specified manually
   * above. 
   */
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wake up");
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // Leave this empty
}
