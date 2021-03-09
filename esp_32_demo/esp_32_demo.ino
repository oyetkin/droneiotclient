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

DHT dht(PIN_DHT11, DHTTYPE);
SSD1306Wire display(0x3c, SDA, SCL);
Adafruit_BMP085 bmp;
HTTPClient http;

//WIFI SETTINGS
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const char* ntpServer = "pool.ntp.org";

//DEVICE SETTINGS
const String sensor_name = "Arjun_weather_kit";
float lat = 35.1222;
float lon = -121.8431;

//state variables
unsigned long epochTime;

void setup() {
  /*
   * Connect to wifi and start sensor modules. 
   * Network name and password should be specified manually
   * above. 
   */
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Wifi has begun");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  http.begin(server);
  http.addHeader("Content-Type", "application/json");
  //Start the DHT driver module
  dht.begin();
  //Start the OLED module
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  //start time
  configTime(0, 0, ntpServer);
  //start the bmp sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}

unsigned long getTime() {
  /*
   * Get the current time, in epoch time. Returns 0 if unable to obtain time. 
   * From: https://randomnerdtutorials.com/epoch-unix-time-esp32-arduino/
   */
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

String post_to_string(float measurement, String unit, bool incl_time, bool incl_location) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"" + sensor_name + "\",\"unit\":\"" + unit + "\",\"value\":\"" + String(measurement);
  if (incl_time) {
    //time_t now;
    unsigned long curr_time = getTime();
    if (curr_time > 0) {
      json = json + "\",\"timestamp\":\"" + String(curr_time);
    }
  }
  if (incl_location) {
    json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    
  }
  json = json + "\"}";
  return json;
}

void display_readings(float humidity, float temp, float pressure) {
    /*
     * Display the current humidity and temperature on the OLED. 
     */
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 12, "Humidity: " + String(humidity) + "%");
    display.drawString(0, 28, "Temp: " + String(temp) + "C");
    display.drawString(0, 44, "Pressure: " + String(pressure/) + "Pa");
    display.display();
}

void loop() {
  /*
   * Read humidity and temperature data from the DHT11 sensor; update the OLED
   * and POST the data. 
   */
  while(true) {
    delay(2000);
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float p = bmp.readPressure();
    if (isnan(h) || isnan(t)) {
     Serial.println(F("Failed to read from DHT sensor!"));
    } else {
      //Print readings to the serial
      Serial.println("Humidity (RF%): " + String(h));
      Serial.println("Temperature (C): " + String(t));
      Serial.println("Pressure (Pa): " + String(p));
      //Post both values separately and report error
      int response_1 = http.POST(post_to_string(t, "Celsius", true, true));
      int response_2 = http.POST(post_to_string(h, "RH%", true, true));
      int response_3 = http.POST(post_to_string(p, "Pa", true, true));
      if ((response_1 != 200) or (response_2 != 200)) {
        Serial.print("HTTP Post error: ");
        Serial.println(http.getString()); 
      }
      //display on OLED
      display_readings(h, t, p);
    }
  }
}
