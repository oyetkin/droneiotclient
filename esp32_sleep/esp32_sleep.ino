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
#include "OLEDDisplay.h"
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
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */

HTTPClient http;

//WIFI SETTINGS
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const char* ntpServer = "pool.ntp.org";

//DEVICE SETTINGS
const String sensor_name = "Arjun_weather_kit";
float lat = 32.636462;
float lon = -117.095660;

DHT dht(PIN_DHT11, DHTTYPE);
SSD1306Wire display(0x3c, SDA, SCL);
Adafruit_BMP085 bmp;

void connect_to_server();
unsigned long getTime();
String post_to_string(float, String, bool, bool);
void display_readings(SSD1306Wire*, float, float, float);
void display_and_sleep(SSD1306Wire*, float, float, float, int);

void setup() {
  /*
   * Connect to wifi and start sensor modules. 
   * Network name and password should be specified manually
   * above. 
   */
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wake up");
  //initialize wifi and servers 
  connect_to_server();
  dht.begin();
  configTime(0, 0, ntpServer); //time
  if (!bmp.begin()) {Serial.println("Could not find a valid BMP085 sensor!");} //bmp

  //Now read the sensors
  Serial.println("Read sensors");
  delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float p = bmp.readPressure();

  //And post to the server
  if (isnan(h) || isnan(t) || isnan(p)) {
    Serial.println("Failed to read!");
  } else {
    //Print readings to the serial
    Serial.println("Humidity (RH%): " + String(h));
    Serial.println("Temperature (C): " + String(t));
    Serial.println("Pressure (Pa): " + String(p));
    //Post all values separately and report error
    int response_1 = http.POST(post_to_string(t, "Celsius", true, true));
    int response_2 = http.POST(post_to_string(h, "RH%", true, true));
    int response_3 = http.POST(post_to_string(p, "Pa", true, true));
    if ((response_1 != 200) or (response_2 != 200) or (response_3 != 200)) {
      Serial.println("HTTP Post error");
    }
    //display on OLED
    display_and_sleep(&display, h, t, p, 2000);
  }

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // Leave this empty
}


void connect_to_server() {
  WiFi.begin(ssid, password);
  Serial.println("Wifi has begun");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  http.begin(server);
  http.addHeader("Content-Type", "application/json");
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

void display_readings(SSD1306Wire* d, float humidity, float temp, float pressure) {
    /*
     * Display the current humidity and temperature on the OLED. 
     */
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0, 12, "Humidity: " + String(humidity) + "%");
  d->drawString(0, 28, "Temp: " + String(temp) + "C");
  d->drawString(0, 44, "Pressure: " + String(pressure) + "Pa");
  d->display();
}

void display_and_sleep(SSD1306Wire* d, float h, float t, float p, int display_len) {
  /*
   * Activate display, show readings, and then turn off the display
   */
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);
  display_readings(d, h, t, p);
  delay(display_len);
  d->displayOff();
}
