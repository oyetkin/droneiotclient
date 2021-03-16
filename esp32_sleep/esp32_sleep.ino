/*
 * Install the following libraries: 
 * "Wifi" (builtin)
 * "HTTP Client" (builtin on some devices)
 * "Adafruit BusIO"
 * "Adafruit Unified Sensor"
 * "ESP8266 and ESP32 OLED driver for SSD1306 displays"
 * "Adafruit BME280"
 * 
 * Connections:
 * 
 */

#include <Wire.h>  
#include "WiFi.h"
#include "HTTPClient.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SH1106Wire.h"
#include "time.h"

// PINOUTS
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_BUTTON GPIO_NUM_13 //must be one of the RTC gpios

#define BUTTON_PIN_BITMASK 0x000002000 // 2^OLED_BUTTON in hex
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

/* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SLEEP  30


//WIFI SETTINGS
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const char* ntpServer = "pool.ntp.org";

//DEVICE SETTINGS
const String sensor_name = "Arjun_weather_kit";
float lat = 32.636462;
float lon = -117.095660;

HTTPClient http;
SH1106Wire display(0x3c, SDA, SCL); //7 bit address only
Adafruit_BME280 bme;
RTC_DATA_ATTR float hum = -1.0;
RTC_DATA_ATTR float temp = -1.0;
RTC_DATA_ATTR float pres = -1.0;

void connect_to_server();
unsigned long getTime();
String post_to_string(float, String, bool, bool);
void display_readings(SH1106Wire*, float, float, float);
void display_and_sleep(SH1106Wire*, int);

void setup() {
  /*
   * Connect to wifi and start sensor modules. 
   * Network name and password should be specified manually
   * above. 
   */
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wake up");

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    display_and_sleep(&display, 2000);
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    boot_and_post_sensors();
  } else {
    Serial.println("other");
  }
  
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void boot_and_post_sensors() {
  /*
   * Call this when the sensor is booted by time. Activates a wifi connection, 
   * loads the sensors, gets their readings, and posts the data. 
   */
  //initialize wifi and servers 
  connect_to_server();
  configTime(0, 0, ntpServer); //time
  if (!bme.begin(0x76)) {Serial.println("Could not find a valid BME sensor!");} //bme
  pinMode(OLED_BUTTON, INPUT);

  //Now read the sensors
  Serial.println("Read sensors");
  delay(2000);
  hum = bme.readHumidity();
  temp = bme.readTemperature();
  pres = bme.readPressure();

  //And post to the server
  if (isnan(hum) || isnan(temp) || isnan(pres)) {
    Serial.println("Failed to read!");
  } else {
    //Print readings to the serial
    Serial.println("Humidity (RH%): " + String(hum));
    Serial.println("Temperature (C): " + String(temp));
    Serial.println("Pressure (Pa): " + String(pres));
    //Post all values separately and report error
    int response_1 = http.POST(post_to_string(temp, "Celsius", true, true));
    int response_2 = http.POST(post_to_string(hum, "RH%", true, true));
    int response_3 = http.POST(post_to_string(pres, "Pa", true, true));
    if ((response_1 != 200) or (response_2 != 200) or (response_3 != 200)) {
      Serial.println("HTTP Post error");
    }
  }
}

void display_and_sleep(SH1106Wire* d, int display_len) {
  /*
   * Call this when the sensor is booted by external trigger. 
   * Activate the OLED display, show readings, and then turn off the display.
   */
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);
  display_readings(d, hum, temp, pres);
  delay(display_len);
  d->displayOff();
}


void connect_to_server() {
  /*
   * Connect to the wifi and Otto's server
   */
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

void display_readings(SH1106Wire* d, float humidity, float temp, float pressure) {
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


void loop() {
  // Leave this empty
}
