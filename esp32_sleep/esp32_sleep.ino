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
#define DEVICE_MODE_SELECT_PIN GPIO_NUM_27
#define TRIGGER_PIN GPIO_NUM_14
#define OLED_BUTTON GPIO_NUM_13 //must be one of the RTC gpios
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define WAKE_PIN_BITMASK 0x000006000 // 2^OLED_BUTTON + 2^TRIGGER_PIN in hex
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define MAX_RECORDS 298 //298 is exactly enough for 3 float arrays (each having 298 elements)

#define TIME_TO_SLEEP  30 //time to sleep in seconds
#define DISPLAY_LEN 2000 //time to show OLED in millis
#define MAX_WIFI_RETRIES 20 //number of times to try connecting to wifi before giving up

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
RTC_DATA_ATTR float hum[MAX_RECORDS];
RTC_DATA_ATTR float temp[MAX_RECORDS];
RTC_DATA_ATTR float pres[MAX_RECORDS];
bool device_mode_wifi; //1 for wifi, 0 for trigger


void setup() {
  /*
   * All the device logic is contained here
   */
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wake up");
  pinMode(DEVICE_MODE_SELECT_PIN, INPUT);
  device_mode_wifi = digitalRead(DEVICE_MODE_SELECT_PIN);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if (device_mode_wifi) {
    Serial.println("Mode: Wifi");
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { // either trigger or button
      display_and_sleep(&display);
      boot_and_post_sensors();
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
      boot_and_post_sensors();
    } else { // usually on device startup
      show_wakeup(&display, device_mode_wifi);
      boot_and_post_sensors();
    }
  } else {
    Serial.println("Mode: Trigger");
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
      int pin_triggered = log(esp_sleep_get_ext1_wakeup_status())/log(2);
      if (pin_triggered == TRIGGER_PIN) {
        //batch upload here
      } else if (pin_triggered == OLED_BUTTON) {
        display_and_sleep(&display); 
      }
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
      //save sensor values
    } else { // usually on device startup
      //save sensors in array
      show_wakeup(&display, device_mode_wifi);
    } 
  }
  
  //Entering sleep
  go_to_sleep(device_mode_wifi);
}


void go_to_sleep(bool wifi_mode) {
  /*
   * Send the device to sleep. Wifi_mode controls which bitmask to use - if in wifi mode,
   * it won't wake up from an external trigger.
   */
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void boot_and_post_sensors() {
  /*
   * Call this when the sensor is booted by time. Activates a wifi connection, 
   * loads the sensors, gets their readings, and posts the data. 
   */
  //initialize wifi and servers 
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME sensor!");
    go_to_sleep(1);
  }

  //Now read the sensors
  Serial.println("Read sensors");
  delay(2000);
  float curr_hum = bme.readHumidity();
  float curr_temp = bme.readTemperature();
  float curr_pres = bme.readPressure();
  if (isnan(curr_hum) || isnan(curr_temp) || isnan(curr_pres)) {
    Serial.println("Failed to read!");
    go_to_sleep(1);
  } 
  push_back(hum, curr_hum, MAX_RECORDS);
  push_back(temp, curr_temp, MAX_RECORDS);
  push_back(pres, curr_pres, MAX_RECORDS); 
  //Print readings to the serial
  Serial.println("Humidity (RH%): " + String(curr_hum));
  Serial.println("Temperature (C): " + String(curr_temp));
  Serial.println("Pressure (Pa): " + String(curr_pres));

  //Post all values separately and report error
  connect_to_server();
  configTime(0, 0, ntpServer);
  int response_1 = http.POST(post_to_string(curr_temp, "Celsius", true, true));
  int response_2 = http.POST(post_to_string(curr_hum, "RH%", true, true));
  int response_3 = http.POST(post_to_string(curr_pres, "Pa", true, true));
  if ((response_1 != 200) or (response_2 != 200) or (response_3 != 200)) {
    Serial.println("HTTP Post error");
  }
}

void push_back(float* a, float v, int a_len) {
  /*
   * Uses the specified array as a circular buffer. Pushes all entries
   * of a back, deleting its last element. Puts v at the front of array. 
   * You can access the most recent value with a[0]. 
   */
   //a is an address. int* p = (int *)a assigns it to a pointer. p[1] is the next address
   memcpy(&a[1], a, sizeof(a[0])*(a_len-1));
   a[0] = v;
}

void connect_to_server() {
  /*
   * Connect to the wifi and Otto's server
   */
  WiFi.begin(ssid, password);
  Serial.println("Wifi has begun");
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    counter++;
    if (counter > MAX_WIFI_RETRIES) {
      Serial.println("Failed to connect!");
      go_to_sleep(1);
    }
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

void show_wakeup(SH1106Wire* d, bool wifi_mode) {
  /*
   * Just show a message on the OLED which mode you're in
   */
  d->init();
  d->flipScreenVertically();
  
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  String wake_string = "Starting up in ";
  if (wifi_mode) {
    wake_string = wake_string + "Wifi Mode";
  } else {
    wake_string = wake_string + "Remote Trigger Mode";
  }
  Serial.println(wake_string);
  d->drawString(0, 12, wake_string);
  d->display();
  delay(DISPLAY_LEN);
  d->displayOff();
}

void display_and_sleep(SH1106Wire* d) {
  /*
   * Call this when the sensor is booted by external trigger. 
   * Activate the OLED display, show readings, and then turn off the display.
   */
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);
  display_readings(d, hum[0], temp[0], pres[0]);
  delay(DISPLAY_LEN);
  d->displayOff();
}

void display_readings(SH1106Wire* d, float humidity, float temperature, float pressure) {
    /*
     * Display the current humidity and temperature on the OLED. 
     */
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0, 12, "Humidity: " + String(humidity) + "%");
  d->drawString(0, 28, "Temp: " + String(temperature) + "C");
  d->drawString(0, 44, "Pressure: " + String(pressure) + "Pa");
  d->display();
}

void loop() {}
