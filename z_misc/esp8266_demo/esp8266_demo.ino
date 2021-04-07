/*
 * Installing the board driver:
 * Follow these instructions:
 * https://arduino-esp8266.readthedocs.io/en/latest/installing.html
 * Use the ESP8266 Generic board.
 * 
 * If you have Mac OSX Big Sur, you may run into an error that 
 * pyserial isn't installed. Resolve it as follows:
 * -open ~/Library/Arduino15/packages/esp8266/hardware/esp8266/2.7.4/tools/pyserial/serial/tools/list_ports_osx.py
 * -comment out lines 29 and 30 and append these lines:
 * iokit = ctypes.cdll.LoadLibrary('/System/Library/Frameworks/IOKit.framework/IOKit')
 * cf = ctypes.cdll.LoadLibrary('/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation')
 * Reference: https://forum.arduino.cc/index.php?topic=702144.0
 * 
 * Set the board to "Generic ESP 8266 Module"
 * 
 * Libraries to install:
 * -Wire should be built in
 * -Adafruit Unified Sensor
 * -Adafruit BME280
 * -ESP8266 and ESP32 OLED Driver for SSD1306 Displays
 * -ESP8266Wifi and ESP8266HTTPClient should be built-in.
 * -time should be built-in
 * 
 * Pins:
 * D1 to SCL pins of OLED, and BME
 * D2 to SDA pins of OLED and BME
 * 3V3 and GND to PWR and GND rails
 * PWR and GND rails to VIN and GND of OLED and BME
 * RST to Node A
 * D0 to Node A
 * Follow circuit diagram to wire the button. 
 * 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SH1106Wire.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include "time.h"


#define RTCMEMORYSTART 65
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define PIN_SDA 2
#define PIN_SCL 1
#define SEALEVELPRESSURE_HPA (1013.25)
#define DISPLAY_TIME 2000


typedef struct {
  float temp;
  float hum;
  float pres;
} rtcStore;

rtcStore rtcMem;

//WIFI SETTINGS
char* ssid = "ATT5yX6g8p"; //library requires this to not be const
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const char* ntpServer = "pool.ntp.org";

//DEVICE SETTINGS
const String sensor_name = "Arjun_esp8266";
float lat = 32.636462;
float lon = -117.095660;

//Init devices
HTTPClient http;
Adafruit_BME280 bme;
SH1106Wire display(0x3c, SDA, SCL); //7 bit address only


//float hum = -1.0;
//float temp = -1.0;
//float pres = -1.0;

void setup() {
  pinMode(14, INPUT);
  int button = digitalRead(14);
  Serial.begin(115200);
  Serial.println();
  Serial.println("wake up");
  
  if (button == LOW) { // button pressed
    Serial.println("button was pressed");
    display_and_sleep(&display, DISPLAY_TIME);
  } else { // timer
    Serial.println("wake from timer.");
    boot_and_post_sensors();
  }
  
  Serial.println("sleep now");
  delay(1000);
  Serial.flush();
  ESP.deepSleep(30e6);
}

void boot_and_post_sensors() {
  //connect_to_server();
  //configTime(0, 0, ntpServer); //time
  bme.begin(0x76); 
  writeToRTCMemory(bme.readTemperature(), bme.readHumidity(), bme.readPressure());
  Serial.println("Humidity (RH%): " + String(rtcMem.hum));
  Serial.println("Temperature (C): " + String(rtcMem.temp));
  Serial.println("Pressure (Pa): " + String(rtcMem.pres));
  Serial.println(post_to_string(rtcMem.temp, "Celsius", false, true));
  int response_1 = http.POST("{\"key\":\"invalid_sensor_test\"}");
  int response_2 = http.POST(post_to_string(rtcMem.hum, "RH%", false, true));
  int response_3 = http.POST(post_to_string(rtcMem.pres, "Pa", false, true));
  
  if ((response_1 != 200) or (response_2 != 200) or (response_3 != 200)) {
    Serial.print("HTTP Post error ");
    Serial.println(http.errorToString(response_1).c_str());
  }
}

void readFromRTCMemory() {
  /*
   * Read the values stored in RTC Memory and store it in the 
   * rtcMem struct (which is empty on boot)
   */
  system_rtc_mem_read(RTCMEMORYSTART, &rtcMem, sizeof(rtcMem));
  Serial.print("read: ");
  Serial.println(rtcMem.temp);
  yield();
}

void writeToRTCMemory(float temp, float hum, float pres) {
  /*
   * Set fields of the rtcMem struct and then write the struct
   * to RTC Memory
   */
  rtcMem.temp = temp;
  rtcMem.hum = hum;
  rtcMem.pres = pres;
  system_rtc_mem_write(RTCMEMORYSTART, &rtcMem, 4);
  yield();
}

void display_and_sleep(SH1106Wire* d, int display_len) {
  /*
   * Call this when the sensor is booted by external trigger. 
   * Activate the OLED display, show readings, and then turn off the display.
   */
  readFromRTCMemory();
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);
  display_readings(d, rtcMem.hum, rtcMem.temp, rtcMem.pres);
  delay(display_len);
  d->displayOff();
}

void connect_to_server() {
  /*
   * Start the wifi connection
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
  /*if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return(0);
  }*/
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
  Serial.println("Never called...");
}
