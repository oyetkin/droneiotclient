#include <Wire.h>  
#include "WiFi.h"
#include "DHT.h"
#include "HTTPClient.h"
#include "SSD1306Wire.h"
//#include "images.h"

// PINOUTS
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22
#define PIN_DHT11 4
#define DHTTYPE DHT11
DHT dht(PIN_DHT11, DHTTYPE);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

SSD1306Wire display(0x3c, SDA, SCL);
HTTPClient http;

//WIFI SETTINGS
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";

//DEVICE SETTINGS
const String sensor_name = "Arjun_weather_kit";
float lat = 35.1222;
float lon = -121.8431;


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
}

String post_to_string(float measurement, String unit, bool incl_time, bool incl_location) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"" + sensor_name + "\",\"unit\":\"" + unit + "\",\"value\":\"" + String(measurement);
  if (incl_time) {
    time_t now;
    time(&now);
    json = json + "\",\"timestamp\":\"" + String(now);
  }
  if (incl_location) {
    json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    
  }
  json = json + "\"}";
  return json;
}

void display_readings(float humidity, float temp) {
    /*
     * Display the current humidity and temperature on the OLED. 
     */
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 12, "Humidity: " + String(humidity) + "%");
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 30, "Temp: " + String(temp) + "C");
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
    if (isnan(h) || isnan(t)) {
     Serial.println(F("Failed to read from DHT sensor!"));
    } else {
      Serial.print("Humidity (RF%): ");
      Serial.println(h);
      Serial.print("Temperature (C): ");
      Serial.println(t);
      int response_1 = http.POST(post_to_string(t, "Celsius", true, true));
      int response_2 = http.POST(post_to_string(h, "RF%", true, true));
      if ((response_1 != 200) or (response_2 != 200)) {
        Serial.println("HTTP Post error");        
      }
      display_readings(h, t);
    }
  }
}
