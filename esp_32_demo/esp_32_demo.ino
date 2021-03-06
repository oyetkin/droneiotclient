#include "WiFi.h"
#include "HTTPClient.h"

const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const String sensor_name = "Arjun_weather_kit";
const String unit = "Celsius";
float lat = 35.1222;
float lon = -121.8431;

int post(String key, String unit, int value, bool incl_time=false) {
  Serial.println("ok");
}

void setup() {
  /*
   * Just connects to wifi. 
   */
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Wifi has begun");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

String post_to_string(float measurement, bool incl_time, bool incl_location) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and unit are hard-coded at the top of this sketch.
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

void loop() {
  /*
   * Posts every few seconds with some data. 
   */
  HTTPClient http;
  http.begin(server);
  http.addHeader("Content-Type", "application/json");
  while(true) {
    int response_code = http.POST(post_to_string(23.2, true, false));
    Serial.println(response_code);
    delay(2000);
  }
}
