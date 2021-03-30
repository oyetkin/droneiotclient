// script to run the arduino like an oscope

#define IR_RECEIVE_PIN 15
#include <IRremote.h>
#define STATUS_PIN 2 //internal LED - flash when sending


#include "HTTPClient.h"
#include "time.h"

HTTPClient http;
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
const char* ntpServer = "pool.ntp.org";

float lat = 38;
float lon = -130;

void setup() {
  Serial.begin(115200);
  //IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  //pinMode(STATUS_PIN, OUTPUT);
  connect_to_server();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (connect_to_server()) {
    float read_value = 12;
    Serial.println(create_post_string(read_value, false, true));
  
    int response_3 = http.POST(create_post_string(read_value, false, true));
    Serial.println(response_3);
  }
  delay(5000);
}


bool connect_to_server() {
  /*
   * Connect to the wifi and Otto's server
   */
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    counter++;
    if (counter > 10) {
      Serial.println("Failed to connect!");
      return false;
    }
  }
  Serial.println("Connected to the WiFi network");
  http.begin(server);
  http.addHeader("Content-Type", "application/json");
  return true;
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



String create_post_string(float value, bool incl_time, bool incl_location) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"arjun_test\", \"measurement_name\":\"temp_test\"";
  json = json + ",\"unit\":\"C_test\",\"value\":\"" + String(value);
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
