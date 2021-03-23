/**
   ESPNOW - Basic communication - Slave - Arvind Ravulavaru <https://github.com/arvindr21>

  Sketch was initially producing errors for me. To fix this, I erased the 
  device's flash memory (https://github.com/espressif/esptool/issues/348)
  Just navigate to esptool.py (/Users/YourName/Library/Arduino15/packages/esp32...)
  And run:       "" python esptool.py --chip esp32 --port /dev/cu.usbserial-0001 
  --baud 115200 --before default_reset --after hard_reset erase_flash ""
  Then it should start working

*/

#include <esp_now.h>
#include "WiFi.h"

#define CHANNEL 1 //esp now transmission channel
#define BUTTON_PIN 16 //for the trigger button
#define LED_PIN 17 //output gpio for the led
#define PWM_CHANNEL 1 //anything from 1-16
#define LED_FREQ 38000 //pwm modulation frequency, depends on receiver hardware
#define DUTY_CYCLE_RES 8 //keep at 8 bits, we don't need better resolution
#define UPLOAD_PIN 23 //button that activates uploading when pressed
#define MAX_WIFI_RETRIES 20 //number of times to try connecting to wifi before giving up

const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";


///// this section of code should also be in the sender!! ////////
#define MAX_RECORDS 298 // data size of the floats
struct measurement {
  float min_value;
  float resolution;
  String unit;
};
typedef struct measurement Measurement;
//16-bit value divided into 2 8-bit values
struct split_short {
  uint8_t high;
  uint8_t low;
};
typedef struct split_short SplitShort;
Measurement pressure = {100000.0, 1.0, "Pa"};
Measurement temperature = {0.0, 0.01, "Celsius"};
Measurement humidity = {0, 0.01, "Relative %"};

float* pressure_data;
float* temperature_data;
float* humidity_data;



////////////////// begin the code    //////////////////////
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESPNow/Basic/Slave Example");
  
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); 
  Serial.println(WiFi.softAPmacAddress());
  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv); //calls when any data is received

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, LED_FREQ, DUTY_CYCLE_RES);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
}

void loop() {
  /*
   * Just wait for a button press, and activate 
   */
  bool trigger = false;
  bool upload = false;
  while(!trigger && !upload) { 
    delay(10);
    trigger = digitalRead(BUTTON_PIN);
    upload = digitalRead(UPLOAD_PIN);
  }
  if (trigger) {
    Serial.println("Trigger pressed!");
    //activate the LED pwm pin
    ledcWrite(PWM_CHANNEL, 128);
    delay(3000);
    ledcWrite(PWM_CHANNEL, 0);
  } else if (upload) {
    Serial.println("Upload pressed!");
    if (connect_to_server()) {
      //batch upload to Otto's server...
    }
  }
}

//////////////////ESP Now and Wifi connection stuff /////////////////////
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String ssid = Prefix + Mac;
  Serial.println("SSID: " + Prefix + Mac);
  String Password = "123456789";
  bool result = WiFi.softAP(ssid.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(ssid));
  }
}

bool connect_to_server() {
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
      return false;
    }
  }
  Serial.println("Connected to the WiFi network");
  http.begin(server);
  http.addHeader("Content-Type", "application/json");
  return true;
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


/////////////////////// Data receiving methods ///////////////////////////

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  /*
   * Called as an interrupt whenever data is received on ESP Now. 
   */
  ledcWrite(PWM_CHANNEL, 0); //stop firing the IR LED once the other ESP sends data
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  float first_recd_float = float_from_data(data, humidity);
  Serial.print("Last Packet Recv Data: "); Serial.println(first_recd_float);
  Serial.println("");
}

float float_from_data(const uint8_t* a, measurement m) {
  /*
   * Go into the specified array which is made up of SplitShorts, 
   * and return the first element in float format.
   * 
   * example: float_from_short_array(hum_data, humidity)
   */
   SplitShort s = {a[0], a[1]};
   return short_to_float(s, m);
}

float short_to_float(SplitShort s, measurement m) {
  /*
   * Given a measurement type and a short, convert the short we get
   * from the internal representation of data to a float used
   * for display. 
   */
   uint16_t v = (s.high << 8) + s.low;
   float x = v*m.resolution + m.min_value;
   return x;
}
