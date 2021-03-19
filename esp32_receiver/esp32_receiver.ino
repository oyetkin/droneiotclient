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

#define CHANNEL 1
#define BUTTON_PIN 16
#define LED_PIN 17
#define PWM_CHANNEL 1
#define LED_FREQ 38000
#define DUTY_CYCLE_RES 8


///// this section of code should also be in the sender!! ////////
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
///////////////////////////////////////////////////////////


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
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
  while(!digitalRead(BUTTON_PIN)) { 
    delay(10);
  }
  Serial.println("Button pressed!");
  //modulate on and off at 38kHz
  ledcWrite(PWM_CHANNEL, 128);
  //digitalWrite(LED_PIN, HIGH);
  delay(3000);
  ledcWrite(PWM_CHANNEL, 0);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  /*
   * Called as an interrupt whenever data is received on ESP Now. 
   */
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
