/**
   ESPNOW - Basic communication - Slave - Arvind Ravulavaru <https://github.com/arvindr21>

  Sketch was initially producing errors for me. To fix this, I erased the 
  device's flash memory (https://github.com/espressif/esptool/issues/348)
  Just navigate to esptool.py (/Users/YourName/Library/Arduino15/packages/esp32...)
  And run:       "" python esptool.py --chip esp32 --port /dev/cu.usbserial-0001 
  --baud 115200 --before default_reset --after hard_reset erase_flash ""
  Then it should start working

  Libraries:
  Dictionary by Anatoli Arkhipenko

*/

#include <esp_now.h>
#include "WiFi.h"
#include "HTTPClient.h"
#include <Dictionary.h>

#define CHANNEL 1 //esp now transmission channel, anything from 1-16
#define TRIGGER_PIN GPIO_NUM_16 //for the trigger button
#define LED_PIN GPIO_NUM_17 //output gpio for the led
#define UPLOAD_PIN GPIO_NUM_23 //button that activates uploading when pressed
#define PWM_CHANNEL 1 //anything from 1-16
#define LED_FREQ 38000 //pwm modulation frequency, depends on receiver hardware
#define DUTY_CYCLE_RES 8 //keep at 8 bits, we don't need better resolution
#define MAX_WIFI_RETRIES 10 //number of times to try connecting to wifi before giving up
#define LED_ON_TIME 3000 //how long to turn on the IR LEDs in millis
#define INVALID_LOCATION -181.0 //invalid lat/lon
#define MAX_SENSORS 3 //maximum number of sensors we can read from at a time

const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor/batch";
HTTPClient http;

///// this section of code should also be in the sender!! ////////
#define MAX_RECORDS 298 // data size of the floats
struct measurement {
  float min_value; //min recordable by senseor
  float resolution; //sensor resolution
  String unit; //e.g. "Celsius"
  String type; //e.g. "temperature"
  String hardware_name; //optionally, hardware used to collect the measurement
};
typedef struct measurement Measurement;

struct record {
  String mac_address;
  String device_name;
  float lat;
  float lon;
  Measurement m;
  float* sensor_data;
  int n_records_recd;
};
typedef struct record Record;

Record all_records[MAX_SENSORS];

//16-bit value divided into 2 8-bit values
struct split_short {
  uint8_t high;
  uint8_t low;
};
typedef struct split_short SplitShort;

int n_sensors_received = 0;

////////////////// begin the code    //////////////////////
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  //Start off in ESP mode to begin with
  setUpESPNow();

  pinMode(TRIGGER_PIN, INPUT);
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
    trigger = digitalRead(TRIGGER_PIN);
    upload = digitalRead(UPLOAD_PIN);
  }
  if (trigger) {
    Serial.println("Trigger pressed!");
    setUpESPNow();
    //activate the LED pwm pin
    ledcWrite(PWM_CHANNEL, 128);
    delay(LED_ON_TIME);
    ledcWrite(PWM_CHANNEL, 0);
  } else if (upload) {
    Serial.println("Upload pressed!");
    if (connect_to_server()) {
      
      int response_1 = 0; //http.POST(multi_posts_from_array(
      //  "arjun_test", temperature_data, 10, temperature, INVALID_LOCATION, INVALID_LOCATION, false));
      
      if (response_1 != 200) {
        Serial.println("HTTP Post error");
      } else {
        Serial.println("Successfully posted.");
      }
      http.end();
    }
  }
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
  Serial.print("Packet Recv from: "); Serial.println(macStr);

  int record_idx = find_record_by_mac(macStr);  
  if (record_idx == -1) { //if we don't have this mac address, it is the first packet from a new device
    Serial.println("Unpack sensor metadata");
    decodeMetaData(data, macStr, data_len);
  } else { //otherwise, it's data; get the parameters to interpret that data based on its mac address
    Measurement m = all_records[record_idx].m;
    float *a = all_records[record_idx].sensor_data;
    int data_head = all_records[record_idx].n_records_recd;
    Serial.println("Process sensor data using " + m.type);
    
    for (int i=0; i < data_len; i+=2) {
      SplitShort s = {data[i], data[i+1]};
      a[data_head + i/2] = short_to_float(s, m);
    }
    Serial.print("Most recent reading sent: "); Serial.println(a[data_head]);  
    all_records[record_idx].n_records_recd += data_len/2; //increment last position where receiving data    
  }
}

void decodeMetaData(const uint8_t *metadata, String mac_str, int data_len) {
  /*
   * Decode the metadata packet. Memcopies the metadata piece by piece
   * into a set of variables. 
   */
  char my_device_name[32] = {};
  float my_lat = -181.0;
  float my_lon = -181.0;
  float my_min_value = -181.0;
  float my_resolution = -181.0;
  char my_type[32] = {};
  char my_unit[32] = {};
  char my_hardware[32] = {};
  int n_packets = 0;
  
  memcpy(&my_device_name[0], metadata, 32); // name
  memcpy(&my_lat, &metadata[32], sizeof(float)); // lat
  memcpy(&my_lon, &metadata[36], sizeof(float)); // lon
  memcpy(&my_type[0], &metadata[40], 32); // name
  memcpy(&my_unit[0], &metadata[72], 32); // name
  memcpy(&my_min_value, &metadata[104], sizeof(float)); // lat
  memcpy(&my_resolution, &metadata[108], sizeof(float)); // lon
  memcpy(&my_hardware[0], &metadata[112], 32); // hardware
  memcpy(&n_packets, &metadata[144], sizeof(int)); // n packets

  Serial.println("Device name: " + String(my_device_name));
  Serial.println("Location: " + String(my_lat) + ", " + String(my_lon));
  Serial.println("Measurement: " + String(my_type) + ", " + String(my_unit));
  Serial.println("Measurement min, res: " + String(my_min_value) + ", " + String(my_resolution));

  //Use all this information to create a Record, containing a zero-instantiated data array
  float data_array[MAX_RECORDS] = {0.0};
  all_records[n_sensors_received] = {mac_str, my_device_name, my_lat, my_lon, 
    {my_min_value, my_resolution, my_type, my_unit, my_hardware}, data_array};
  n_sensors_received += 1;

}

int find_record_by_mac(String mac_to_match) {
  /*
   * Given a mac address, return the index of that record
   */
  for (int i=0; i<MAX_SENSORS; i++) {
    if (all_records[i].mac_address.equals(mac_to_match)) {
      return i;
    }
  }
  return -1;
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

/////////////////////////////////// Lower level wifi post methods //////////////////////////////

String multi_posts_from_array(String device_name, float* values, int n_values, Measurement m, float lat, float lon, bool incl_hardware) {
  /*
   * Convert the array of input measurements into a json-style list
   */
  String out = "[";
  for (int i = 0; i < n_values-1; i++) {
    String post = create_post_string(device_name, values[i], m, lat, lon, incl_hardware);
    out = out + post + ", ";
  }
  String post = create_post_string(device_name, values[n_values-1], m, lat, lon, incl_hardware);
  out = out + post + "]";
  return out;
}


String multi_post_string(String* posts, int n_posts) {
  /*
   * Take a set of json-formatted post strings (using create_post_string method) and combine
   * them into a json-formatted list, in order to batch post them. 
   */
   String out = "[";
   for (int i=0; i<n_posts-1; i++) {
    out = out + posts[i] + ", ";
   }
   out = out + posts[n_posts-1] + "]";
   return out;
}

String create_post_string(String device_name, float value, Measurement m, float lat, float lon, bool incl_hardware) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m.type;
  json = json + "\",\"unit\":\"" + m.unit + "\",\"value\":\"" + String(value);

  if (lat != INVALID_LOCATION && lon != INVALID_LOCATION) {
    json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    
  }
  if (incl_hardware) {
    json = json + "\",\"hardware\":\"" + m.hardware_name;
  }
  json = json + "\"}";
  return json;
}



//////////////////ESP Now and Wifi setup /////////////////////

void setUpESPNow() {
  /*
   * Sets device in Access Point mode and configures other ESP Now stuff
   */
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv); //calls when any data is received
}


bool connect_to_server() {
  /*
   * Connect to the wifi and Otto's server. Return true if successful
   */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
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
