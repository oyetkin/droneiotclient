/*
 * Notes...
 * 
 * 
 * Author: Arjun Tambe, Analytical Mechanical Associates
 * Source: https://github.com/oyetkin/droneiotclient
 * 
 * Credit for some portions of code: 
 *    ESP-Now Demo by Arvind Ravulavaru <https://github.com/arvindr21>
*/

#include <esp_now.h>
#include "WiFi.h"
#include "HTTPClient.h"

#define CHANNEL 1 //esp now transmission channel, anything from 1-16
#define TRIGGER_PIN GPIO_NUM_16 //for the trigger button
#define LED_PIN GPIO_NUM_17 //output gpio for the led
#define UPLOAD_PIN GPIO_NUM_23 //button that activates uploading when pressed
#define PWM_CHANNEL 1 //anything from 1-16
#define LED_FREQ 38000 //pwm modulation frequency, depends on receiver hardware
#define DUTY_CYCLE_RES 8 //keep at 8 bits, we don't need better resolution
#define MAX_WIFI_RETRIES 40 //number of times to try connecting to wifi before giving up
#define LED_ON_TIME 3000 //how long to turn on the IR LEDs in millis
#define INVALID_LOCATION -181.0 //invalid lat/lon
#define MAX_SENSORS 3 //maximum number of sensors we can read from at a time
#define RECORD_SIZE 2 //n bytes in a single record
#define HEADER_LEN 2

const char* ssid = "2firestar";
const char* password =  "sachin12";
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
  String mac_address_idx;
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

#define POST_LENGTH 250 //number of characters in a single json-formatted post request

char batch_post[2 + MAX_RECORDS*POST_LENGTH];

////////////////// begin the code    //////////////////////
void setup() {
  /*
   * Initialize the serial and configure the pinouts. 
   */
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIGGER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, LED_FREQ, DUTY_CYCLE_RES);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
}

void loop() {
  /*
   * Waits for one of the two buttons to be pressed. The first button makes it activate the LED,
   * and listen for ESP-Now packets. The other makes it upload its data to the server. 
   */
  bool trigger = false;
  bool upload = false;
  while(!trigger && !upload) { //stay in this loop until one of the buttons is pressed
    delay(10);
    trigger = digitalRead(TRIGGER_PIN);
    upload = digitalRead(UPLOAD_PIN);
  }
  if (trigger) {
    Serial.println("Trigger pressed!");
    setUpESPNow();
    //turn on the LED pin for a few seconds. The PWM frequency is configured in setup(). 
    //When data is received, it calls the callback function "OnDataRecv." 
    ledcWrite(PWM_CHANNEL, 128);
    delay(LED_ON_TIME);
    ledcWrite(PWM_CHANNEL, 0);
  } else if (upload) { //Connect to the wifi and upload your data
    Serial.println("Upload pressed!");
    //TODO: decompose this
    if (connect_to_server()) {
      for (int i=0; i<MAX_SENSORS;i++) {
        post_all_records(all_records[i]);
        int response = http.POST(batch_post);
        if (response != 200) {
          Serial.print("HTTP Post error "); Serial.println(response);
        } else {
          Serial.println("Successfully posted.");
        }
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
  Serial.print("Packet Recv from: "); Serial.print(macStr); 
  
  uint8_t index = 0;
  memcpy(&index, &data[0], sizeof(uint8_t)); // data index
  Serial.print(" with index "); Serial.println(index);
  
  int record_idx = find_record_by_mac(macStr + String(index));  
  //if we have this mac address but the number of records from it == MAX_RECORDS...
  if (record_idx == -1) { //if we don't have this mac address, it is the first packet from a new device
    Serial.println("Unpack sensor metadata");
    decodeMetaData(data, macStr);
  } else { //read the sensor's data
    Measurement m = all_records[record_idx].m;
    float *a = all_records[record_idx].sensor_data;
    int data_head = all_records[record_idx].n_records_recd;
    Serial.println("Process sensor data using " + m.type);
    for (int i=HEADER_LEN; i < data_len; i+=RECORD_SIZE) { //iterate 2 bytes at a time
      SplitShort s = {data[i], data[i+1]};
      a[data_head - HEADER_LEN + i/RECORD_SIZE] = short_to_float(s, m);
    }
    Serial.print("First reading sent: "); Serial.println(a[data_head]);  
    all_records[record_idx].n_records_recd += (data_len-HEADER_LEN)/RECORD_SIZE; //increment last position where receiving data    
  }
}

void decodeMetaData(const uint8_t *metadata, String mac_str) {
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
  uint8_t index = 0;
  uint8_t n_packets = 0;
  
  memcpy(&my_device_name[0], metadata, 32); // name
  memcpy(&my_lat, &metadata[32], sizeof(float)); // lat
  memcpy(&my_lon, &metadata[36], sizeof(float)); // lon
  memcpy(&my_type[0], &metadata[40], 32); // name
  memcpy(&my_unit[0], &metadata[72], 32); // name
  memcpy(&my_min_value, &metadata[104], sizeof(float)); // lat
  memcpy(&my_resolution, &metadata[108], sizeof(float)); // lon
  memcpy(&my_hardware[0], &metadata[112], 32); // hardware
  memcpy(&index, &metadata[144], sizeof(uint8_t)); // data index
  memcpy(&n_packets, &metadata[145], sizeof(uint8_t)); // n packets

  Serial.println("Device name: " + String(my_device_name));
  Serial.println("Location: " + String(my_lat) + ", " + String(my_lon));
  Serial.println("Measurement: " + String(my_type) + ", " + String(my_unit));
  Serial.println("Measurement Min, Res: " + String(my_min_value) + ", " + String(my_resolution));
  Serial.println("Next packets must be from: " + mac_str + String(index));
  
  //check if we've seen this sensor's data before. if it's new, use the sensor number to index.
  int record_idx = find_record_by_mac(mac_str + String(index));
  if (record_idx == -1) { 
    record_idx = n_sensors_received;
  }
  //Use all this information to create a Record, containing a zero-instantiated data array
  float data_array[MAX_RECORDS] = {0.0};
  all_records[record_idx] = {mac_str + String(index), my_device_name, my_lat, my_lon, 
    {my_min_value, my_resolution, my_type, my_unit, my_hardware}, data_array, 0};
  n_sensors_received += 1;
  Serial.println();
}

int find_record_by_mac(String mac_to_match) {
  /*
   * Given a mac address, return the index of that record
   */
  for (int i=0; i<MAX_SENSORS; i++) {
    if (all_records[i].mac_address_idx.equals(mac_to_match)) {
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

void post_all_records(Record r) {
  /*
   * Convert the array of input measurements into a json-style list. 
   * Saves it into the batch_post buffer.
   */
   
  //set the first char to open bracket
  memset(batch_post, '[', 1);
  //set the second char to terminator, so concatenation starts from here instead of adding on to the end of the previous
  memset(&batch_post[1], '\0', 1);
  String post = create_post_string(r.device_name, r.sensor_data[0], r.m, r.lat, r.lon) + ",";
  strncat(batch_post, post.c_str(), strlen(post.c_str())+1);//one extra for the null terminator. may have to memset this ourselves
  for (int i = 1; i < 2; i++) { //should be to r.n_records_recd
    String post = create_post_string(r.device_name, r.sensor_data[i], r.m, r.lat, r.lon);
    if (i < 2-1) {
      post = post + ",";
    }
    strncat(batch_post, post.c_str(), strlen(post.c_str()));
  }
  strcat(batch_post, "]");
  Serial.println("Built post string.");
  Serial.println(batch_post);
}

String create_post_string(String device_name, float value, Measurement m, float lat, float lon) {
  /*
   * Convert the input measurement to a json-formatted string. Pass in the device name, the measured
   * value, the measurement type, and the coordinates. 
   * 
   * For non-mandatory fields:
   *  -For lat/lon, if you pass in the INVALID_LOCATION value,  it won't include the coordinates
   *  -If the Measurement's hardware_name parameter is an empty string, it won't be included
   *  -Time is not included. 
   * 
   */
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m.type;
  json = json + "\",\"unit\":\"" + m.unit + "\",\"value\":\"" + String(value);

  if (lat != INVALID_LOCATION && lon != INVALID_LOCATION) {
    json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    
  } if (!m.hardware_name.equals("")) {
    json = json + "\",\"hardware\":\"" + m.hardware_name;
  }
  json = json + "\"}";
  return json;
}

//////////////////ESP Now and Wifi setup /////////////////////

void setUpESPNow() {
  /*
   * Prepares the device to receive data on ESP-Now. Disconnects from Wifi, 
   * configures the device as an Access Point, and prints out its mac address. 
   * 
   */
  WiFi.disconnect(); //TODO: make sure this works after connecting to wifi, may need to move after configAP
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv); //OnDataRecv will get called whenever we receive data. 
}


bool connect_to_server() {
  /*
   * Connect to the wifi and the server. Return true if successful. 
   * Gives up if it can't connect to the WiFi network after several tries. 
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
  http.addHeader("Content-Type", "application/json"); //all our POST requests will be in json format!
  Serial.println("Connected to server.");
  return true;
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
