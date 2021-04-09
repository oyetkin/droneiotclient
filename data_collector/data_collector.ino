/*
 * This code is for the Data Collector, a device that wakes up other 
 * Data Loggers using an IR signal, and reads data from the devices 
 * using ESP-Now. It also posts to the server.  
 * 
 * Author: Arjun Tambe, Analytical Mechanical Associates
 * Source: https://github.com/oyetkin/droneiotclient
 * 
*/

#include <esp_now.h>
#include "WiFi.h"
#include "HTTPClient.h"
#include <time.h>

//Edit these!
const char* ssid = "<NETWORK>"; //Name of your wifi network
const char* password =  "<PWD>"; //Password for your wifi network
const char* server = "<SERVER>"; //URL for your server

//Pinouts and configurable
#define TRIGGER_PIN GPIO_NUM_16 //for the trigger button
#define LED_PIN GPIO_NUM_17 //output gpio for the led
#define UPLOAD_PIN GPIO_NUM_23 //button that activates uploading when pressed
#define MAX_WIFI_RETRIES 40 //number of times to try connecting to wifi before giving up
#define LED_ON_TIME 3000 //how long to turn on the IR LEDs in millis
#define MAX_SERIES 3 //maximum number of different data series we can read from (see below)

//Other stuff
#define CHANNEL 1 //esp now transmission channel, anything from 1-16
#define PWM_CHANNEL 1 //anything from 1-16
#define LED_FREQ 38000 //pwm modulation frequency, depends on data logger IR hardware
#define DUTY_CYCLE_RES 8 //keep at 8 bits, we don't need better resolution
#define INVALID_LOCATION -181.0 //an invalid lat/lon to use as a placeholder

//This MUST match the code in the data logger to accurately send metadata
#define RECORD_SIZE 2 //n bytes in a single record
#define PREAMBLE_LEN 2
#define MAX_RECORDS 298 // Max number of readings that each sensor could contain

//This struct defines a category of measurement, like "temperature" or "humidity."
struct measurement {
  float min_value; //min recordable by senseor
  float resolution; //sensor resolution
  String unit; //e.g. "Celsius"
  String type; //e.g. "temperature"
  String hardware_name; //optionally, hardware used to collect the measurement
  //The data logger version has graph lower and upper bounds, which we don't need here.
};
typedef struct measurement Measurement;

//One data_series contains metadata and data for a single measurement type. 
//For example, temperature data from one device and humidity data from the same device
//are considered two separate data_series. 
//We don't have to instantiate these ourselves - we'll use the metadata contained in packets to do that!
struct data_series {
  String series_id; //A unique ID (MAC address + a 2-byte preamble) used to identify packets
  String device_name; //Name of the data logger
  float lat; //coordinates of the logger
  float lon;
  int time_interval; //time in between each measurement
  unsigned long last_measurement_time; //unix time for the last measurement
  Measurement m; //the type of measurement this data series tracks
  float sensor_data[MAX_RECORDS]; //contains the actual data for this data series
  int n_records_recd = 0; //how many values the sensor_data array actually contains right now
};
typedef struct data_series dataSeries;

//Create an array containing ALL the data series
dataSeries all_data_series[MAX_SERIES];

//This is just a way to store 2 bytes together. There are methods to go between the 2-byte datatype to a float. 
struct twobyte {
  uint8_t high;
  uint8_t low;
};
typedef struct twobyte TwoByte;

//Number of different data series we've received so far
int n_series_received = 0;

//Fix this later...
#define POST_LENGTH 250 //number of characters in a single json-formatted post request
char batch_post[2 + MAX_RECORDS*POST_LENGTH];
HTTPClient http;
const char* ntpServer = "pool.ntp.org";
unsigned long last_time_stamp;


////////////////// begin the code    //////////////////////
void setup() {
  /*
   * Initialize the serial and configure the pinouts. 
   */
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIGGER_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  //Used for PWM; set LED_FREQ to the desired freuqency
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
  while(!trigger && !upload) {              //stay in this loop until one of the buttons is pressed
    delay(10);
    trigger = digitalRead(TRIGGER_PIN);
    upload = digitalRead(UPLOAD_PIN);
  }
  if (trigger) {       // if it was the trigger button, then prepare for ESP Now and fire the IR LED
    Serial.println("Trigger pressed!");
    //update the current time, which is used to figure out the times of data received
    updateRealTime();
    Serial.println(last_time_stamp);
    //When data is received, it calls the callback function "OnDataRecv." 
    setUpESPNow();
    sendIRTrigger();
  } else if (upload) {       //If it was the upload button, connect to the wifi and upload your data
    Serial.println("Upload pressed!");
    uploadData();
  }
}

//////////////////// High level functions ///////////////////////

void sendIRTrigger() {
  /*
   * Simply turn the LED on, and turn it back off after a few seconds
   * The PWM frequency of the LED is configured in setup(), and must match
   * the hardware of the data logger's IR receiver.
   */
  //Write a 50% duty cycle (a square wave that's HIGH half the time) by dividing the 
  //maximum resolution value (2^DUTY_CYCLE_RES) by 2. 
  ledcWrite(PWM_CHANNEL, pow(2, DUTY_CYCLE_RES-1));
  delay(LED_ON_TIME);
  ledcWrite(PWM_CHANNEL, 0); //write 0% duty cycle, aka off
}


void uploadData() {
  /*
   * Upload all data to the server. Iterates through each of the series we have, 
   * and posts each of the records for that series. 
   */
  if (connect_to_server()) {
    for (int i=0; i<n_series_received; i++) {
      Serial.println("Posting records for " + String(all_data_series[i].series_id));
      post_all_records(&(all_data_series[i]));
    }
    http.end();
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  /*
   * Called as an interrupt whenever data is received on ESP Now. 
   */
  ledcWrite(PWM_CHANNEL, 0); //stop firing the IR LED once the other ESP sends data
  String series_id = get_series_id(mac_addr, data); //convert mac address and the top of the data into the Series ID
  Serial.print("Packet Recv from: "); Serial.println(series_id); 

  //Get the index in the all_series array of this series id. If it's not in the array, the index will be -1
  int idx = find_series_by_id(series_id);
  if (idx == -1) {                                            //if not found, it is the first packet for a new series, so it's metadata
    Serial.println("Unpack sensor metadata");
    decodeMetaData(data, series_id);
    n_series_received += 1; //increment counter of series received only if it's a new series; and must be after decode metadata
  } else if (all_data_series[idx].n_records_recd >= MAX_RECORDS) { //if we've seen this series id, but the number of records is full,
    Serial.println("Buffer full from this ID; overwriting");        // then treat it as a new metdata and overwrite the old data.
    decodeMetaData(data, series_id);
    all_data_series[idx].n_records_recd = 0;                        //reset the number of records to 0
  } else {                                                          //If it's found, we now have regular data, so save it in the array
    save_series_in_series_array(idx, data, data_len);
  }
}

/////////////////////// Data receiving methods ///////////////////////////


String get_series_id(const uint8_t *mac_addr, const uint8_t *data) {
  /*
   * Create a Series ID string, which is just the mac address plus first 2 bytes (preamble) of the data 
   */
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  uint8_t index = 0;
  memcpy(&index, &data[0], sizeof(uint8_t)); // first two bytes of the data are the "index"

  return macStr + String(index);
}


void decodeMetaData(const uint8_t *metadata, String series_id) {
  /*
   * Decode the metadata packet. Memcopies the metadata piece by piece
   * into a set of variables. 
   */
  char device_name[32] = {};
  float lat = INVALID_LOCATION;
  float lon = INVALID_LOCATION;
  float min_value = 0.0;
  float resolution = 0.0;
  char type[32] = {};
  char unit[32] = {};
  char hardware[32] = {};
  uint16_t index = 0;
  uint8_t n_packets = 0;
  uint16_t time_interval = 0;
  uint16_t time_offset = 0;

  //Go through each field, one by one, and save all the values
  memcpy(&index, &metadata[0], sizeof(uint8_t)); // data index
  memcpy(&device_name, &metadata[2], 32); // name
  memcpy(&lat, &metadata[34], sizeof(float)); // lat
  memcpy(&lon, &metadata[38], sizeof(float)); // lon
  memcpy(&type[0], &metadata[42], 32); // name
  memcpy(&unit[0], &metadata[74], 32); // name
  memcpy(&min_value, &metadata[106], sizeof(float)); // lat
  memcpy(&resolution, &metadata[110], sizeof(float)); // lon
  memcpy(&hardware[0], &metadata[114], 32); // hardware
  memcpy(&n_packets, &metadata[146], sizeof(uint8_t)); // n packets
  memcpy(&time_interval, &metadata[147], sizeof(uint16_t)); // time interval
  memcpy(&time_offset, &metadata[149], sizeof(uint16_t)); // time offset

  //Print some of the key values
  Serial.println("Device name: " + String(device_name));
  Serial.println("Location: " + String(lat) + ", " + String(lon));
  Serial.println("Measurement: " + String(type) + ", " + String(unit));
  Serial.println("Measurement Min, Res: " + String(min_value) + ", " + String(resolution));
  Serial.println("Time interval, offset: " + String(time_interval) + ", " + String(time_offset));

  //check if we've seen this sensor's data before, and get its index in the series array.
  //If we've never seen it before, we'll use the next available index - 
  // which is equal to the number of series we've received so far
  int series_index = find_series_by_id(series_id);
  if (series_index == -1) { 
    series_index = n_series_received;
  }
  //Use all the variables above to create a dataSeries struct
  all_data_series[series_index].series_id = series_id;
  all_data_series[series_index].device_name = device_name;
  all_data_series[series_index].lat = lat;
  all_data_series[series_index].lon = lon;
  all_data_series[series_index].time_interval = time_interval;
  //last measurement time is the receipt time (last_time_stamp) minus the time offset.
  all_data_series[series_index].last_measurement_time = last_time_stamp - time_offset;
  all_data_series[series_index].m = {min_value, resolution, type, unit, hardware};
  Serial.println();

}

void save_series_in_series_array(int series_index, const uint8_t *data, int data_len) {
  /*
   * Save a data packet received via ESP-Now to the data field of one of the dataSeries.
   * 
   * series_index: index of the series, in the array of data series, to save to
   * data: Full data from ESP Now, INCLUDING the metadata
   */
  Measurement m = all_data_series[series_index].m; //Get the type of measurement this series records, which we got from metadata
  float *a = all_data_series[series_index].sensor_data; //Get a pointer to the array where data will be saved
  Serial.println("Process sensor data using " + m.type);

  int data_head = all_data_series[series_index].n_records_recd; //Start saving data where the last packet left off
  for (int i=PREAMBLE_LEN; i < data_len; i+=RECORD_SIZE) { //iterate 2 bytes at a time, since every 2 bytes in data is 1 record
    TwoByte s = {data[i], data[i+1]};
    a[data_head - PREAMBLE_LEN + i/RECORD_SIZE] = twobyte_to_float(s, m);
  }
  Serial.print("First reading sent: "); Serial.println(a[data_head]);  
  all_data_series[series_index].n_records_recd += (data_len-PREAMBLE_LEN)/RECORD_SIZE; //increment last position to receive data next time
}

/////////////////////////// Data manipulation methods ///////////////////////

int find_series_by_id(String search_id) {
  /*
   * Given a Series ID (Mac + preamble), iterate through the array of all the data series, and return
   * the dataSeries that contains a matching Series ID. 
   */
  for (int i=0; i<MAX_SERIES; i++) {
    if (all_data_series[i].series_id.equals(search_id)) {
      return i;
    }
  }
  return -1;
}

float last_float_from_data(uint8_t* a, measurement m) {
  /*
   * Go into the specified array which is made up of TwoBytes, 
   * and return the first element in float format.
   * 
   * example: last_float_from_data(hum_data, humidity)
   */
   TwoByte s = {a[0], a[1]};
   return twobyte_to_float(s, m);
}

float twobyte_to_float(TwoByte s, measurement m) {
  /*
   * Given a measurement type and a twoByte, convert the twoByte we get
   * from the internal representation of data, to a float. The conversion 
   * depends on the measurement type. 
   */
   uint16_t v = (s.high << 8) + s.low;
   float x = v*m.resolution + m.min_value;
   return x;
}

/////////////////////////////////// Lower level wifi post methods //////////////////////////////

void post_all_records(dataSeries* r) {
  /*
   * Iterates through all records in the series, and posts each one, one by one. 
   */
  Serial.println("Posting records...");
  //the most recent measurement was made at last_measurement time; we already calculated its value when receiving
  unsigned long post_time = r->last_measurement_time;
  //iterate through each record and post it individually
  for (int i = 0; i < r.n_records_recd; i++) { 
    String post = create_post_string(r->device_name, r->sensor_data[i], &(r->m), r->lat, r->lon, post_time);
    int response = http.POST(post);
    if (response != 200) {
      Serial.print("HTTP Post error "); Serial.println(response);
    }
    //the next measurement was taken BEFORE this one, so decrease the time by the time interval
    post_time = post_time - r->time_interval;
  }
}

String create_post_string(String device_name, float value, Measurement* m, float lat, float lon, unsigned long timestamp) {
  /*
   * Convert the input measurement to a json-formatted string. Pass in the device name, the measured
   * value, the measurement type, and the coordinates. 
   * 
   * For non-mandatory fields:
   *  For lat/lon, if you pass in the INVALID_LOCATION value,  it won't include the coordinates
   *  If the Measurement's hardware_name parameter is an empty string, it won't be included
   * 
   */
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m->type;
  json = json + "\",\"unit\":\"" + m->unit + "\",\"value\":" + String(value);

  json = json + ",\"timestamp\":\"" + String(timestamp) + "\"";
  if (lat != INVALID_LOCATION && lon != INVALID_LOCATION) {
    json = json + ",\"lat\":" + String(lat) + ",\"lon\":" + String(lon);    
  }
  if (!m->hardware_name.equals("")) {
    json = json + ",\"hardware\":\"" + m->hardware_name + "\"";
  }
  json = json + "}";
  return json;
}

//////////////////ESP Now and Wifi setup /////////////////////

void setUpESPNow() {
  /*
   * Prepares the device to receive data on ESP-Now. Disconnects from Wifi, 
   * configures the device as an Access Point, and prints out its mac address. 
   * 
   */
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
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
    delay(1000);
    WiFi.begin(ssid, password);
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

void updateRealTime() {
  /*
   * Connects to the Wifi network to fetch the current time, in epoch time (seconds since 1970). 
   * Saves in the global variable last_time_stamp.
   */
  WiFi.mode(WIFI_STA);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    counter ++;
    delay(1000);
  }
  Serial.println("Connected.");
  http.begin(ntpServer);
  configTime(0, 0, ntpServer);
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  } else {
    time(&now);
    last_time_stamp = now;
  }
  http.end();
}
