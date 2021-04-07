/*
 * Install the following libraries: 
 * "Wifi" (builtin)
 * "HTTP Client" (builtin on some devices)
 * "Adafruit BusIO"
 * "Adafruit Unified Sensor"
 * "ESP8266 and ESP32 OLED driver for SSD1306 displays"
 * "Adafruit BME280"
 * 
 * Author: Arjun Tambe, Analytical Mechanical Associates
 * Source: https://github.com/oyetkin/droneiotclient
 * 
 * Credit for some portions of code: 
 *    ESP-Now Demo by Arvind Ravulavaru <https://github.com/arvindr21>
 */
#include <Wire.h>  
#include <WiFi.h>
#include "HTTPClient.h"
#include <Adafruit_BME280.h>
#include "SH1106Wire.h"
#include <time.h>
#include <esp_now.h>
#include "driver/adc.h"
#include "esp_wifi.h"

// User should change these settings as needed
#define TIME_TO_SLEEP  60 //time to sleep in seconds
const char* ssid = "2firestar"; //your wifi network name
const char* password =  "sachin12"; //your wifi network password
const char* server = "https://api.is-conic.com/api/v0p1/sensor/batch"; //the URL to post to your server
const String device_name = "arjun_station"; //name of your device
float lat = 32.636462; //the latitude of the location of the device
float lon = -117.095660; //the latitude of the location of the device

//more advanced user settings here
#define MAX_WIFI_RETRIES 30 //number of times to try connecting to wifi before giving up
#define DISPLAY_LEN 2000 //time to show OLED in millis
#define MODE_SELECT_LEN 4000 //time after a button press that the device returns to sleep
#define DEBOUNCE_TIME 400 //n millis to wait between 2 button presses

// PINOUTS
#define DEVICE_MODE_SELECT_PIN GPIO_NUM_27
#define TRIGGER_PIN GPIO_NUM_14 //remote trigger circuit read from this pin 
#define OLED_BUTTON GPIO_NUM_13 //trigger button read from this pin
#define ESP_NOW_CHANNEL 0
#define WAKE_PIN_BITMASK 0x008006000 // 2^OLED_BUTTON + 2^TRIGGER_PIN in hex

// OLED DiSPLAY AND GRAPH
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define POS_X_AXIS       5 //how far is the axis away from the margin?
#define POS_Y_AXIS       1
#define TITLE_X 10
#define TITLE_Y 6

//OTHER CONFIGURABLE
#define MAX_RECORDS 298 // total readings for EACH of the sensor data
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define RECORD_SIZE 2 //n bytes in a single record
#define HEADER_LEN 2
#define BME_ADDR 0x76 //default address of the BME sensor

///// this section of code should also be in the receiver!! ////////
struct measurement {
  float min_value; //min recordable by senseor
  float resolution; //sensor resolution
  String unit; //e.g. "Celsius"
  String type; //e.g. "temperature"
  float graph_lower; //lowest value to display on graph
  float graph_upper; //highest value for graph
  String hardware_name; //optionally, hardware used to collect the measurement
};
typedef struct measurement Measurement;
//16-bit value divided into 2 8-bit values
struct split_short {
  uint8_t high;
  uint8_t low;
};
typedef struct split_short SplitShort;

//make a "Measurement" for each of your sensors. Max is ~65K increments.
//the format is {min_value, increment, units, measurement name, graph_min, graph_max, hardware name}
Measurement pressure = {80000.0, 1.0, "Pa", "pressure", 90000, 110000, "BME280"};
Measurement temperature = {0.0, 0.01, "Celsius", "temperature", 15, 35, "BME280"};
Measurement humidity = {0, 0.01, "Relative %", "humidity", 30, 100, "BME280"};
Measurement measurements[3] = {temperature, humidity, pressure};
//edit these - change their names or add or remove arrays
RTC_DATA_ATTR uint8_t hum_data[MAX_RECORDS];
RTC_DATA_ATTR uint8_t temp_data[MAX_RECORDS];
RTC_DATA_ATTR uint8_t pres_data[MAX_RECORDS];
uint8_t* all_data[3] = {temp_data, hum_data, pres_data}; //match the order of measurements variable above!
uint8_t metadata[ESP_NOW_MAX_DATA_LEN];

////////////////////
const char* ntpServer = "pool.ntp.org";

HTTPClient http;
SH1106Wire display(0x3c, SDA, SCL); //7 bit address only
Adafruit_BME280 bme;

//state tracking
RTC_DATA_ATTR bool device_mode_wifi = 0; //1 for wifi, 0 for trigger
RTC_DATA_ATTR int n_cycles_recorded = 0;

//for ESP Now - MAC address, channel, encrypt - set MAC address to device or to broadcast address
//esp_now_peer_info_t slave = {{0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF}, ESP_NOW_CHANNEL, 0};
esp_now_peer_info_t slave = {{0x9C, 0x9C, 0x1F, 0xC9, 0x50, 0x61}, ESP_NOW_CHANNEL, 0};

void setup() {
  /*
   * This is the main function controlling what the device does. It opens the serial, checks
   * the cause of the wakeup, and decides to do based on the cause:
   *  -If it was the mode select pin, allow the user to change the mode
   *  -Otherwise, what we do depends on the mode we're in.
   *    -In Wifi mode:
   *      -if we woke from the trigger button, read the sensors, post to the server, and show the graph
   *      -if we woke from the timer, read and post but don't show the graph
   *      -if we're waking up the first time, display that on the OLED
   *    -In Trigger mode:
   *      -if we woke from the trigger button, immediately send the data using ESP-Now, and show the graph
   *      -if we woke from the timer, just read the sensors
   *      -if we're waking up for the first time, display that on the OLED.
   */
  Serial.begin(115200);
  delay(1000);

  //get and report wakeup cause, and print the mode the device is in.
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  print_wakeup_reason(wakeup_reason);
  Serial.println("Mode: " + mode_to_str(device_mode_wifi));
  int pin_triggered = get_wakeup_pin_number();
  
  if (pin_triggered == DEVICE_MODE_SELECT_PIN) { //The Mode Select button.
    mode_select(&display);
  } else if (device_mode_wifi) {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { //Either the IR Trigger,  or the Trigger Button
      read_sensors();
      post_sensors();
      display_graph(&display);
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) { //The timer
      read_sensors();
      post_sensors();
    } else {                                      // Device booted up for the first time
      show_wakeup(&display, device_mode_wifi);
      read_sensors();
      post_sensors();
    }
  } else {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { //Either the IR Trigger,  or the Trigger Button
      ESPNowToMac();
      display_graph(&display);
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) { //The timer
      read_sensors();
    } else {                                      // Device booted up for the first time
      show_wakeup(&display, device_mode_wifi);
      read_sensors();
    } 
  }
  go_to_sleep(device_mode_wifi);
}

///////////////////////////Top level methods///////////////////////

void mode_select(SH1106Wire* d) {
  /*
   * Call when the device is woken by the mode select pin. Activates the OLED
   * and allows the user to select the mode. Exits after a few seconds idle.
   */
  Serial.println("Press to select mode, currently " + mode_to_str(device_mode_wifi));
  pinMode(DEVICE_MODE_SELECT_PIN, INPUT);
  //turn on the display and show the current mode
  d->init();
  d->flipScreenVertically();
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0, 12, "Mode: " + mode_to_str(device_mode_wifi));
  d->display();
  unsigned long start_time = millis();
  unsigned long debounce = millis();
  while (millis() - start_time <= MODE_SELECT_LEN) {
    //if button is pressed, change mode and restart timer
    if (digitalRead(DEVICE_MODE_SELECT_PIN) && millis() - debounce >= DEBOUNCE_TIME) {
      device_mode_wifi = 1 - device_mode_wifi;
      Serial.println("Button pressed. Mode is: " + mode_to_str(device_mode_wifi));
      d->clear();
      d->drawString(0, 12, "Mode: " + mode_to_str(device_mode_wifi));
      d->display();
      start_time = millis();
      debounce = start_time;
    }
  }
  //turn off the OLED display
  d->displayOff();
}

void read_sensors() {
  /*
   * Read the sensor values and print to the serial. 
   */
  if (!bme.begin(BME_ADDR)) {
    Serial.println("Could not find a valid BME sensor!");
    go_to_sleep(1);
  }
  Serial.println("Read sensors");
  delay(2000);
  float curr_hum = bme.readHumidity();
  float curr_temp = bme.readTemperature();
  float curr_pres = bme.readPressure();
  if (isnan(curr_hum) || isnan(curr_temp) || isnan(curr_pres)) {
    Serial.println("Failed to read!");
    go_to_sleep(1);
  } 
  //Save all the measurements. The function handles the conversion between floats and bytes for us. 
  save_float_measurement(curr_temp, temperature, temp_data, MAX_RECORDS);
  save_float_measurement(curr_hum, humidity, hum_data, MAX_RECORDS);
  save_float_measurement(curr_pres, pressure, pres_data, MAX_RECORDS);

  //Print readings to the serial
  Serial.println("Humidity (RH%): " + String(curr_hum));
  Serial.println("Temperature (C): " + String(curr_temp));
  Serial.println("Pressure (Pa): " + String(curr_pres));

  n_cycles_recorded ++;
}

void post_sensors() {
  /*
   * Call this when the sensor is booted by time. Activates a wifi connection, 
   * loads the sensors, gets their readings, and posts the data. 
   */
  //Connect to wifi and get the time
  connect_to_server();
  configTime(0, 0, ntpServer);

  //Now get the last measurements from the data buffers
  float temp = last_float_from_data(temp_data, temperature);
  float hum = last_float_from_data(hum_data, humidity);
  float pres = last_float_from_data(pres_data, pressure);
  
  String posts[3] = {create_post_string(temp, temperature, true, true, true), 
                      create_post_string(hum, humidity, true, true, true),
                      create_post_string(pres, pressure, true, true, true)};
  Serial.println(multi_post_string(posts, 3));
  int response = http.POST(multi_post_string(posts, 3));
  if (response != 200) {
    Serial.println("HTTP Post error");
  }
  http.end();
}

void ESPNowBroadcast() {
  /*
   * Send ESP Now data as a broadcast. Doesn't check for pairing success. 
   * Set the channel to 0. The receiver channel does not have to be the same. 
   */
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_add_peer(&slave);
  sendDataMulti(temp_data, temperature, 1);
  sendDataMulti(hum_data, humidity, 2);
  sendDataMulti(pres_data, pressure, 3);
}

void ESPNowToMac() {
  /*
   * Send ESP Now data to a specific, hard-coded MAC address. Does not
   * scan WiFi networks, but does verify that a listening device with
   * the mac address exists. 
   * Pairing failure is not currently handled. 
   */
  WiFi.mode(WIFI_STA);
  InitESPNow();
  if (slave.channel == ESP_NOW_CHANNEL) { //check for correct channel; add peer
    Serial.print("Slave Status: ");
    esp_err_t addStatus = esp_now_add_peer(&slave);
    if (debug_ESP_error(addStatus)) { //this also prints the status
      Serial.println("Slave is paired");

      sendDataMulti(temp_data, temperature, 1);
      sendDataMulti(hum_data, humidity, 2);
      sendDataMulti(pres_data, pressure, 3);
    } else {
      Serial.println("Slave pair failed!");
    }
  } else {
    Serial.println("Non matching slave channel!");
  }
}

void go_to_sleep(bool wifi_mode) {
  /*
   * Send the device to sleep. Wifi_mode doesn't do anything currently, but you can 
   * use it to edit how the device will wake up. 
   */
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  adc_power_off(); //turn off ADC
  WiFi.mode(WIFI_MODE_NULL); //turn off WiFi
  esp_wifi_stop();
  //Before we sleep, make sure to enable the device to wake up next time!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //timer wakeup
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH); //pin wakeup
  esp_deep_sleep_start();
}

////////////////////////// DATA METHODS /////////////////////////////

void save_float_measurement(float v, Measurement m, uint8_t* a, int a_len) {
  /*
   * Given a float measurement, save it in the data buffer. 
   * Converts types and saves as two bytes
   */
  SplitShort v_short = float_to_short(v, m);
  push_back_short(a, v_short, a_len);
}

void push_back_short(uint8_t* a, SplitShort v, int a_len) {
  /*
   * Uses the specified array as a circular buffer. Pushes all its entries
   * back by 2 positions, then stores the SplitShort (which is just 2
   * uint8's) in the first 2 spots. HIGH bits go in position 0; LOW go in
   * position 1. 
   */
   //a is an address. int* p = (int *)a assigns it to a pointer. p[1] is the next address
   memcpy(&a[2], a, sizeof(a[0])*(a_len-2));
   a[0] = v.high;
   a[1] = v.low;
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

SplitShort float_to_short(float v, Measurement m) {
  /*
   * Given a measurement type and a value, convert the float we get
   * from the Arduino interface to the short used for ESP Now
   */
   uint16_t x = (v - m.min_value)/m.resolution;
   uint8_t xlow = x & 0xFF;
   uint8_t xhigh = (x >> 8);
   return {xhigh, xlow};
}

float last_float_from_data(uint8_t* a, measurement m) {
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

/////////////////////////// ESP METHODS /////////////////////////////////////

void sendDataMulti(uint8_t* a, Measurement m, uint8_t index) {
  /*
   * Send the array a on ESP Now in multiple packets - because the max
   * packet size is only 250 bytes. 
   * Note that the math only works because a is a byte array!
   */
  int total_sent = 0; //bytes of how much data you've sent so far
  makeMetaData(m, index, MAX_RECORDS*RECORD_SIZE);
  Serial.println("Sending metadata");
  esp_now_send(slave.peer_addr, metadata, ESP_NOW_MAX_DATA_LEN);
  Serial.println("Enter while loop");
  while (total_sent < MAX_RECORDS*RECORD_SIZE) {
    int remaining = MAX_RECORDS*RECORD_SIZE - total_sent;
    int data_len = min(ESP_NOW_MAX_DATA_LEN - HEADER_LEN, remaining); // this is in bytes excluding header
    Serial.println("Sending " + String(data_len) + " bytes of " + String(MAX_RECORDS*RECORD_SIZE));

    //create the actual packet to send
    uint8_t to_send[ESP_NOW_MAX_DATA_LEN] = {0};
    memset(&to_send[0], index, sizeof(uint8_t)); // index number of the data
    memcpy(&to_send[HEADER_LEN], &a[total_sent], data_len);

    SplitShort s = {to_send[2], to_send[3]};
    SplitShort s2 = {to_send[4], to_send[5]};
    Serial.print("Sending "); Serial.println(short_to_float(s, m));
    Serial.print("Sending "); Serial.println(short_to_float(s2, m));
    
    esp_now_send(slave.peer_addr, &to_send[0], data_len + HEADER_LEN);
    total_sent += data_len;
   }
}

uint8_t* makeMetaData(Measurement m, uint8_t index, int data_len) {
  /*
   * Send a metadata packet that includes the following info in the following order:
   *    Sensor name (32 chars)
   *    Lat and lon (two 4-byte floats)
   *    Measurement name (32 chars)
   *    Min measurement value (4-byte float)
   *    Resolution (4-byte float)
   *    Hardware name (32 chars)
   *    N packets of data (2-byte int)
   *  It accomplishes this by memcpy'ing each piece of information in the respective
   *  positions. The receiver must contain code to read this metadata to function properly.
   *  
   *  data_len is the total number of bytes we will be sending after the metadata. 
   */
  const char* device_name_c = device_name.c_str();
  const char* measurement_type_c = m.type.c_str();
  const char* unit_c = m.unit.c_str();
  const char* hardware_name_c = m.hardware_name.c_str();
  uint8_t n_packets = data_len/ESP_NOW_MAX_DATA_LEN + (data_len%ESP_NOW_MAX_DATA_LEN != 0); 

  memcpy(&metadata[0], device_name_c, min((size_t) 32, strlen(device_name_c)+1)); // name
  memcpy(&metadata[32], &lat, sizeof(float)); // lat
  memcpy(&metadata[36], &lon, sizeof(float)); // lon
  memcpy(&metadata[40], measurement_type_c, min((size_t) 32, strlen(measurement_type_c)+1)); // measurement type
  memcpy(&metadata[72], unit_c, min((size_t) 32, strlen(unit_c)+1)); // measurement type
  memcpy(&metadata[104], &m.min_value, sizeof(float)); // min value
  memcpy(&metadata[108], &m.resolution, sizeof(float)); // resolution
  memcpy(&metadata[112], hardware_name_c, min((size_t) 32, strlen(hardware_name_c)+1)); // hardware name
  memset(&metadata[144], index, sizeof(uint8_t)); // index number of the data
  memcpy(&metadata[145], &n_packets, sizeof(uint8_t)); // n packets
  
  //round up by adding 1 if the division is not even
  Serial.println("Successfully created metadata");
  //return &metadata[0];
}

void update_mac(uint8_t n) {
  /*
   * When sending multiple data sets, edit the last byte of the mac address so that each data set is sent
   * with a different mac address. n cannot be greater than 256 (1 byte)
   */
  uint8_t mac[6];
  WiFi.macAddress(mac);
  mac[5] = n;
  esp_wifi_set_mac(WIFI_IF_STA, &mac[0]);
}

void defaultESPSend() {
  /*
   * Send on ESP Now using the logic from the standard sketch:
   * Scan for networks; find an SSID that contains "Slave:", 
   * get that network's MAC address, set that as the slave, 
   * add it as a peer, and then send data. 
   * 
   * Set the channel to the receiver's channel number in the script
   * 
   * Note that the callback DOES interrupt sleep!
   */
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
  ScanForSlave();
  if (slave.channel == ESP_NOW_CHANNEL) { //check for correct channel; add peer
    Serial.print("Slave Status: ");
    esp_err_t addStatus = esp_now_add_peer(&slave);
    if (debug_ESP_error(addStatus)) { //this also prints the status
      uint16_t send_this = 12345;
      sendData(send_this);
    } else {
      Serial.println("Slave pair failed!");
    }
  } else {
    Serial.println("Non matching slave channel!");
  }
}

void InitESPNow() {
  /*
   * Just initialize the ESP Now
   */
  //WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
  }
}

void ScanForSlave() {
  /*
   * Find all wifi networks and check if any of their SSIDs start with 
   * "Slave". Use it to update the global slave object - peer_addr, channel
   */
  int8_t scanResults = WiFi.scanNetworks();
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      //delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = ESP_NOW_CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        Serial.println("Slave found!");
        WiFi.scanDelete();
        return; // only one listener, so break after finding the first one
      }
    }
  }
  Serial.println("Slave Not Found, trying again.");
  WiFi.scanDelete();
}

void sendData(uint8_t data_to_send) {
  /*
   * Send data to the receiver/slave address
   */
  Serial.print("Sending: "); Serial.println(data_to_send);
  esp_err_t result = esp_now_send(slave.peer_addr, &data_to_send, sizeof(data_to_send));
  Serial.print("Send Status: ");
  debug_ESP_error(result);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool debug_ESP_error(esp_err_t err) {
  /*
   * Just look at the error value and print the message corresponding to it
   */
  if (err == ESP_OK) {
    Serial.println("Success");
    return true;
  } else if (err == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNOW not Init.");
    return false;
  } else if (err == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
    return false;
  } else if (err == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
    return false;
  } else if (err == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
    return false;
  } else if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
    return false;
  } else if (err == ESP_ERR_ESPNOW_FULL) {
    Serial.println("Peer list full");
    return false;
  } else if (err == ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Peer Exists");
    return true;
  } else {
    Serial.println("Not sure what happened");
    return false;
  }
}



/////////////////////////// Regular wifi methods /////////////////////////////

void connect_to_server() {
  /*
   * Connect to the wifi and Otto's server
   */
  WiFi.begin(ssid, password);
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

String create_post_string(float value, Measurement m, bool incl_time, bool incl_location, bool incl_hardware) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m.type;
  json = json + "\",\"unit\":\"" + m.unit + "\",\"value\":\"" + String(value);
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
  if (incl_hardware) {
    json = json + "\",\"hardware\":\"" + m.hardware_name;
  }
  json = json + "\"}";
  return json;
}

////////////////////// OLED control methods ////////////////////////////

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

void display_graph(SH1106Wire* d) {
  /*
   * Show a graph of all the senseor readings 
   */
  Serial.println("Displaying graph");
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);

  for (int i = 0; i<(sizeof(measurements)/sizeof(measurements[0])); i++) {
    d->clear();
    add_graph(d, all_data[i], measurements[i]);
    d->display();
    delay(DISPLAY_LEN);
  }
  
  d->displayOff();
}

void add_graph(SH1106Wire* d, uint8_t* a, Measurement m) {
  /*
   * Plot the given array onto the graph
   * Does not actually display anything yet - just adds the element to the OLED screen
   */

  d->drawString(TITLE_X, TITLE_Y, m.type);
  d->drawLine(0, SCREEN_HEIGHT-POS_X_AXIS, SCREEN_WIDTH, SCREEN_HEIGHT-POS_X_AXIS); //x 
  d->drawLine(POS_Y_AXIS, 0, POS_Y_AXIS, SCREEN_HEIGHT); //y
  
  float last_y = float_to_screen_y(short_to_float({a[0], a[1]}, m), m.graph_lower, m.graph_upper);;
  float last_x = POS_Y_AXIS;
  for (int i=1; i < min(MAX_RECORDS, n_cycles_recorded); i++) {
    float x_pos = float_to_screen_x(i);
    SplitShort s = {a[i*2], a[i*2 + 1]};
    float y_pos = float_to_screen_y(short_to_float(s, m), m.graph_lower, m.graph_upper);
    d->drawLine(last_x, last_y, x_pos, y_pos);
    last_y = y_pos;
    last_x = x_pos;
  }
}

float float_to_screen_x(int i) {
  /*
   * Given the iteration of the data we are currently on, return the position on the screen
   * that data point should be plotted
   */
  return map(i, 0, min(MAX_RECORDS, n_cycles_recorded)-1, POS_Y_AXIS, SCREEN_WIDTH);
}

float float_to_screen_y(float v, float min_value, float max_value) {
  /*
   * Given a float value and min/max values of a measurement, return
   * the position on the y axis that the measurement would be plotted
   * 
   * currently does not handle negative values
   */
   float mapped = map(v, 0, max_value, POS_X_AXIS, SCREEN_HEIGHT);
   return SCREEN_HEIGHT - mapped;
}

void display_last_reading_and_sleep(SH1106Wire* d) {
  /*
   * Call this when the sensor is booted by external trigger. 
   * Activate the OLED display, show readings, and then turn off the display.
   */
  d->init(); //OLED
  d->flipScreenVertically();
  d->setFont(ArialMT_Plain_10);
  display_readings(d, 
    last_float_from_data(hum_data, humidity), 
    last_float_from_data(temp_data, temperature), 
    last_float_from_data(pres_data, pressure));
  delay(DISPLAY_LEN);
  d->displayOff();
}

void display_readings(SH1106Wire* d, float h, float t, float p) {
    /*
     * Display the current humidity and temperature on the OLED. 
     */
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0, 12, "Humidity: " + String(h) + "%");
  d->drawString(0, 28, "Temp: " + String(t) + "C");
  d->drawString(0, 44, "Pressure: " + String(p) + "Pa");
  d->display();
}


///////////////////// MISCELLANEOUS METHODS ////////////////////

int get_wakeup_pin_number() {
  /*
   * Get the number of the pin that woke us up. 
   * Log base 2 of wakeup_status equals the pin number so we use change of base.
   */
  return log(esp_sleep_get_ext1_wakeup_status())/log(2);
}


void print_wakeup_reason(esp_sleep_wakeup_cause_t wakeup_reason) {
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.print("Wakeup cause: interrupt. ");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.print("Wakeup cause: timer. ");
  }
}

String mode_to_str(bool device_mode) {
  /*
   * Convert device mode to string
   */
  if (device_mode) {
    return "WiFi";
  } else {
    return "Trigger";
  }
}

void loop() {}
