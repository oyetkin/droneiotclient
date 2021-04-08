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
 *    Getting the time: https://randomnerdtutorials.com/epoch-unix-time-esp32-arduino/
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
#define TIME_TO_SLEEP  20 //How long the device sleeps in between measurements
const char* ssid = "2firestar"; //your wifi network name
const char* password =  "sachin12"; //your wifi network password
const char* server = "https://api.is-conic.com/api/v0p1/sensor/batch"; //the URL to post to your server
const String device_name = "arjun_station"; //name of your device
float lat = 32.636462; //the latitude of the location of the device
float lon = -117.095660; //the latitude of the location of the device

//more advanced user settings
#define MAX_WIFI_RETRIES 30 //number of times to try connecting to wifi before giving up
#define DISPLAY_LEN 2000 //time to show OLED in millis
#define MODE_SELECT_LEN 4000 //time after a button press that the device returns to sleep
#define DEBOUNCE_TIME 400 //n millis to wait between 2 button presses

// PINOUTS
#define DEVICE_MODE_SELECT_PIN GPIO_NUM_27 //pin connected to mode select button
#define TRIGGER_PIN GPIO_NUM_14 //pin connected to remote trigger circuit
#define OLED_BUTTON GPIO_NUM_13 //pin connected to trigger button
#define WAKE_PIN_BITMASK 0x008006000 // must be equal to 2^MODE_SELECT + 2^OLED_BUTTON + 2^TRIGGER_PIN
#define ESP_NOW_CHANNEL 0 // can select any from 0-15

// OLED display settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define POS_X_AXIS 5 //how far to draw x axis from the bottom margin, in pixels
#define POS_Y_AXIS 1 //how far to draw y axis from left margin
#define TITLE_X 10 // x coordinate where to draw graph title
#define TITLE_Y 6 // y coordinate where to draw graph title

//Addresses of the hardware devices. You shouldn't need to change these unless you get errors. 
#define OLED_ADDR 0x3c //use the 7-bit (not 8-bit) address for the OLED screen!
#define BME_ADDR 0x76 //default address of the BME sensor

//OTHER CONFIGURABLE
#define MAX_RECORDS 298 // Total N readings for each type of sensor data
#define RECORD_SIZE 2 //n bytes in a single record
#define PREAMBLE_LEN 2 //n bytes in the preamble for each packet
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds

/// Code for the data structure. This should be in the receiver too

//This struct defines a category of measurement, like "temperature" or "humidity."
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

//This is just a way to store 2 bytes together. There are methods to go between the 2-byte datatype to a float. 
struct twobyte {
  uint8_t high;
  uint8_t low;
};
typedef struct twobyte TwoByte;


/////////// Now, we'll set up the data that our sensor will record. 

//Create an instance of Measurement for each of the kinds of measurements your sensor will record
//the format is {min_value, increment, units, measurement name, graph_min, graph_max, hardware name}
Measurement pressure = {80000.0, 1.0, "Pa", "pressure", 90000, 110000, "BME280"};
Measurement temperature = {0.0, 0.01, "Celsius", "temperature", 15, 35, "BME280"};
Measurement humidity = {0, 0.01, "Relative %", "humidity", 30, 100, "BME280"};

//These arrays stores our data in byte (instead of float) form, to more efficiently use RTC memory.
//The RTC_DATA_ATTR prefix keeps this in RTC memory, so it won't get erased when the device sleeps
RTC_DATA_ATTR uint8_t hum_data[MAX_RECORDS*RECORD_SIZE];
RTC_DATA_ATTR uint8_t temp_data[MAX_RECORDS*RECORD_SIZE];
RTC_DATA_ATTR uint8_t pres_data[MAX_RECORDS*RECORD_SIZE];

//These arrays helps us iterate through the types without having to refer to each one specifically
Measurement measurements[3] = {temperature, humidity, pressure};
uint8_t* all_data[3] = {temp_data, hum_data, pres_data}; //Make sure to match the order of measurements!

//////// Finally, some housekeeping items...

//This server is used to get the real time
const char* ntpServer = "pool.ntp.org";

//Make global variables for our various devices. 
HTTPClient http;
SH1106Wire display(OLED_ADDR, SDA, SCL); //7 bit address only
Adafruit_BME280 bme;

//Global variable for a metadata array, used when sending ESP-Now data.
uint8_t metadata[ESP_NOW_MAX_DATA_LEN];

//Keeps track of the state of the device. 
RTC_DATA_ATTR bool device_mode_wifi = 0; //1 for wifi, 0 for trigger
RTC_DATA_ATTR int n_cycles_recorded = 0;

//for ESP Now - MAC address, channel, encrypt - set MAC address to device or to broadcast address
//esp_now_peer_info_t slave = {{0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF}, ESP_NOW_CHANNEL, 0};
esp_now_peer_info_t slave = {{0x9C, 0x9C, 0x1F, 0xC9, 0x50, 0x61}, ESP_NOW_CHANNEL, 0};


//////////////////////// Now for the actual code! /////////////////////////


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
    mode_select();
  } else if (device_mode_wifi) {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { //Either the IR Trigger,  or the Trigger Button
      if (!read_sensors()) {                      //If we fail to read the sensors, don't bother posting, and go to sleep instead.
        go_to_sleep();                            //Sleeping exits this code block. 
      }
      post_sensors();                             //If desired, you can use the bool this returns to check if posting was successful.
      display_graph();
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) { //The timer
      if (!read_sensors()) {
        go_to_sleep();
      }
      post_sensors();
    } else {                                      // Device booted up for the first time
      show_wakeup();
      if (!read_sensors()) {
        go_to_sleep();
      }
      post_sensors();
    }
  } else {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { //Either the IR Trigger,  or the Trigger Button
      ESPNowToMac();
      display_graph();
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) { //The timer
      if (!read_sensors()) {
        go_to_sleep();
      }
    } else {                                      // Device booted up for the first time
      show_wakeup();
      read_sensors();
    } 
  }
  go_to_sleep();
}

///////////////////////////Top level methods///////////////////////

void mode_select() {
  /*
   * Call when the device is woken by the mode select pin. Activates the OLED
   * and allows the user to select the mode. Exits after a few seconds idle.
   */
  Serial.println("Press to select mode, currently " + mode_to_str(device_mode_wifi));
  pinMode(DEVICE_MODE_SELECT_PIN, INPUT);

  init_display();
  display.clear();
  display.drawString(0, 12, "Mode: " + mode_to_str(device_mode_wifi));
  display.display();
  unsigned long start_time = millis();
  unsigned long debounce = millis();
  while (millis() - start_time <= MODE_SELECT_LEN) {
    //if button is pressed, change mode and restart timer
    if (digitalRead(DEVICE_MODE_SELECT_PIN) && millis() - debounce >= DEBOUNCE_TIME) {
      device_mode_wifi = 1 - device_mode_wifi;
      Serial.println("Button pressed. Mode is: " + mode_to_str(device_mode_wifi));
      display.clear();
      display.drawString(0, 12, "Mode: " + mode_to_str(device_mode_wifi));
      display.display();
      start_time = millis();
      debounce = start_time;
    }
  }
  //turn off the OLED display
  display.displayOff();
}


bool read_sensors() {
  /*
   * Read the sensor values and print to the serial. 
   * Returns true if read was successful. 
   */
  if (!bme.begin(BME_ADDR)) {
    Serial.println("Could not find a valid BME sensor!");
    return false;
  }
  Serial.println("Read sensors");
  delay(2000); //The sensor takes 2 seconds to read after we turn it on
  float curr_hum = bme.readHumidity();
  float curr_temp = bme.readTemperature();
  float curr_pres = bme.readPressure();
  if (isnan(curr_hum) || isnan(curr_temp) || isnan(curr_pres)) {
    Serial.println("Failed to read!");
    return false;
  } 
  //Save all the measurements. The function handles the conversion between floats and bytes for us. 
  save_float_to_buffer(curr_temp, temperature, temp_data, MAX_RECORDS*RECORD_SIZE);
  save_float_to_buffer(curr_hum, humidity, hum_data, MAX_RECORDS*RECORD_SIZE);
  save_float_to_buffer(curr_pres, pressure, pres_data, MAX_RECORDS*RECORD_SIZE);

  //Print readings to the serial
  Serial.println("Humidity (RH%): " + String(curr_hum));
  Serial.println("Temperature (C): " + String(curr_temp));
  Serial.println("Pressure (Pa): " + String(curr_pres));

  n_cycles_recorded ++;
  return true;
}

bool post_sensors() {
  /*
   * Call this when the sensor is booted by time. Activates a wifi connection, 
   * loads the sensors, gets their readings, and posts the data. 
   * 
   * Returns true if posting was successful.
   */
  if (connect_to_server()) { //Attempt to connect to server, and proceed only if successful
    configTime(0, 0, ntpServer);
  
    //Now get the last measurements from the data buffers
    float temp = last_float_from_data(temp_data, temperature);
    float hum = last_float_from_data(hum_data, humidity);
    float pres = last_float_from_data(pres_data, pressure);
  
    //Create a batch POST request from a series of 3 post strings. 
    String posts[3] = {create_post_string(temp, temperature, true, true, true), 
                        create_post_string(hum, humidity, true, true, true),
                        create_post_string(pres, pressure, true, true, true)};
    int response = http.POST(multi_post_string(posts, 3));
    if (response != 200) { //200 indicates post was successful. Any other value indicates an error.
      Serial.println("HTTP Post error");
    }
    http.end(); ///End the HTTP client. 
    return (response == 200); //Returns true if response is 200, else false.
  } else {
    return false;
  }
}

void ESPNowBroadcast() {
  /*
   * Send ESP Now data as a broadcast. This doesn't check that a device is ready
   * to listen, which makes it faster and more power efficient. 
   * 
   * The channel should be 0 for this to work correctly.  
   */
  InitESPNow();
  esp_now_add_peer(&slave);
  
  //Send each of the 3 data buffers.
  sendData(temp_data, temperature, 1);
  sendData(hum_data, humidity, 2);
  sendData(pres_data, pressure, 3);
}

void ESPNowToMac() {
  /*
   * Send ESP Now data to a specific, hard-coded MAC address.  This avoids the
   * time/power required to scan all wifi networks, but still checks that a device
   * is ready to listen. 
   * 
   * Currently does not retry or handle pairing failure! 
   */
  InitESPNow();
  esp_now_register_send_cb(OnDataSent); //optional, tells you the status of each packet sent.
  if (slave.channel == ESP_NOW_CHANNEL) { //check for correct channel; add peer
    Serial.print("Slave Status: ");
    esp_err_t addStatus = esp_now_add_peer(&slave);
    if (debug_ESP_error(addStatus)) { //Check if adding was successful
      //Send each of the 3 data buffers
      sendData(temp_data, temperature, 1);
      sendData(hum_data, humidity, 2);
      sendData(pres_data, pressure, 3);
    } else {
      Serial.println("Slave pair failed!");
    }
  } else {
    Serial.println("Non matching slave channel!");
  }
}

void go_to_sleep() {
  /*
   * Send the device to sleep.
   */
  Serial.println("Going to sleep now");
  delay(1000); //wait a second before flushing the serial.
  Serial.flush(); 

  //turn off ADC and wifi
  adc_power_off();
  WiFi.mode(WIFI_MODE_NULL); 
  esp_wifi_stop();
  //Before we sleep, make sure to enable the device to wake up next time!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //timer wakeup
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH); //pin wakeup
  esp_deep_sleep_start();
}


/////////////////////////// ESP METHODS /////////////////////////////////////

void sendData(uint8_t* a, Measurement m, uint16_t index) {
  /*
   * Send the array a on ESP Now in multiple packets because the max
   * packet size is only 250 bytes. 
   * 
   * index: the "Packet ID" preamble which is used in conjunction with MAC Address.
   */

  //First, send the metadata.
  Serial.println("Sending metadata");
  makeMetaData(m, index, MAX_RECORDS*RECORD_SIZE);
  esp_now_send(slave.peer_addr, metadata, ESP_NOW_MAX_DATA_LEN);

  //Continue until you've sent all the bytes in the data buffer
  int total_sent = 0; //bytes of how much data you've sent so far
  while (total_sent < MAX_RECORDS*RECORD_SIZE) {
    //N bytes of data we can send is the bytes remaining, capped at the max ESP-now length minus the preamble length
    int remaining = MAX_RECORDS*RECORD_SIZE - total_sent; //N bytes not yet sent in the buffer
    int data_len = min(ESP_NOW_MAX_DATA_LEN - PREAMBLE_LEN, remaining);
    Serial.println("Sending " + String(data_len) + " bytes of " + String(MAX_RECORDS*RECORD_SIZE));

    //create the actual packet to send: first 2 bytes are the preamble, the rest is just the array
    uint8_t to_send[ESP_NOW_MAX_DATA_LEN] = {0};
    memset(&to_send[0], index, sizeof(uint8_t)); // index number of the data
    memcpy(&to_send[PREAMBLE_LEN], &a[total_sent], data_len);
    esp_now_send(slave.peer_addr, &to_send[0], data_len + PREAMBLE_LEN); //amount to send is the amount of data plus the preamble!

    //To help see what we're sending, print out the first data point (after the preamble)
    TwoByte s = {to_send[2], to_send[3]};
    Serial.print("Sending "); Serial.println(twobyte_to_float(s, m));

    //Update how many bytes we've sent
    total_sent += data_len;
   }
}

uint8_t* makeMetaData(Measurement m, uint16_t index, int data_len) {
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
   *  index is an arbitrary number to separate different sets of data from each other. 
   *  
   *  data_len is the total number of bytes we will be sending after the metadata. 
   *  
   *  Remember that any changes to the structure of this metadata must also be reflected 
   *  in the collector code!
   */
  //Create all the parameters we'll be sending; they need to be char arrays instead of Strings
  const char* device_name_c = device_name.c_str();
  const char* measurement_type_c = m.type.c_str();
  const char* unit_c = m.unit.c_str();
  const char* hardware_name_c = m.hardware_name.c_str();
  //N packets is the total N bytes to send, divided by the bytes in each packet. Round up by adding 1 if the division isn't even.
  uint8_t n_packets = data_len/ESP_NOW_MAX_DATA_LEN + (data_len%ESP_NOW_MAX_DATA_LEN != 0);
  
  int byte_pos = 0; //As we memcpy stuff into the metadata, increment this to see where the next field is

  // First is the preamble. It's a 2-byte int. 
  memset(&metadata[byte_pos], index, sizeof(uint16_t)); 
  byte_pos += sizeof(uint16_t);
  
  // Next, copy the device name string. This function handles special cases for us. 
  set_null_terminated_metadata_string(byte_pos, device_name_c, 32);
  byte_pos += 32;
  //Next, lat and lon floats
  memcpy(&metadata[byte_pos], &lat, sizeof(float)); // lat
  byte_pos += sizeof(float);
  memcpy(&metadata[byte_pos], &lon, sizeof(float)); // lon
  byte_pos += sizeof(float);
  //More strings. Call the same function as above.
  set_null_terminated_metadata_string(byte_pos, measurement_type_c, 32);
  byte_pos += 32;
  set_null_terminated_metadata_string(byte_pos, unit_c, 32);
  byte_pos += 32;
  memcpy(&metadata[byte_pos], &m.min_value, sizeof(float)); // min value
  byte_pos += sizeof(float);
  memcpy(&metadata[byte_pos], &m.resolution, sizeof(float)); // resolution
  byte_pos += sizeof(float);
  set_null_terminated_metadata_string(byte_pos, hardware_name_c, 32);
  byte_pos += 32;
  memcpy(&metadata[byte_pos], &n_packets, sizeof(uint8_t)); // n packets
  
  Serial.println("Successfully created metadata");
}

void InitESPNow() {
  /*
   * Just initialize the ESP Now
   */
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect(); //this is only necessary if you're directly switching from wifi to esp now
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  /*
   * Use this to check if packets were sent successfully. Call it with 
   * esp_now_register_send_cb(OnDataSent) and this function will be called
   * every time you send a packet. 
   */
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool debug_ESP_error(esp_err_t err) {
  /*
   * Just look at the error value and print a readable message corresponding to it
   */
  if (err == ESP_OK) {
    Serial.println("Successfully paired");
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
    Serial.println("Unknown error");
    return false;
  }
}

////////////////////////// DATA METHODS /////////////////////////////

void save_float_to_buffer(float v, Measurement m, uint8_t* a, int a_len) {
  /*
   * Given a float measurement, save it in the data buffer. 
   * 
   * Float: the float value measurement to save (e.g. 25.6, meaning 25.6 degrees C)
   * Measurement: the type of measurement (e.g. temperature - should be instantiated as a struct)
   * a: The data array you are storing the measurements in
   * a_len: Number of bytes in a (NOT the number of measurements!)
   * 
   * Converts to twoByte, then saves the twoByte to the buffer
   */

  TwoByte v_short = float_to_twobyte(v, m);
  save_bytes_to_buffer(a, v_short, a_len);
}

void save_bytes_to_buffer(uint8_t* a, TwoByte v, int a_len) {
  /*
   * Uses the specified array as a circular buffer. Pushes all its entries
   * back by 2 bytes, then stores the TwoByte in the first 2 bytes. 
   */
   //copy the array RECORD_SIZE spaces back. 
   memcpy(&a[RECORD_SIZE], a, sizeof(a[0])*(a_len-RECORD_SIZE));
   //save the TwoByte in the first two bytes. HIGH bits in position 0; LOW in position 1.
   a[0] = v.high;
   a[1] = v.low;
}

TwoByte float_to_twobyte(float v, Measurement m) {
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
   * Go into the specified array which is made up of TwoBytes, 
   * and return the first element in float format.
   * 
   * example: float_from_short_array(hum_data, humidity)
   */
   TwoByte s = {a[0], a[1]};
   return twobyte_to_float(s, m);
}

float twobyte_to_float(TwoByte s, measurement m) {
  /*
   * Given a measurement type and a twoByte, convert the twoByte we get
   * from the internal representation of data to a float used
   * for display. 
   */
   uint16_t v = (s.high << 8) + s.low;
   float x = v*m.resolution + m.min_value;
   return x;
}

void set_null_terminated_metadata_string(int byte_pos, const char* str, int max_str_len) {
  /*
   * Set the bytes in metadata starting at byte_post equal to the string str, but force
   * the last bytes of the string, max_len bytes after the start position, to be null-terminated, 
   * even if the string is longer than max_len. 
   */
  //First, copy up to max_str_len-1 bytes to metadata. Copy strlen + 1 chars to include the null terminator (strlen doesn't count it)
  memcpy(&metadata[byte_pos], str, min((size_t) max_str_len, strlen(str)+1));  
  //then set the last char to the null terminator, in case the string was longer than the number of characters
  memset(&metadata[byte_pos + max_str_len-1], '\0', 1);
}

/////////////////////////// Regular wifi methods /////////////////////////////

bool connect_to_server() {
  /*
   * Connect to the wifi and the server. Gives up and exits after several. 
   * Returns true if connection was successful. 
   */
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

unsigned long getTime() {
  /*
   * Get the current time, in epoch time (seconds since 1970). 
   * Returns 0 if unable to obtain time. 
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
   * Take an array of json-formatted post strings (using create_post_string method) and combine
   * them into a json-formatted list, in order to batch post them. 
   */
   String out = "["; //The json-formatted list starts and ends with a square bracket
   for (int i=0; i<n_posts-1; i++) {
    out = out + posts[i] + ", "; //add each of the posts, separated by a comma
   }
   out = out + posts[n_posts-1] + "]"; //remove the comma from the last post, and close the bracket
   return out;
}

String create_post_string(float value, Measurement m, bool incl_time, bool incl_location, bool incl_hardware) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are global variables, at the top of this sketch.
   * 
   * incl_time, incl_location, incl_hardware: These are all optional parameters, so specify
   * whether or not to include them. 
   */
  //Start with {, and add in key-value pairs for all of the relevant components.
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m.type;
  json = json + "\",\"unit\":\"" + m.unit + "\",\"value\":\"" + String(value);
  if (incl_time) { // get the current time and include it if told to do so
    unsigned long curr_time = getTime();
    if (curr_time > 0) { //getTime returns 0 if it was unable to retrieve the time; don't add it in this case
      json = json + "\",\"timestamp\":\"" + String(curr_time);
    }
  }
  if (incl_location) { //if including location, add both lat and lon
    json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    
  }
  if (incl_hardware) { // add the hardware
    json = json + "\",\"hardware\":\"" + m.hardware_name;
  }
  json = json + "\"}";
  return json;
}

////////////////////// OLED display methods ////////////////////////////

void show_wakeup() {
  /*
   * Just show a message on the OLED which mode you're currently in, and print it on the Serial.
   */
  init_display();
  display.clear();
  Serial.println("Booting up in " + mode_to_str(device_mode_wifi) + " Mode.");
  display.drawString(0, 6, "Booting up.");
  display.drawString(0, 24, mode_to_str(device_mode_wifi) + " Mode.");
  display.display();
  delay(DISPLAY_LEN); //pause for a few seconds before shutting off the display
  display.displayOff();
}

void display_graph() {
  /*
   * Show a graph of all the sensor readings. 
   */
  Serial.println("Displaying graph");
  init_display();
  display.setFont(ArialMT_Plain_10);

  //for each of the measurements, draw a graph of the data in that measurement's buffer
  for (int i = 0; i<(sizeof(measurements)/sizeof(measurements[0])); i++) {
    display.clear();
    add_graph(all_data[i], measurements[i]);
    display.display();
    delay(DISPLAY_LEN);
  }
  display.displayOff();
}

void add_graph(uint8_t* a, Measurement m) {
  /*
   * Plot the given array onto the graph
   * Does not actually display anything yet - just adds the element to the OLED screen
   */
  //This is the x and y axis
  display.drawLine(0, SCREEN_HEIGHT-POS_X_AXIS, SCREEN_WIDTH, SCREEN_HEIGHT-POS_X_AXIS); //x 
  display.drawLine(POS_Y_AXIS, 0, POS_Y_AXIS, SCREEN_HEIGHT); //y

  //Display the graph title
  display.drawString(TITLE_X, TITLE_Y, m.type);

  //We create the graph by drawing lines between the last point and the next point, iterating through all the recorded data. 
  //Start by setting last_x, last_y to the most recent data.
  //Get the last recorded float, and convert it into screen coordinates. Use the graph upper and lower bounds in the measurement struct.
  float last_y = float_to_screen_y(last_float_from_data(a, m), m.graph_lower, m.graph_upper);
  float last_x = POS_Y_AXIS;
  for (int i=1; i < min(MAX_RECORDS, n_cycles_recorded); i++) {
    //convert the index to screen coordinates
    float x_pos = float_to_screen_x(i);
    //Get the i-th to last float from data by passing the address of the next location in the array
    //The data array stores 2 bytes for every measurement, so we need to iterate by 2.
    float y_pos = float_to_screen_y(last_float_from_data(&a[i*2], m), m.graph_lower, m.graph_upper);
    //Draw the line, and update the positions of the last coordinate.
    display.drawLine(last_x, last_y, x_pos, y_pos);
    last_y = y_pos;
    last_x = x_pos;
  }
}

float float_to_screen_x(int i) {
  /*
   * Given the iteration of the data we are currently on, return the position on the screen
   * that data point should be plotted
   */
  //0 should map to the y-axis, and the max x-value should map to the edge of the screen
  return map(i, 0, min(MAX_RECORDS, n_cycles_recorded)-1, POS_Y_AXIS, SCREEN_WIDTH);
}

float float_to_screen_y(float v, float min_value, float max_value) {
  /*
   * Given a float value and min/max values of a measurement, return
   * the position on the y axis that the measurement would be plotted
   * 
   * currently does not handle negative values
   */
  //the min value maps to the x-axis, and the max value maps to the top of the screen
  float mapped = map(v, 0, max_value, POS_X_AXIS, SCREEN_HEIGHT);
  //positions on the screen start from 0 at the top instead of the bottom, so flip the value
  return SCREEN_HEIGHT - mapped;
}

void display_last_reading_and_sleep() {
  /*
   * Call this when the sensor is booted by external trigger. 
   * Activate the OLED display, show readings, and then turn off the display.
   */
  init_display();
  display_readings(
    last_float_from_data(hum_data, humidity), 
    last_float_from_data(temp_data, temperature), 
    last_float_from_data(pres_data, pressure));
  delay(DISPLAY_LEN); //pause before shutting off the display
  display.displayOff();
}

void display_readings(float h, float t, float p) {
  /*
   * Display the current humidity and temperature on the OLED. 
   */
  display.clear();
  display.drawString(0, 12, "Humidity: " + String(h) + "%");
  display.drawString(0, 28, "Temp: " + String(t) + "C");
  display.drawString(0, 44, "Pressure: " + String(p) + "Pa");
  display.display();
}

void init_display() {
  /*
   * Initialize the display.
   */
  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
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
  /*
   * Print the reason that the sensor woke up from sleep
   */
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
