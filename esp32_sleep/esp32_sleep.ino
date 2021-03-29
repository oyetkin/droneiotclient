/*
 * Install the following libraries: 
 * "Wifi" (builtin)
 * "HTTP Client" (builtin on some devices)
 * "Adafruit BusIO"
 * "Adafruit Unified Sensor"
 * "ESP8266 and ESP32 OLED driver for SSD1306 displays"
 * "Adafruit BME280"
 * 
 * TODO: 
 *    --deep sleep issues: https://github.com/espressif/arduino-esp32/issues/1113
 *    --handle error on failure to init esp now
 *    --can clean up display_readings
 *    
 */
#include <Wire.h>  
#include "WiFi.h"
#include "HTTPClient.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SH1106Wire.h"
#include "time.h"
#include <esp_now.h>
#include "driver/adc.h"
#include "esp_wifi.h"

// PINOUTS
#define DEVICE_MODE_SELECT_PIN GPIO_NUM_27
#define TRIGGER_PIN GPIO_NUM_14 //remote trigger circuit read from this pin 
#define OLED_BUTTON GPIO_NUM_13 //trigger button read from this pin
#define ESP_NOW_CHANNEL 0
#define WAKE_PIN_BITMASK 0x008006000 // 2^OLED_BUTTON + 2^TRIGGER_PIN in hex

//OLED STUFF
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//how far is the axis away from the margin?
#define POS_X_AXIS       5 
#define POS_Y_AXIS       1
#define TITLE_X 10
#define TITLE_Y 6


//OTHER CONFIGURABLE
#define MAX_WIFI_RETRIES 30 //number of times to try connecting to wifi before giving up
#define TIME_TO_SLEEP  60 //time to sleep in seconds
#define DISPLAY_LEN 2000 //time to show OLED in millis
#define MODE_SELECT_LEN 4000 //time after a button press that the device returns to sleep
#define DEBOUNCE_TIME 400 //n millis to wait between 2 button presses
#define MAX_RECORDS 298 // total readings for EACH of the sensor data. won't compile if too high
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds


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

//make a measurement for each of your sensors. Max is ~65K increments.
Measurement pressure = {100000.0, 1.0, "Pa", "pressure", 90000, 110000, "BME280"};
Measurement temperature = {0.0, 0.01, "Celsius", "temperature", 15, 35, "BME280"};
Measurement humidity = {0, 0.01, "Relative %", "humidity", 30, 100, "BME280"};
Measurement measurements[3] = {temperature, humidity, pressure};
////////////////////

//WIFI SETTINGS
const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor/batch";
const char* ntpServer = "pool.ntp.org";

//DEVICE SETTINGS
const String device_name = "Arjun_weather_kit";
float lat = 32.636462;
float lon = -117.095660;

HTTPClient http;
SH1106Wire display(0x3c, SDA, SCL); //7 bit address only
Adafruit_BME280 bme;
RTC_DATA_ATTR uint8_t hum_data[MAX_RECORDS];
RTC_DATA_ATTR uint8_t temp_data[MAX_RECORDS];
RTC_DATA_ATTR uint8_t pres_data[MAX_RECORDS];
uint8_t* all_data[3] = {temp_data, hum_data, pres_data}; //match the order of measurements variable above!

//state tracking
RTC_DATA_ATTR bool device_mode_wifi = 1; //1 for wifi, 0 for trigger
RTC_DATA_ATTR int n_cycles_recorded = 0;

//for ESP Now - MAC address, channel, encrypt - set MAC address to device or to broadcast address
esp_now_peer_info_t slave = {{0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF}, ESP_NOW_CHANNEL, 0};
//esp_now_peer_info_t slave = {{0x9C, 0x9C, 0x1F, 0xC9, 0x50, 0x61}, ESP_NOW_CHANNEL, 0};

void setup() {
  /*
   * All the device logic is contained here
   */
  Serial.begin(115200);
  delay(1000);

  //get and report wakeup cause
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.print("Wakeup cause: interrupt. ");
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.print("Wakeup cause: timer. ");
  }
  Serial.println("Mode: " + mode_to_str(device_mode_wifi));

  //check if we want to change the mode, and change it. 
  int pin_triggered = log(esp_sleep_get_ext1_wakeup_status())/log(2);
  if (pin_triggered == DEVICE_MODE_SELECT_PIN) {
    mode_select(&display); // this also sends us to sleep
    go_to_sleep(device_mode_wifi);
  } else if (device_mode_wifi) {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) { // either trigger or button
      collect_and_post_sensors();
      display_graph_and_sleep(&display);
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
      collect_and_post_sensors();
    } else { // on device startup
      show_wakeup(&display, device_mode_wifi);
      collect_and_post_sensors();
    }
  } else {
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
      display_graph_and_sleep(&display);
      ESPNowBroadcast();
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
      read_sensors();
    } else { // on device startup
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
  unsigned long start_time = millis();
  unsigned long debounce = millis();
  //turn on the display and show the current mode
  d->init();
  d->flipScreenVertically();
  d->clear();
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0, 12, "Mode: " + mode_to_str(device_mode_wifi));
  d->display();
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
  if (!bme.begin(0x76)) {
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
  //convert these to uint8_t tuple formats (SplitShort) and save in array
  SplitShort hum_short = float_to_short(curr_hum, humidity);
  SplitShort temp_short = float_to_short(curr_temp, temperature);
  SplitShort pres_short = float_to_short(curr_pres, pressure);

  push_back_short(hum_data, hum_short, MAX_RECORDS);
  push_back_short(temp_data, temp_short, MAX_RECORDS);
  push_back_short(pres_data, pres_short, MAX_RECORDS); 
  
  //Print readings to the serial
  Serial.println("Humidity (RH%): " + String(curr_hum));
  Serial.println("Temperature (C): " + String(curr_temp));
  Serial.println("Pressure (Pa): " + String(curr_pres));

  n_cycles_recorded ++;
}

void collect_and_post_sensors() {
  /*
   * Call this when the sensor is booted by time. Activates a wifi connection, 
   * loads the sensors, gets their readings, and posts the data. 
   */
  read_sensors();

  //Connect to wifi and get the time
  connect_to_server();
  configTime(0, 0, ntpServer);

  //Now post all to the server
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
   * Send ESP Now data as a broadcast. Doesn't check for matching 
   * Slave is set globally at the top of this script. Set the channel to 0. 
   * The receiver channel does not have to be the same. 
   * 
   * We don't check for pairing success because in broadcast mode, adding 
   * the "peer" (the broadcast address) is always successful. 
   */
  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_add_peer(&slave);
  sendDataMulti(hum_data);
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
      uint8_t send_this[2] = {17, 18};
      esp_now_send(slave.peer_addr, send_this, sizeof(send_this));
    } else {
      Serial.println("Slave pair failed!");
    }
  } else {
    Serial.println("Non matching slave channel!");
  }
}

void go_to_sleep(bool wifi_mode) {
  /*
   * Send the device to sleep. Wifi_mode doesn't do anything currently.
   */
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  //turn off adcs and wifi
  adc_power_off();
  WiFi.mode(WIFI_MODE_NULL);
  esp_wifi_stop(); //this may not be necessary
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext1_wakeup(WAKE_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

////////////////////////// DATA METHODS /////////////////////////////

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

SplitShort float_to_short(float v, measurement m) {
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

void sendDataMulti(uint8_t* a) {
  /*
   * Send the array a on ESP Now in multiple packets - because the max
   * packet size is only 250 bytes. 
   * Note that the math only works because a is a byte array!
   */
   int total_sent = 0;
   while (total_sent < MAX_RECORDS) {
    int remaining = MAX_RECORDS - total_sent;
    Serial.print("Sending: "); Serial.println(a[total_sent]);
    esp_now_send(slave.peer_addr, &a[total_sent], min(ESP_NOW_MAX_DATA_LEN, remaining));
    total_sent += ESP_NOW_MAX_DATA_LEN;
   }
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

void display_graph_and_sleep(SH1106Wire* d) {
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
