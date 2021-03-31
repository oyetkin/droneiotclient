///Use this to post many measurements as if from several sensors...
#include "WiFi.h"
#include "HTTPClient.h"


const char* ssid = "ATT5yX6g8p";
const char* password =  "35fcs6hyi#yj";
const char* server = "https://api.is-conic.com/api/v0p1/sensor";
HTTPClient http;

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

Measurement pressure = {100000.0, 1.0, "Pa"};
Measurement temperature = {0.0, 0.01, "Celsius"};
Measurement humidity = {0, 0.01, "Relative %"};
Measurement measurements[3] = {temperature, humidity, pressure};

//add your sensor locations here
float lat1 = 32.709082;
float lon1 = -117.085275;
float lat2 = 32.709693;
float lon2 = -117.159829;
float lat3 = 32.693231;
float lon3 = -117.201190;
float lat4 = 32.670180;
float lon4 = -117.239755;
float lat5 = 32.689846;
float lon5 = -117.173752;
float lat6 = 32.721920;
float lon6 = -117.125176;
float lat7 = 32.740256;
float lon7 = -117.225782;
float lat8 = 32.766110;
float lon8 = -117.136914;

void setup() {
  Serial.begin(115200);
  delay(1000);

  /*
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); 
  Serial.println(WiFi.softAPmacAddress());*/
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
    n_records_recd = 0;
    //activate the LED pwm pin
    ledcWrite(PWM_CHANNEL, 128);
    delay(LED_ON_TIME);
    ledcWrite(PWM_CHANNEL, 0);
  } else if (upload) {
    Serial.println("Upload pressed!");
    if (connect_to_server()) {
      
      int response_1 = http.POST(create_post_string("san_diego_1", n_records_recd + 10, temperature, lat1, lon1, false));
      http.POST(create_post_string("san_diego_2", 30 - n_records_recd/2.0, temperature, lat2, lon2, false));
      http.POST(create_post_string("san_diego_3", 30 - n_records_recd/4.0, temperature, lat3, lon3, false));
      http.POST(create_post_string("san_diego_4", 25 - n_records_recd/2.0, temperature, lat4, lon4, false));
      http.POST(create_post_string("san_diego_5", 15 - n_records_recd*2, temperature, lat5, lon5, false));
      http.POST(create_post_string("san_diego_6", n_records_recd/4.0 + 25, temperature, lat6, lon6, false));
      http.POST(create_post_string("san_diego_7", n_records_recd/2.0 + 15, temperature, lat7, lon7, false));
      http.POST(create_post_string("san_diego_8", n_records_recd*2.0 + 10, temperature, lat8, lon8, false));
      n_records_recd += 1;
      
      if (response_1 != 200) {
        Serial.println("HTTP Post error");
      } else {
        Serial.println("Successfully posted.");
      }
      http.end();
    }
  }
}

bool connect_to_server() {
  /*
   * Connect to the wifi and Otto's server. Return true if successful
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

String create_post_string(String device_name, float value, Measurement m, float lat, float lon, bool incl_hardware) {
  /*
   * Convert the input measurement to a json-compatible string. 
   * The sensor name and location are hard-coded at the top of this sketch.
   */
  String json = "{\"key\":\"" + device_name + "\",\"measurement_name\":\"" + m.type;
  json = json + "\",\"unit\":\"" + m.unit + "\",\"value\":\"" + String(value);

  json = json + "\",\"lat\":\"" + String(lat) + "\",\"lon\":\"" + String(lon);    

  if (incl_hardware) {
    json = json + "\",\"hardware\":\"" + m.hardware_name;
  }
  json = json + "\"}";
  return json;
}
