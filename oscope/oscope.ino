// script to run the arduino like an oscope

#define READ_IN 14
#define IR_RECEIVE_PIN 15
#include <IRremote.h>
#define STATUS_PIN 2 //internal LED - flash when sending

void setup() {
  Serial.begin(115200);
  pinMode(READ_IN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(STATUS_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int read_value = analogRead(READ_IN);
  Serial.println(photo_value);
}
