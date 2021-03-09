/*
 * Miscellaneous script for testing various features. 
 * Currently testing the IR LED stuff.
 * 
 */

#include "IRLremote.h" 
 
const int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;
IRsend mySender;

#define pinIR 2
#define pinSendIR 3
#define IRL_DEBOUCE 300

void setup(){
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
}

void loop(){
  if (irrecv.decode(&results)){
        Serial.println(results.value, HEX);
        irrecv.resume();
  }
  if (Serial.read() != -1) {
    //send a code every time a character is received from the serial port
    //Sony DVD power A8BCA
    Serial.println("try to send");
    mySender.send(SONY,0xa8bca, 17);
  }
}
