/*
 * Miscellaneous script for testing various features. 
 * Currently testing the IR LED stuff.
 * 
 */
#include <IRremote.h>

//Pin definitions
#define IR_RECEIVE_PIN 15
#define SEND_BUTTON_PIN 16 // pull this low to trigger send event
//apparently pin 3 PWM is the IR send pin
#define STATUS_PIN 2 //internal LED - flash when sending

//State tracking
int lastButtonState;

// Storage for the recorded code
struct storedIRDataStruct {
    IRData receivedIRData;
    // extensions for sendRaw
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
    uint8_t rawCodeLength; // The length of the code
} sStoredIRData;

void setup(){
  Serial.begin(115200);
  Serial.println(IR_SEND_PIN);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver, enable feedback LED, take LED feedback pin from the internal boards definition
  IrSender.begin(true); // Enable feedback LED at default feedback LED pin
  pinMode(SEND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_PIN, OUTPUT);
 
}

void sendCode(storedIRDataStruct *aIRDataToSend) {
  /*
   * Send using NEC protocol
   * Data received: 
   * Protocol=NEC 
   * Address=0xC7EA 
   * Command=0x33 
   * Raw-Data=0xCC33C7EA 
   * 32 bits LSB first
   */
  aIRDataToSend->receivedIRData.protocol = NEC
  IrSender.write(&aIRDataToSend->receivedIRData, NO_REPEATS);
  Serial.print("Sent: ");
  printIRResultShort(&Serial, &aIRDataToSend->receivedIRData);
}

void loop(){
  
}
