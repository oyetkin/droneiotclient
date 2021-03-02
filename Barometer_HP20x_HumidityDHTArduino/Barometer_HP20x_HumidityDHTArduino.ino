/*
   Demo name   : HP20x_dev demo
   Usage       : I2C PRECISION BAROMETER AND ALTIMETER [HP206C hopeRF]
   Author      : Oliver Wang from Seeed Studio
   Version     : V0.1
   Change log  : Add kalman filter 2014/04/04
*/

#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h"
#include <KalmanFilter.h>


#include "DHT.h"

#define DHTPIN A0     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);


unsigned char ret = 0;

/* Instance */
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter


void setup()
{
  Serial.begin(9600);        // start serial for output

  //Serial.println("Temperature,Pressure,Humidity");
  //Serial.println("****HP20x_dev demo by seeed studio****\n");
  //Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
  /* Power up,delay 150ms,until voltage is stable */
  delay(150);
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(100);

  /* Determine HP20x_dev is available or not */
  ret = HP20x.isAvailable();
  if (OK_HP20X_DEV == ret)
  {
    //Serial.println("HP20x_dev is available.\n");
  }
  else
  {
    Serial.println("HP20x_dev isn't available.\n");
  }
   dht.begin();
}


void loop()
{

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();


   // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(h)) 
    {
        Serial.println("Failed to read from DHT");
    } 
   
    
  char display[40];
  if (OK_HP20X_DEV == ret)
  {
    long Temper = HP20x.ReadTemperature();
    //Serial.print("Temper: ");
    float t = Temper / 100.0;
    Serial.print(t);

    long Pressure = HP20x.ReadPressure();
    //Serial.print(" Pressure:");

    float p = Pressure / 100.0;
    Serial.print(",");
    Serial.print(p);
    Serial.print(",");
    Serial.print(h);
    Serial.println();
    //Serial.println(" hPa.\n");
  }
  delay(5000);
}

