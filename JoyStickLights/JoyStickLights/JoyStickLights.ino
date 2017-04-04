/*
  Pitch follower

  Plays a pitch that changes based on a changing analog input

  circuit:
   8-ohm speaker on digital pin 9
   photoresistor on analog 0 to 5V
   4.7K resistor on analog 0 to ground

  created 21 Jan 2010
  modified 31 May 2012
  by Tom Igoe, with suggestion from Michael Flynn

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Tone2

*/

#include "FastLED.h"
#define NUM_LEDS 24
#define DATA_PIN 5
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];

int hue;
bool runX;
bool runY;

void setup() {
  // initialize serial communications (for debugging only):
  Serial.begin(9600);
  FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, GRB>(leds, NUM_LEDS);
  LEDS.setBrightness(84);
}

void loop() {
  //  for(int i = 0; i < 13; i++) {
  //    // Set the i'th led to red
  //    leds [i] = CHSV(0, 0, 0);
  //    // Show the leds
  //    FastLED.show();
  //  }
  // read the sensor:
  long x = analogRead(A3) - 512;
  long y = analogRead(A4) - 512;
  float angle = atan2( y, x );                 // in radians, zero is joystick move to right

  /*
    float magnitude = sqrt( x*x + y*y);    // Pythagoras

    if( magnitude > 512) magnitude = 512;

    x = magnitude * cos( angle );
    y = magnitude * sin( angle );
    angle = atan2( y, x );
  */

  angle = angle * 57296 / 1000;
/*
  Serial.print(angle);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
*/
  //int sensorReading = analogRead(A3);
  // print the sensor reading so you know its range
  //Serial.println(sensorReading);
  // map the analog input range (in this case, 400 - 1000 from the photoresistor)
  // to the output pitch range (120 - 1500Hz)
  // change the minimum and maximum input numbers below
  // depending on the range your sensor's giving:
  if ( x <= 50 && x >= -50){ runX = false;}else{runX = true;}
  if(y <= 50 && y >= -50){ runY = false;}else{runY = true;}
  if(runX == true || runY ==true){
  if (digitalRead(7) == HIGH) {
    angle = map(angle, -179, 180, 13, 24);
    Serial.println(angle);
    int ledOn = int(angle);
    for (int i = 13; i < NUM_LEDS; i++) {
      // Set the i'th led to red
      leds [i] = CHSV(0, 0, 0);
    }
    leds [ledOn] = CHSV(hue, 123, 200);
    FastLED.show();

  }
  else {
    angle = map(angle, -179, 180, 0, 255);
    hue = int(angle);

    for (int i = 13; i < NUM_LEDS; i++) {
      // Set the i'th led to red
      leds [i] = CHSV(hue, 123, 200);
      // Show the leds
      FastLED.show();
    }

  }
  } 
//  else {
//    for (int i = 13; i < NUM_LEDS; i++) {
//      leds [i] = CHSV(0, 0, 0);
//      FastLED.show();
//    }
//  }
  //tone(9, thisPitch, 10);
  delay(1);        // delay in between reads for stability
}






