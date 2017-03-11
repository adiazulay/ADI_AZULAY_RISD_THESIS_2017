#include <FastLED.h>

#define DEBUG

#define DATA_PIN 8
#define CLOCK_PIN 9
#define BUTTON_A_PIN 2
#define BUTTON_B_PIN 3
#define BUTTON_C_PIN 4
#define BUTTON_D_PIN 5

static const int numLEDS = 48;

CRGB leds[numLEDS];

void setup() {

  #ifdef DEBUG
    Serial.begin (9600);
  #endif

  pinMode (BUTTON_A_PIN, INPUT);
  LEDS.addLeds<LPD8806,DATA_PIN, CLOCK_PIN, RGB>(leds, numLEDS);
  LEDS.setBrightness (84);
  FastLED.show();


}

void loop() {
  int val = digitalRead (BUTTON_A_PIN);

  #ifdef DEBUG
    Serial.println (val);
  #endif

  if (val >= 1){
    for(int i = 0; i < 3; i++){
      leds[i] = CRGB(255,255,255);
      FastLED.show();
    }
  } else {
      for(int i = 0; i < 48; i++){
      leds[i] = CRGB(0,0,0);
      FastLED.show();
    }
  }

}
