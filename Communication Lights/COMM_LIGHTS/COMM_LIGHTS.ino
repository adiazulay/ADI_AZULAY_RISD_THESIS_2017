#include <FastLED.h>

//Uncomment DEBUG to activate debugging mode
//#define DEBUG

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
  FastLED.clear(true);
  FastLED.show();


}

void loop() {
  bool buttonA = checkState (BUTTON_A_PIN);
  bool buttonB = checkState (BUTTON_B_PIN);
  bool buttonC = checkState (BUTTON_C_PIN);
  bool buttonD = checkState (BUTTON_D_PIN);

  #ifdef DEBUG
    Serial.println (buttonA);
    Serial.println (buttonB);
    Serial.println (buttonC);
    Serial.println (buttonD);
  #endif

  if (buttonA >= true){
    for(int i = 0; i < 3; i++){
      leds[i] = CRGB(255,255,255);
      FastLED.show();
    }
  } else if (buttonA == false){
      for(int i = 0; i < 3; i++){
      leds[i] = CRGB(0,0,0);
      FastLED.show();
    }
  }
  if (buttonB >= true){
    for(int i = 3; i < 6; i++){
      leds[i] = CRGB(255,255,255);
      FastLED.show();
    }
  } else if (buttonB == false){
      for(int i = 3; i < 6; i++){
      leds[i] = CRGB(0,0,0);
      FastLED.show();
    }
  }
  if (buttonC >= true){
    for(int i = 6; i < 9; i++){
      leds[i] = CRGB(255,255,255);
      FastLED.show();
    }
  } else if (buttonC == false){
      for(int i = 6; i < 9; i++){
      leds[i] = CRGB(0,0,0);
      FastLED.show();
    }
  }
  if (buttonD >= true){
    for(int i = 9; i < 12; i++){
      leds[i] = CRGB(255,255,255);
      FastLED.show();
    }
  } else if (buttonD == false){
      for(int i = 9; i < 12; i++){
      leds[i] = CRGB(0,0,0);
      FastLED.show();
    }
  }
}

bool checkState(int button){
  int buttonState = digitalRead (button);
  if (buttonState >= 1){
    return true;
  } else {
    return false;
  }
}
