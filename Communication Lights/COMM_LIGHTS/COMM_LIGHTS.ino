/********************************************
  Communication Lights
  Thesis
  Adi Azulay
*********************************************/

#include <FastLED.h>
#include <RFM69.h>
#include <SPI.h>

//Uncomment DEBUG to activate debugging mode
//#define DEBUG

#define DATA_PIN 8
#define CLOCK_PIN 7
#define BUTTON_A_PIN 3
#define BUTTON_B_PIN 4
#define BUTTON_C_PIN 5
#define BUTTON_D_PIN 6

//Radio Variables
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
//Radio Pins
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9

int16_t packetnum = 0;

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

bool wired = false;

static const int numLEDS = 48;

char allButtonValues[20];

CRGB leds[numLEDS];

void setup() {

  Serial.begin (115200);

  Serial.println("Arduino RFM69HCW Transmitter");
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  radio.encrypt(ENCRYPTKEY);

  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");


  pinMode (BUTTON_A_PIN, INPUT);
  LEDS.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, numLEDS);
  LEDS.setBrightness (84);
  FastLED.clear(true);
  FastLED.show();
  allButtonValues[0] = '%';
  Serial.println("Setup Complete");
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

  if (buttonA >= true) {
    //Serial.print ('A');
    allButtonValues[1] = 'A';
    for (int i = 0; i < 3; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (buttonA == false) {
    //Serial.print ('a');
    allButtonValues[1] = 'a';
    for (int i = 0; i < 3; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }
  if (buttonB >= true) {
    //Serial.print('B');
    allButtonValues[2] = 'B';
    for (int i = 3; i < 6; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (buttonB == false) {
    //Serial.print('b');
    allButtonValues[2] = 'b';
    for (int i = 3; i < 6; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }
  if (buttonC >= true) {
    allButtonValues[3] = 'C';
    for (int i = 6; i < 9; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (buttonC == false) {
    allButtonValues[3] = 'c';
    for (int i = 6; i < 9; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }
  if (buttonD >= true) {
    allButtonValues[4] = 'D';
    for (int i = 9; i < 12; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (buttonD == false) {
    allButtonValues[4] = 'd';
    for (int i = 9; i < 12; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }

  Serial.println (allButtonValues);
  itoa(packetnum++, allButtonValues+13, 10);
  
  //Serial.print("Sending "); Serial.println(allButtonValues);
  if (radio.sendWithRetry(RECEIVER, allButtonValues, strlen(allButtonValues))) { //target node Id, message as string or byte array, message length
    Serial.println("OK");
  }

  radio.receiveDone(); //put radio in RX mode
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
  delay(200);
}

bool checkState(int button) {
  int buttonState = digitalRead (button);
  if (buttonState >= 1) {
    return true;
  } else {
    return false;
  }
}
