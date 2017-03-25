/*
   Reciever Code
*/

#include <FastLED.h>
#include <RFM69.h>
#include <SPI.h>

//Uncomment DEBUG to activate debugging mode
//#define DEBUG


//LED SETTINGS
#define DATA_PIN 8
#define CLOCK_PIN 7

//Wireless Transiciver Settings
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        1    // The unique identifier of this node
#define RECEIVER      2    // The recipient of packets
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//Wireless Pins
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);




static const int numLEDS = 12;

int incomingByte;
char* dataPack;
char incomingButtonValues[6];

CRGB leds[numLEDS];

void setup() {

  Serial.begin (115200);
  Serial.println("Starting Up");

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

  Serial.print("\nListening at ");
  Serial.print(FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");
  
  //Initialize LEDs
  LEDS.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, numLEDS);
  LEDS.setBrightness (84);

  for (int i = 0; i < 12; i++) {
    leds[i] = CRGB(255, 255, 255);
    FastLED.show();
  }
  delay (1000);
  FastLED.clear(true);
  FastLED.show();
}

void loop() {
  
/***************************************
 * Radio Code
 ****************************************/
 if (radio.receiveDone())
  {
    //print message received to serial
    //Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
    //Serial.print((char*)radio.DATA);
    //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    dataPack = radio.DATA;
    Serial.print (dataPack[0]);
    //check if received message contains Hello World
    if (strstr((char *)radio.DATA, "%"))
    {
      //check if sender wanted an ACK
      if (radio.ACKRequested())
      {
        radio.sendACK();
        Serial.println(" - ACK sent");
      }
    }  
  }

  radio.receiveDone(); //put radio in RX mode
  
  /*******************************
   * End Radio Code
   *******************************/
   
    checkIncomingByte (1,'A', 'a', 0, 4);
    checkIncomingByte (2,'B', 'b', 4, 6);
    checkIncomingByte (3,'C', 'c', 6, 8);
    checkIncomingByte (4,'D', 'd', 8, 12);

    
  delay (50);
  
  //Serial Communication Code
  
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    Serial.println (incomingByte);
    /*
    checkSerialByte ('A', 'a', 0, 4);
    checkSerialByte ('B', 'b', 4, 6);
    checkSerialByte ('C', 'c', 6, 8);
    checkSerialByte ('D', 'd', 8, 12);
    delay (25);
 */
    
    /*
      if (incomingByte == '%'){
      delay (25);
      int i = 0;
      while (Serial.available() && i < 6){
        incomingButtonValues[i++] = incomingByte;
      }
      incomingButtonValues[i++] = '\0';
      }
      /*
      if (incomingButtonValues[1] = 'A'){
      digitalWrite (13, HIGH);
      } else {digitalWrite (13, LOW);}
    */


    /*
        if (incomingByte == 'B') {
          for (int i = 4; i < 6; i++) {
            leds[i] = CRGB(255, 255, 255);
            FastLED.show();
          }
        } else if (incomingByte == 'b') {
          for (int i = 4; i < 6; i++) {
            leds[i] = CRGB(0, 0, 0);
            FastLED.show();
          }
        }
    */
  }
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}

void checkIncomingByte (char p, char X, char x, int ledStart, int ledEnd) {
  incomingByte = dataPack[p];
  if (incomingByte == X) {
    for (int i = ledStart; i < ledEnd; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (incomingByte == x) {
    for (int i = ledStart; i < ledEnd; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }
}

void checkSerialByte (char X, char x, int ledStart, int ledEnd){
    incomingByte = Serial.read();
  if (incomingByte == X) {
    for (int i = ledStart; i < ledEnd; i++) {
      leds[i] = CRGB(255, 255, 255);
      FastLED.show();
    }
  } else if (incomingByte == x) {
    for (int i = ledStart; i < ledEnd; i++) {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
    }
  }
}

