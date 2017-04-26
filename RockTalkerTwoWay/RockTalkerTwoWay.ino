/*
  Rock Talker Code
  © Adi Azulay 2017
  RISD Thesis
*/

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <RH_RF69.h>

//LED Variables
#define PIN 5
#define NUM_LEDS 12
#define BRIGHTNESS 50

//IMU Variables
#define BNO055_SAMPLERATE_DELAY_MS (100)

//Radio Variables
#define RF69_FREQ 915.0
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

//Button Pins
#define SEND_PIN 6
#define CIRCLE_PIN 10

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//Debug
#define DEBUG

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);

uint8_t ledGamma[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

int hue;
bool runX;
bool runY;

//Communication Variables
int incomingHue;
int incomingLEDOn;
char outgoingHue [20];
char outgoingLEDOn[20];
char indicator[20];
bool circle;
char hueChar [10];
char ledOnChar [10];

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

#ifdef DEBUG
  Serial.println("Rock Talker is starting up in Debug Mode");
#endif

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  //Radio Settings Comment out if no radio used.
#ifdef DEBUG
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  Serial.println();
#endif




  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

#ifdef DEBUG
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
#endif

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();

  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 255));
    strip.show();
    delay (20);
  }
  delay (200);
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    // Show the leds
    strip.show();
  }
}

void loop() {
  #ifdef DEBUG
  Serial.println (digitalRead (SEND_PIN));
#endif

  if (digitalRead (SEND_PIN) == HIGH) {

    // read the sensor:
    sensors_event_t event;
    bno.getEvent(&event);

    long x = event.orientation.y;
    long y = event.orientation.z;

    float angle = atan2( y, x );                 // angle in radians -180 to 180

    /*
      float magnitude = sqrt( x*x + y*y);    // Pythagoras

      if( magnitude > 512) magnitude = 512;

      x = magnitude * cos( angle );
      y = magnitude * sin( angle );
      angle = atan2( y, x );
    */

    angle = angle * 57296 / 1000; //convert angle from radians to degrees

#ifdef DEBUG
    Serial.print (angle);
    Serial.print ("\t");
    Serial.print (x);
    Serial.print ("\t");
    Serial.println(y);
#endif

    if ( x <= 5 && x >= -5) {
      runX = false;
    } else {
      runX = true;
    }
    if (y <= 5 && y >= -5) {
      runY = false;
    } else {
      runY = true;
    }

    if (runX == true || runY == true) {

      if (digitalRead (CIRCLE_PIN) == HIGH) {
        angle = map(angle, -179, 180, 0, 11);

#ifdef DEBUG
        Serial.println(angle);
#endif

        int outgoingLEDOnInt = int(angle);
        itoa (outgoingLEDOnInt, ledOnChar, 10);
        outgoingLEDOn [0] = 'b';
        for (int i = 1; i < 4; i++) {
          outgoingLEDOn [i] = ledOnChar[i - 1];
        }

        itoa(packetnum++, outgoingLEDOn + 13, 10);

#ifdef DEBUG
        Serial.print("Sending "); Serial.println(outgoingLEDOn);
#endif
        // Send a message!
        rf69.send((uint8_t *)outgoingLEDOn, strlen(outgoingLEDOn));
        rf69.waitPacketSent();

        /*
          itoa(packetnum++, outgoingLEDOn + 13, 10);
          Serial.print("Sending "); Serial.println(outgoingLEDOn);
          // Send a message!
          rf69.send((uint8_t *)outgoingLEDOn, strlen(outgoingLEDOn));
          rf69.waitPacketSent();
        */

      } else {
        int outgoingHueInt;
        if (angle < 0) {
          angle = map(angle, -179, 0, 255, 0);
          outgoingHueInt = int(angle);
        } else if (angle >= 0) {
          angle = map(angle, 0, 180, 0, 255);
          outgoingHueInt = int(angle);
        }

        itoa (outgoingHueInt, hueChar, 10);
        outgoingHue [0] = 'a';
        for (int i = 1; i < 4; i++) {
          outgoingHue [i] = hueChar[i - 1];
        }
        itoa(packetnum++, outgoingHue + 13, 10);
        Serial.print("Sending "); Serial.println(outgoingHue);

        // Send a message!
        rf69.send((uint8_t *)outgoingHue, strlen(outgoingHue));
        rf69.waitPacketSent();

      }
    }




  } else if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
    }

    char newBuf [10];
    for (int i = 1; i < len + 1; i ++) {
      newBuf[i - 1] = char (buf[i]);
    }

#ifdef DEBUG
    Serial.print ("New Buf:");
    Serial.println ((char* )newBuf);
#endif
    if (char (buf[0]) == 'b') {
      incomingLEDOn = atoi ((char*)newBuf);
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
      }
      strip.setPixelColor(incomingLEDOn, strip.Color(0, 0, 0, 255));
      strip.show();
    } else if (char (buf[0]) == 'a') {
      incomingHue = atoi ((char*)newBuf);
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(incomingHue, incomingHue, incomingHue, 0));
        strip.show();
      }
    }
    /*

      if (runX == true || runY == true) {
      if (digitalRead(7) == HIGH) {


      }
      else {
        if (angle < 0){
          angle = map(angle, -179, 0, 255, 0);
          hue = int(angle);
        }
        else if (angle >= 0){
             angle = map(angle, 0, 180, 0, 255);
        hue = int(angle);
        }
      //      angle = map(angle, -179, 180, 0, 126);
      //      int hue2 = int(angle);

        }

      }
      } */
  }
  delay(1);        // delay in between reads for stability
}

