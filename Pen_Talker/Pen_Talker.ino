/*
 * Pen Talker
 * Adi Azulay 2017
 */

#include <Servo.h>

//Uncomment this code to activate debugging mode
#define DEBUG

//Pins for all componenets
#define ENCODER_PIN_A 0
#define ENCODER_PIN_B 2
#define POT_PIN A0
#define NUM_READINGS 10
#define SERVO_PIN 9
#define SERVO_2_PIN 6

//Check frequency for buildling coordinets
const static int checkFrequency = 250;

Servo rotationServo;
Servo lengthServo;

static uint8_t encPrevPos = 0;
volatile unsigned uint8_t encPos = 0;

//Encoder Variables
int readings[NUM_READINGS];
int readIndex = 0;
int average = 0;
int lastAverage;

//Potentiometer Variables
int potPos;
int lastPotRead;
int potTotal = 0;

long timer;
long lastTimer = 0;

long readNumber = 0;

int x[60];
int y[60];

void setup() {

  pinMode (ENCODER_PIN_A, INPUT);
  pinMode (ENCODER_PIN_B, INPUT);
  digitalWrite (ENCODER_PIN_A, HIGH);
  digitalWrite (ENCODER_PIN_B, HIGH);

  //Encoder interrupt
  attachInterrupt(0, doEncoder, CHANGE)

  rotationServo.attach (SERVO_PIN);
  lengthServo.attach (SERVO_2_PIN);

  Serial.begin (9600);

}

void loop() {
  timer = millis();
  int timeCheck = timer - lastTimer;

  if (timeCheck > checkFrequency){
    x[readNumber] = encPos;
    y[readNumber] = potPos;
    #ifdef DEBUG
      Serial.print("x =");
      Serial.print(x[readNumber]);
      Serial.print("\t");
      Serial.print("y =");
      Serial.println(y[readNumber]);
    #endif
    readNumber++;
    timer = lastTimer;
  }



  lengthServo.write (readEncoder (ENCODER_PIN_A, ENCODER_PIN_B));

  potPos = readPot (POT_PIN);
  if (lastPotRead != potPos){
   rotationServo.write (potPos);
  }
  lastPotRead = potPos;
/*
  Serial.print(lastPotRead);
  Serial.print("\t");
  Serial.println(potPos);
  */
  /*
    for (int i = 0; i < 100; i++){
      potPos = analogRead (POT_PIN);
      potTotalPos = potTotalPos + potPos ;
    }
    potPos = potTotalPos/100;
    potTotalPos = 0;
  if (lastPot != potPos){
     //Serial.println(potPos);
  }
     lastPot = potPos;
     */
     if (readNumber >= 59){
       readNumber = 0;
     }
}



int readEncoder (int x, int y){
  int n = digitalRead(x);
  if ((encPrevPos == LOW) && (n == HIGH)){
    if(digitalRead(y) == LOW){
      encPos --;
    } else {
      encPos ++;
    }
    //Serial.println(encPos);
  }
  encPrevPos = n;
  return encPos;
}

int readPot (int x){
  potTotal = potTotal - readings[readIndex];
  readings[readIndex] = analogRead (x);
  potTotal = potTotal + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= NUM_READINGS){
    readIndex = 0;
  }

  average = potTotal / NUM_READINGS;
  average = map (average, 0, 1023, 0, 180);
  if (lastAverage != average){
    // Serial.println(average);
  }
  lastAverage = average;
  return average;
}
