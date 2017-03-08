/*
 * Pen Talker
 * Adi Azulay 2017
 */


#define ENCODER_PIN_A 0
#define ENCODER_PIN_B 2
#define POT_PIN A0
#define NUM_READINGS 10
#define SERVO_PIN 9

  
static uint8_t encPrevPos = 0;
static uint8_t encPos = 0;

int readings[NUM_READINGS];
int readIndex = 0;
int average = 0;
int lastAverage;

int potPos;
int lastPotRead;
int total = 0;

//int n = LOW;

void setup() {
  
  pinMode (ENCODER_PIN_A, INPUT);
  pinMode (ENCODER_PIN_B, INPUT);
  digitalWrite (ENCODER_PIN_A, HIGH);
  digitalWrite (ENCODER_PIN_B, HIGH);

  pinMode (SERVO_PIN, OUTPUT);
  
  Serial.begin (9600);

}

void loop() {
  readEncoder (ENCODER_PIN_A, ENCODER_PIN_B);

  int potRead = readPot(POT_PIN);
  if (lastPotRead != potRead);{
    analogWrite (SERVO_PIN, (readPot (POT_PIN)));
  }
  lastPotRead = potRead;
  /*
    for (int i = 0; i < 100; i++){
      potPos = analogRead (POT_PIN);
      totalPos = totalPos + potPos ;
    }
    potPos = totalPos/100;
    totalPos = 0;
  if (lastPot != potPos){
     //Serial.println(potPos);
  }
     lastPot = potPos;
     */
}

void readEncoder (int x, int y){
  int n = digitalRead(x);
  if ((encPrevPos == LOW) && (n == HIGH)){
    if(digitalRead(y) == LOW){
      encPos --;
    } else {
      encPos ++;
    }
    Serial.println(encPos);
  }
  encPrevPos = n;
}

int readPot (int x){
  total = total - readings[readIndex];
  readings[readIndex] = analogRead (x);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= NUM_READINGS){
    readIndex = 0;
  }

  average = total / NUM_READINGS;
  average = map (average, 0, 1023, 0, 180);
  if (lastAverage != average){
    Serial.println(average);
  }
  lastAverage = average;
  return average;
}

