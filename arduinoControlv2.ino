/***************************************************
* This sketch uses an arduino mega to read values
* from an RX then passes them to the FC
***************************************************/

#include <Servo.h> 

#define thrInPin 9
#define ailInPin 10
#define eleInPin 11
#define rudInPin 12

unsigned long thrDuration;
unsigned long ailDuration;
unsigned long eleDuration;
unsigned long rudDuration;

Servo thrOutPin;
Servo ailOutPin;
Servo eleOutPin;
Servo rudOutPin;

int thrVal, ailVal, eleVal, rudVal;
int oldthrVal, oldailVal, oldeleVal, oldrudVal;

void setup() {
  Serial.begin(9600);

  pinMode(thrInPin, INPUT);
  pinMode(ailInPin, INPUT);
  pinMode(eleInPin, INPUT);
  pinMode(rudInPin, INPUT);

  thrOutPin.attach(6);
  ailOutPin.attach(4);  
  eleOutPin.attach(5);
  rudOutPin.attach(7);
}

void readVal() {
  thrDuration = pulseIn(thrInPin, HIGH);
  thrVal = map(thrDuration, 1400, 2400, 0, 180);
  Serial.print("\t>> \t");
  Serial.println(thrVal);
  
  if( abs(oldthrVal-thrVal) > 8) {
    oldthrVal = thrVal;
  }else {
    thrVal = oldthrVal;
  }
  
  ailDuration = pulseIn(ailInPin, HIGH);
  ailVal = map(ailDuration, 1400, 2400, 0, 180);
  Serial.print("\t>> \t");
  Serial.println(ailVal);
  
  if( abs(oldailVal-ailVal) > 8) {
    oldailVal = ailVal;
  }else {
    ailVal = oldailVal;
  }
  
  eleDuration = pulseIn(eleInPin, HIGH);
  eleVal = map(eleDuration, 1400, 2400, 0, 180);
  Serial.print("\t>> \t");
  Serial.println(eleVal);
  
  if( abs(oldeleVal-eleVal) > 8) {
    oldeleVal = eleVal;
  }else {
    eleVal = oldeleVal;
  }
  
  rudDuration = pulseIn(rudInPin, HIGH);
  rudVal = map(rudDuration, 1400, 2400, 0, 180);
  Serial.print("\t>> \t");
  Serial.println(rudVal);
  
  if( abs(oldrudVal-rudVal) > 8) {
    oldrudVal = rudVal;
  }else {
    rudVal = oldrudVal;
  }
}

void writeVal() {
  thrOutPin.write(thrVal);
  ailOutPin.write(ailVal);
  eleOutPin.write(eleVal);
  rudOutPin.write(rudVal);
}

void loop() {
  readVal();
  writeVal();
  delay(15);
}
