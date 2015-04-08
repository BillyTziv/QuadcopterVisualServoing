/***************************************************
*
*
*
*
*
***************************************************/

#include <Servo.h> 

#define THROTTLE_FROM_TX 2
#define AILERON_FROM_TX 3
#define ELEVATOR_FROM_TX 6
#define RUDDER_FROM_TX 7

/*
* These values will be passed to the flight controller (as four servo)
*/
Servo THROTTLE_TO_FC;
Servo AILERON_TO_FC;
Servo ELEVATOR_TO_FC;
Servo RUDDER_TO_FC;

int thrVal, ailVal, eleVal, rudVal, oldthrVal, oldailVal, oldeleVal, oldrudVal;
unsigned long thrDuration, ailDuration, eleDuration, rudDuration;

/*
* Setup function runs only once and initializes whatever needs an initialization
*/
void setup() {
  Serial.begin(9600);

  pinMode(THROTTLE_FROM_TX, INPUT);
  pinMode(AILERON_FROM_TX, INPUT);
  pinMode(ELEVATOR_FROM_TX, INPUT);
  pinMode(RUDDER_FROM_TX, INPUT);

  THROTTLE_TO_FC.attach(5);
  AILERON_TO_FC.attach(7);  
  ELEVATOR_TO_FC.attach(6);
  RUDDER_TO_FC.attach(4);
  
  //arm();
}

/*
 * Automatically arms the ECS and the motors within 3 seconds
 */
void arm() {
  Serial.println("Arming the motors...");
  THROTTLE_TO_FC.write(0);
  RUDDER_TO_FC.write(0);
  delay(4000);
  RUDDER_TO_FC.write(90);
}

void readFromRX() {
  /*
  * Theottle
  */
  thrDuration = pulseIn(THROTTLE_FROM_TX, HIGH);
  thrVal = map(thrDuration, 1400, 2400, 0, 180);
  Serial.print("Throttle\t");
  Serial.print(thrDuration);
  
  /*if( abs(oldthrVal-thrVal) > 8) {
    oldthrVal = thrVal;
  }else {
    thrVal = oldthrVal;
  }*/
  
  /*
  * Aileron
  */
  ailDuration = pulseIn(AILERON_FROM_TX, HIGH);
  ailVal = map(ailDuration, 1400, 2400, 0, 180);
  Serial.print(" Aileron\t");
  Serial.print(ailDuration);
  /*
  if( abs(oldailVal-ailVal) > 8) {
    oldailVal = ailVal;
  }else {
    ailVal = oldailVal;
  }*/
  
  /*
  * Elevator
  */
  eleDuration = pulseIn(ELEVATOR_FROM_TX, HIGH);
  eleVal = map(eleDuration, 1400, 2400, 0, 180);
  Serial.print(" Elevator\t");
  Serial.print(eleDuration);
  
  /*
  if( abs(oldeleVal-eleVal) > 8) {
    oldeleVal = eleVal;
  }else {
    eleVal = oldeleVal;
  }*/
  
  /*
  * Rudder
  */
  rudDuration = pulseIn(RUDDER_FROM_TX, HIGH);
  rudVal = map(rudDuration, 1400, 2400, 0, 180);
  Serial.print(" Rudder\t");
  Serial.print(rudDuration);
  
  /*
  if( abs(oldrudVal-rudVal) > 8) {
    oldrudVal = rudVal;
  }else {
    rudVal = oldrudVal;
  }*/
  
  Serial.println();
}

void writeToFC() {
  THROTTLE_TO_FC.write(thrVal);
  AILERON_TO_FC.write(ailVal);
  ELEVATOR_TO_FC.write(eleVal);
  RUDDER_TO_FC.write(rudVal);
}

void printStatus() {
   
}

void loop() {
  readFromRX();
  
  //writeToFC();
  
  //printStatus();
  
  delay(15);
}
