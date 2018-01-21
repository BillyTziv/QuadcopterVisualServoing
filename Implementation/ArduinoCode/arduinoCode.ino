/*
 * This sketch READ, PRINT and FORWARD all the four basic signals
 * from a common RC Receiver. It was tested using Turnigy 9XR Pro
 * transmiter and a FLY sKY FS-RS602 module. I also used an arduino
 * UNO which is connected to the receiver.
 * 
 * Contact me at vtzivaras@gmail.com for any issues or suggestions.
 * 
 * 
 */

#include <Servo.h>

#define THROTTLE_IN_PIN 3
#define AILERON_IN_PIN 4
#define ELEVATOR_IN_PIN 5
#define RUDDER_IN_PIN 6

/*#define THROTTLE_OUT_PIN 8
#define AILERON_OUT_PIN 9
#define ELEVATOR_OUT_PIN 10
#define RUDDER_OUT_PIN 11
*/

Servo THROTTLE_TO_FC;
Servo AILERON_TO_FC;
Servo ELEVATOR_TO_FC;
Servo RUDDER_TO_FC;

unsigned long thrDuration;
unsigned long ailDuration;
unsigned long eleDuration;
unsigned long rudDuration;

// Input pins
const byte thrInPin = THROTTLE_IN_PIN;
const byte ailInPin = AILERON_IN_PIN;
const byte eleInPin = ELEVATOR_IN_PIN;
const byte rudInPin = RUDDER_IN_PIN;

// Output pins
/*const byte thrOutPin = THROTTLE_OUT_PIN;
const byte ailOutPin = AILERON_OUT_PIN;
const byte eleOutPin = ELEVATOR_OUT_PIN;
const byte rudOutPin = RUDDER_OUT_PIN;
*/
void setup() {
  // Set the baud rate to 115200
	Serial.begin(115200);

  // Define the in pinouts
	pinMode(thrInPin, INPUT);
	pinMode(ailInPin, INPUT);
	pinMode(eleInPin, INPUT);
	pinMode(rudInPin, INPUT);

  THROTTLE_TO_FC.attach(8);
  AILERON_TO_FC.attach(9);
  ELEVATOR_TO_FC.attach(10);
  RUDDER_TO_FC.attach(11);
  
  // Define the out pinouts
  /*pinMode(thrOutPin, OUTPUT);
  pinMode(ailOutPin, OUTPUT);
  pinMode(eleOutPin, OUTPUT);
  pinMode(rudOutPin, OUTPUT);*/
}

/*
* This method prints all the values of the RC receiver.
*/
void printRCInput() {
	Serial.print(thrDuration);
	Serial.print("\t");
	Serial.print(ailDuration);
	Serial.print("\t");
	Serial.print(eleDuration);
	Serial.print("\t");
	Serial.println(rudDuration);
}

/*
* This method reads RC receiver incoming signals. First 
* it read a PWM pulse and then it uses the map function 
* to change the range (optional).
*/
void readRCInput() {
	thrDuration = pulseIn(thrInPin, HIGH);
	//thrDuration = map(thrDuration, 1000, 2000, 980, 2020);

	ailDuration = pulseIn(ailInPin, HIGH);
	//ailDuration = map(ailDuration, 1000, 2000, 980, 2020);

	eleDuration = pulseIn(eleInPin, HIGH);
	//eleDuration = map(eleDuration, 1000, 2000, 980, 2020);

	rudDuration = pulseIn(rudInPin, HIGH);
	//rudDuration = map(rudDuration, 1000, 2000, 980, 2020);
}

/*
 * This method forward all the values to the flight controller
 */
void writeRCOutput() {
	THROTTLE_TO_FC.writeMicroseconds(thrDuration);
	AILERON_TO_FC.writeMicroseconds(ailDuration);
	ELEVATOR_TO_FC.writeMicroseconds(eleDuration);
	RUDDER_TO_FC.writeMicroseconds(rudDuration);
}

/*
* LOOP forever and ever
*/
void loop() {
  // First read the receiver input signals
	readRCInput();

  // Print those signals (optional)
	//printRCInput();

  // Forward signals to the next component (eg. a quadcopter
  // flight controller or any other device similar to that)
	writeRCOutput();
}
