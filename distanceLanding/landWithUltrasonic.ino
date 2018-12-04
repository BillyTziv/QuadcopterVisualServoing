/*
 * This sketch READ, PRINT and FORWARD all the four basic signals
 * from a common RC Receiver. It was tested using Turnigy 9XR Pro
 * transmiter and a FLY sKY FS-RS602 module. I also used an arduino
 * UNO which is connected to the receiver.
 * 
 * The input signals are Throttle, Switch
 * 
 * Contact me at vtzivaras@gmail.com for any issues or suggestions.
 * 
 */

#include <Servo.h>
#include <stdio.h>
#include <Wire.h>  

/*
 * Connect the receiver pins as following.
 * 
 * RECV Throttle Pin -> Digital Pin 4
 * FC Throttle Pin -> Digital Pin 8
 * Switch Pin -> Digital Pin 7
 * Echo Pin -> Digital Pin 10
 * Trig Pin -> Digital Pin 9
 */
 
#define THROTTLE_IN_PIN 4
#define SWITCH_IN_PIN 7
#define echoPin 10
#define trigPin 9

/*
 * The following constants will be used to attach output
 * pins and connect them to the Flight Controller Input pins.
 * They are defined in the setup() function.
 */
Servo THROTTLE_TO_FC;
int switchDuration;
int thrValue = 1000;

int DESATT = 100;

// Input pins
const byte thrInPin = THROTTLE_IN_PIN;
const byte switchInPin = SWITCH_IN_PIN;

int STATE = 0; // 0 for attitude gain 1 for landing
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
int duration, distance; // Duration used to calculate distance

/*
* This method reads RC receiver incoming signals. First 
* it read a PPM pulse and then it uses the map function 
* to change the range (optional).
*/
void readRCInput() {
  //thrDuration = pulseIn(thrInPin, HIGH);
  //thrDuration = map(thrDuration, 1000, 2000, 980, 2020);
  
  switchDuration = pulseIn(switchInPin, HIGH);
  //switchDuration = map(switchDuration, 1000, 2000, 980, 2020);
}

/*
* This method prints all the values of the RC receiver.
*/
void printRCInput() {
  //Serial.print("THROTLE: ");
  //Serial.print(thrDuration);
  //Serial.print("\t");
  Serial.print("SWITCH: ");
  Serial.println(switchDuration);
}

/*
 * This method forward all the values to the flight controller
 */
void writeRCOutput(int value) {
  THROTTLE_TO_FC.writeMicroseconds(value);
}

/*
 * Read the distance
 */
void updateDistance() {
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58.2;
}

/*
 * SETUP function that runs only once.
 */
void setup() {
	Serial.begin(9600);
 
  while (!Serial);
      
  // Define the inputs
	pinMode(thrInPin, INPUT);
	pinMode(switchInPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  THROTTLE_TO_FC.attach(8);

  //Serial.println("Arduino is ready.");
}

/*
* Dont stop the looping!
*/
void loop() {
  // Read any PPM signal from the Receiver
  readRCInput();  // THROTLE and SWITCH

  // Print the PPM signals that you have just read (optional)
  //printRCInput();

  // Read the distance
  updateDistance();
Serial.println(distance);
  // Switch is DOWN [ACTIVATED]
  if(switchDuration > 1600) {
    //Serial.print("Dist: ");
    
    //Serial.print("\t");
    if(STATE == 0) {
      if(DESATT > distance) {
        thrValue = thrValue +1;
        //Serial.print("Gaining attitude\t");
      }else {
        //Serial.print("STOP HERE \t");
        STATE = 1;
      }
    }
    if(STATE == 1) {
      thrValue = thrValue - 0.5*(150-distance);
      //Serial.print("\tLanding\t");
    }
   
    if(thrValue>= 1480) {
      //Serial.print("\tThr: 1480");
      writeRCOutput(1480);
    }else if((thrValue<1410) &&(STATE == 1)) {
      //Serial.print("\tThr: 1410");
      writeRCOutput(1400);
    }else {
      //Serial.print("\tThr: ");
      //Serial.println(thrValue);
      writeRCOutput(thrValue);
    }
    //Serial.println();
  }else {
    /*
     * In this case we write the input value that we have read before.
     */
    writeRCOutput(980);
  }
}
