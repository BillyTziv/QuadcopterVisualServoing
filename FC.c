/***************************************************************
*	Project : Quadcopter
*	Developer : Tzivaras Vasillis 
*
*	computer Science and Engineering Department
*	University of Ioanina
*
*	Contact mail : vtzivaras(at)gmail.com
***************************************************************/

#include <Servo.h>

#define MIN_MV 60	// Minimum motor value after the ARM mode
#define MAX_MV 120	// Maximum motor value after the ARM mode

Servo M1;		// Front left motor
Servo M2;		// Front right motor
Servo M3;		// Back left motor
Servo M4;		// Back right motor

int throttle = 0;

int LFMV;		// Value of the left front motor [M1]
int RFMV;		// Value fo the right Front motor [M2]
int LBMV;		// Value of the left back motor [M3]
int RBMV;		// Value of the Right back motor [M4]

/*
* Inialize the motor pins and write the min value (arm the motors)
*/
void initMotors(void) {
     	M1.attach(9);
     	M2.attach(8);
     	M3.attach(10);
     	M4.attach(11);
}

void arm(void) {
          M1.write(20);
     	M2.write(20);
     	M3.write(20);
     	M4.write(20);
       
     	delay(10000);    // 10 seconds
       
     	Serial.println("Ready...\n");
}

void setup() {
     	Serial.println("Initializing Jarvis");
     	Serial.begin(9600);
     
     	initMotors();
     	arm();
}

/****************************************
	Quadcopter moves
****************************************/
void moveFront() {
	/*
	 * Need to check if the value +-3 is out fo borders
	*/

	// Increase by 3 points the two front motors
	M1.write(LFMV+2);
	M2.write(RFMN+2);

	// Decrease by 3 points the two back motors
	M3.write(LBMV-3);
	M4.write(LBMV-3);
}

void moveLeft() {
	// ...
}

void moveRight() {
	// ...
}

void moveBack() {
	// ...
}

void turnCW() {
	// ...
}

void turnCCW() {
	// ...
}

/***************************************
	Extra functions
***************************************/
void stabilize() {
	// ...
}

void autoLanding() {
	/*
	 * check if the sonar sensor is giving values diff of -1.
	 * If so, low all MV (motor value) in a loop till the 
	 * distance from the ground is ~ 3cm...
	*/
}

/*
 * Operation 1
 * Procees with steady motor value at four motors
 *
 * Status: Completed
*/
void steadyMotor(void) {
	M1.write(MIN_MV);
	M2.write(MIN_MV);
	M3.write(MIN_MV);
	M4.write(MIN_MV);
}

/*
 * Looping some values at four motors
 *
 * Status: Completed
*/
void loopMotor(void) {
	for(throttle = 55; throttle <= 70; throttle+=5){
        	M1.write(throttle);
        	M2.write(throttle);
        	M3.write(throttle);
        	M4.write(throttle);
           
        	Serial.println(throttle);
	}
}

int main(void) {
	steadyMotor();
	// loopMotor();
}
