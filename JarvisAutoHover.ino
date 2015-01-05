/************************************************************************************
* This scetch is calculating the distance from the ground according to
* four HC-SR04 Ultrasonic sensors using an Arduino UNO.
*
* After that it fixes the error (if exists) and returns an average value.
*
* The quadcopter slope is been calculated by a mathematical model according
* to the four sensor values.
*
* Developed by Tzivaras Vasilis (vtzivaras@gmail.com)
*
* Copyright GPL version 2, http://www.gnu.org/licenses/gpl-2.0.html 
*
* Copyright (C) 1989, 1991 Free Software Foundation, Inc.  
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
* Everyone is permitted to copy and distribute verbatim copies
* of this license document, but changing it is not allowed. 
*************************************************************************************/

#include <Servo.h>

#define echoPin2 6 		// Echo Pin
#define trigPin2 7  	//Trigger Pin

#define echoPin1 8 		// Echo Pin
#define trigPin1 9		// Trigger Pin

#define echoPin3 4 		// Echo Pin
#define trigPin3 5 		// Trigger Pin

#define echoPin4 2 		// Echo Pin
#define trigPin4 3 		// Trigger Pin


int maximumRange = 200; 	// Maximum range needed
int minimumRange = 0;	// Minimum range needed

long duration1, duration2, duration3, duration4;
long distance1, distance2, distance3, distance4;

#define MIN_MV 60		// Minimum motor value after the ARM mode
#define MAX_MV 100		// Maximum motor value after the ARM mode

Servo M1;				// Front left motor
Servo M2;				// Front right motor
Servo M3;				// Back right motor
Servo M4;				// Back left motor

int LFMV = MIN_MV;		// Value of the left front motor [M1]
int RFMV = MIN_MV;		// Value fo the right Front motor [M2]
int RBMV = MIN_MV;		// Value of the right back motor [M3]
int LBMV = MIN_MV;		// Value of the left back motor [M4]

/*
* Inialize the motor pins and write the min value (arm the motors)
*/
void initMotors(void) {
     	M1.attach(10);
     	M2.attach(11);
     	M3.attach(12);
     	M4.attach(13);
}

void arm(void) {
  	Serial.println("I am back to life ^_^");
  	M1.write(20);
     	M2.write(20);
     	M3.write(20);
     	M4.write(20);
       
     	delay(10000);    // 10 seconds
       
     	Serial.println("Motors have been armed...\n");
}

void printHeaders() {
  Serial.print("HC-1");

	Serial.print("\t");

	Serial.print("HC-2");

	Serial.print("\t");

	Serial.print("HC-3");

	Serial.print("\t");

	Serial.print("HC-4");

	Serial.print("\t");

	

	Serial.print("M1");

	Serial.print("\t");

	Serial.print("M2");

	Serial.print("\t");

	Serial.print("M4");

	Serial.print("\t");

	Serial.print("M3");

	Serial.print("\t");

	

	Serial.println();  
}

void setup() {
   Serial.begin (9600);
 
	pinMode(trigPin1, OUTPUT);
	pinMode(echoPin1, INPUT);
 
	pinMode(trigPin2, OUTPUT);
	pinMode(echoPin2, INPUT);
 
	pinMode(trigPin3, OUTPUT);
	pinMode(echoPin3, INPUT);
 
	pinMode(trigPin4, OUTPUT);
	pinMode(echoPin4, INPUT);

	initMotors();
   	arm();
     printHeaders();
}

//Calculate the distance (in cm) based on the speed of sound.
int calculateDistance(int trig, int echo) {
	long dur, dist;
		digitalWrite(trig, LOW); 
		delayMicroseconds(2); 

		digitalWrite(trig, HIGH);
		delayMicroseconds(10); 

		digitalWrite(trig, LOW);
		dur = pulseIn(echo, HIGH);
		
		dist = dur/58.2;
	
	return dist;
}

void printStatus() {
	/* Send the distance to the computer using Serial protocol, and

	turn LED OFF to indicate successful reading. */

	

	Serial.print(distance1);

	Serial.print("\t");

	Serial.print(distance2);

	Serial.print("\t");

	Serial.print(distance3);

	Serial.print("\t");

	Serial.print(distance4);

	Serial.print(LFMV);

	Serial.print("\t");

	Serial.print(RFMV);

	Serial.print("\t");

	Serial.print(LBMV);

	Serial.print("\t");

	Serial.print(RBMV);

	Serial.println();
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
	M2.write(RFMV+2);

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
 * Finds the center point (average value) and stabilizes the quadcopter
 * writting more or less throttle to motors. The sensors are 4 points into
 * a line and we can take the average of them. This is the center that they
 * all have to go to remain at the same altitude as much as possible.
 *
 * Control the motors giving more or less throttle to stabilize the quad
*/
void stabilize(int point1, int point2, int point3, int point4) {
	int stabilizationPoint = 0;
		stabilizationPoint = (point1+point2+point3+point4)/4;
		if( stabilizationPoint >= distance1) {
			// Increase the throttle
			LFMV = LFMV+3;
			delay(50);
		}else {
			LFMV = LFMV-3;
			delay(50);
		}
		
		if( stabilizationPoint >= distance2) {
			// Increase the throttle
			RFMV = RFMV+3;
			delay(50);
		}else {
			RFMV = RFMV-3;
			delay(50);
		}
		
		if( stabilizationPoint >= distance3) {
			// Increase the throttle
			LBMV = LBMV+3;
			delay(50);
		}else {
			LBMV = LBMV-3;
			delay(50);
		}
		
		if( stabilizationPoint >= distance4) {
			// Increase the throttle
			RBMV = RBMV+3;
			delay(50);
		}else {
			RBMV = RBMV-3;
			delay(50);
		}
}

void engine() {
    if((LFMV >= 70) || (RFMV >= 70) || (RBMV >= 70) || (LBMV >= 70)) {
      M1.write(MIN_MV);
      M2.write(MIN_MV);
      M3.write(MIN_MV);
      M4.write(MIN_MV);
    }else {
      M1.write(LFMV+1);
      M2.write(RFMV+1);
      M3.write(RBMV+1);
      M4.write(LBMV+1);
    }
  delay(1000);
}

/*
 * Virtual engine monitors the engines and it does not use them at all.
 * Debugging purpose!
*/
void virtualEngine() {
	if((LFMV >= 70) || (RFMV >= 70) || (RBMV >= 70) || (LBMV >= 70)) {
      LFMV = MIN_MV;
      RFMV = MIN_MV;
      RBMV = MIN_MV;
      LBMV = MIN_MV;
    }else {
      LFMV = LFMV+1;
      RFMV = RFMV+1;
      RBMV = RBMV+1;
      LBMV = LBMV+1;
    }
  delay(20);
}

void loop() {
	distance1 = calculateDistance(trigPin1, echoPin1);
	distance2 = calculateDistance(trigPin2, echoPin2);
	distance3 = calculateDistance(trigPin3, echoPin3);
	distance4 = calculateDistance(trigPin4, echoPin4);

	//engine();
        virtualEngine();
	stabilize(distance1, distance2, distance3, distance4);
	
	printStatus();
	//...
     //guard();
	//Delay 50ms before next reading.
	delay(50);
}

