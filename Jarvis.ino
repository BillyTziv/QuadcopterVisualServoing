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

/***************************************************************************
* ULTRASONIC DISTANCE SENSOR VARIALBES ARE DEFINED HERE
***************************************************************************/
#define ECHO1   8 			// HC-SR04-1 Echo Pin attached on pin 8
#define TRIG1   9			// HC-SR04-1 Trig Pin attached on pin 9

#define ECHO2   6 			// HC-SR04-2 Echo Pin attached on pin 6
#define TRIG2   7  			// HC-SR04-2 Trig Pin attached on pin 7

#define ECHO3	4 			// HC-SR04-3 Echo Pin attached on pin 4
#define TRIG3	5 			// HC-SR04-3 Trig Pin attached on pin 5

#define ECHO4	2 			// HC-SR04-4 Echo Pin attached on pin 2
#define TRIG4	3 			// HC-SR04-4 Trig Pin attached on pin 3

/***************************************************************************
* MOTOR VARIALBES ARE DEFINED HERE
***************************************************************************/
#define MOTOR1	10			// M1 attached on pin 10
#define MOTOR2	11			// M2 attached on pin 11
#define MOTOR3	12			// M3 attached on pin 12
#define MOTOR4	13			// M4 attached on pin 13

#define ARMV 	20			// Value that arms motors (<60)
#define MIN_MV 	60			// Minimum motor value after the ARM mode
#define MAX_MV 	80			// Maximum motor value after the ARM mode

/***************************************************************************
* GLOBAL VARIALBES
***************************************************************************/
int DSMaxRange = 200; 			// Maximum range needed
int DSMinRange = 3;			// Minimum range needed

int DS1Dur, DS2Dur, DS3Dur, DS4Dur;	// Four durations from the HC SR04 Ultrasonic sensors (return value)
int DS1Val, DS2Val, DS3Val, DS4Val;	// Four distances calculated according to the durations

int stabilizationPoint = 0;		// An average distance from the ground of the 4 Ultrasonic sensors

//int distBuf1[5];			// Distance buffer 1
//int distBuf2[5];			// Distance buffer 2
//int distBuf3[5];			// Distance buffer 3
//int distBuf4[5];			// Distance buffer 4

//int numOfBufElem1 = 0;		// Number of elements at buffer1
//int numOfBufElem2 = 0;		// Number of elements at buffer2
//int numOfBufElem3 = 0;		// Number of elements at buffer3
//int numOfBufElem4 = 0;		// Number of elements at buffer4

Servo M1;				// Front left motor
Servo M2;				// Front right motor
Servo M3;				// Back right motor
Servo M4;				// Back left motor

int LFMV = MIN_MV;			// Value of the left front motor [M1]
int RFMV = MIN_MV;			// Value fo the right front motor [M2]
int RBMV = MIN_MV;			// Value of the right back motor [M3]
int LBMV = MIN_MV;			// Value of the left back motor [M4]

/***************************************************************************
* FUNCTIONS
***************************************************************************/

/*
* Prints the project Logo name
*/
void printLogo() {
	Serial.print("\n\n");
	Serial.print("\tJJJJJJJJJJJ   AAAAA        RRRRRRRRRRR       VVVV          VVVV    IIIIIIIIII       SSSSSSSSSS\n");          
	Serial.print("\tJJJJJJJJJJJ  AAAAAAA       RRRRRRRRRRRRR     VVVV          VVVV    IIIIIIIIII      SSSSSSSSSSS\n");               
	Serial.print("\t    JJJJ     AAA AAA       RRRA      RRR     VVVV          VVVV       IIII         SSSS       \n");  
	Serial.print("\t    JJJJ    AAA   AAA      RRRR      RRR     VVVV          VVVV       IIII          SSSS      \n");  
	Serial.print("\t    JJJJ    AAA   AAA      RRRRRRRRRRRR      VVVV          VVVV       IIII            SSSS    \n");  
	Serial.print("\t    JJJJ    AAA   AAA      RRRRRRRRRRRRR      VVVV        VVVV        IIII             SSSSSS \n");       
	Serial.print("\t    JJJJ   AAAAAAAAAAA     RRRRR     RRRR      VVVV      VVVV         IIII               SSSSS\n");            
	Serial.print("\t   JJJJJ   AAAAAAAAAAA     RRRR      RRRR       VVVV    VVVV          IIII               SSSSS\n");        
	Serial.print("\tJJJJJJ    AAAA     AAAA    RRRR      RRRR        VVVVVVVVVV        IIIIIIIIII      SSSSSSSSSSS\n");                 
	Serial.print("\tJJJJJ     AAAA     AAAA    RRRR      RRRR         VVVVVVVV         IIIIIIIIII     SSSSSSSSSS  \n");                
	Serial.print("\n");
}

/*
* Attack the four motors to arduino pins
*/
void attachMotors(void) {
	M1.attach(MOTOR1);
	M2.attach(MOTOR2);
	M3.attach(MOTOR3);
	M4.attach(MOTOR4);
}

/*
* Arms the four motors with very low values
*/
void armMotors(void) {
  	Serial.println("Arming...");

  	M1.write(ARMV);
	M2.write(ARMV);
	M3.write(ARMV);
	M4.write(ARMV);

	delay(10000);    // 10 seconds
	Serial.println("Motors have been armed...\n");
}

void disarmMotors(void) {
	Serial.println("Disarming...");
	
	M1.write(ARMV);
	M2.write(ARMV);
	M3.write(ARMV);
	M4.write(ARMV);
	
	delay(1000);
	Serial.println("Motors have been disarmed...\n");
}
void printHeaders() {
	Serial.print("HC-1\t");
	Serial.print("HC-2\t");
	Serial.print("HC-3\t");
	Serial.print("HC-4\t");
	
	Serial.print("M1\t");
	Serial.print("M2\t");
	Serial.print("M4\t");
	Serial.print("M3\t");
	Serial.print("SP\t");
	
	Serial.println();  
}
void setup() {
	Serial.begin (9600);
 	printLogo();
	pinMode(TRIG1, OUTPUT);
	pinMode(ECHO1, INPUT);
 
	pinMode(TRIG2, OUTPUT);
	pinMode(ECHO2, INPUT);
 
	pinMode(TRIG3, OUTPUT);
	pinMode(ECHO3, INPUT);
 
	pinMode(TRIG4, OUTPUT);
	pinMode(ECHO4, INPUT);
	attachMotors();
   	armMotors();
     
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
	Serial.print(DS1Val);
	Serial.print("\t");
	Serial.print(DS2Val);
	Serial.print("\t");
	Serial.print(DS3Val);
	Serial.print("\t");
	Serial.print(DS4Val);
	
	Serial.print("\t");
	Serial.print(LFMV);
	Serial.print("\t");
	Serial.print(RFMV);
	Serial.print("\t");
	Serial.print(LBMV);
	Serial.print("\t");
	Serial.print(RBMV);
	Serial.print("\t");
	Serial.print(stabilizationPoint);
	
	Serial.println();
}
/*
* check if the sonar sensor is giving values diff of -1.
* If so, low all MV (motor value) in a loop till the 
* distance from the ground is ~ 3cm...
*/
void autoLanding() {
	
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
	int step = 1;
	int delayTime = 20;
	
		stabilizationPoint = (point1+point2+point3+point4)/4;
		if( stabilizationPoint >= DS1Val) {
			// Increase the throttle
			LFMV = LFMV+step;
			delay(delayTime);
		}else {
			LFMV = LFMV-step;
			delay(delayTime);
		}
		
		if( stabilizationPoint >= DS2Val) {
			// Increase the throttle
			RFMV = RFMV+step;
			delay(delayTime);
		}else {
			RFMV = RFMV-step;
			delay(delayTime);
		}
		
		if( stabilizationPoint >= DS3Val) {
			// Increase the throttle
			LBMV = LBMV+step;
			delay(delayTime);
		}else {
			LBMV = LBMV-step;
			delay(delayTime);
		}
		
		if( stabilizationPoint >= DS4Val) {
			// Increase the throttle
			RBMV = RBMV+step;
			delay(50);
		}else {
			RBMV = RBMV-step;
			delay(50);
		}
}
void engine() {
    if(stabilizationPoint < 30) {
      M1.write(LFMV+1);
      M2.write(RFMV+1);
      M3.write(RBMV+1);
      M4.write(LBMV+1);
     }else {
      M1.write(LFMV-1);
      M2.write(RFMV-1);
      M3.write(RBMV-1);
      M4.write(LBMV-1);
    }
}
/*
 * Virtual engine monitors the engines and it does not use them at all.
 * Debugging purpose!
*/
void virtualEngine() {
    if(stabilizationPoint < 30) {
      LFMV = LFMV+1;
      RFMV = RFMV+1;
      RBMV = RBMV+1;
      LBMV = LBMV+1;
    }else {
      LFMV = LFMV-1;
      RFMV = RFMV-1;
      RBMV = RBMV-1;
      LBMV = LBMV-1;
    }
  delay(20);
}
/*
* If the buffer has less than 5 elements just insert the new element
* and return the new element even if there is an error
*/
/*
int fixError(int d1) {
	if(numOfElem >= 5) {
		for(int i=0; i<4 i++) {
			distBuf1[i] = distBuf1[i+1];
		}
		distBuf1[4] = d1;
	}else {
		distBuf1[numOfBufElem1] = d1;
		numOfBufElem1++;
		return d1;
	}
	
	for(int i=0; i<5; i++) {
		errorPos[5]; //
		errorVal = distBuf1[i];
		
		for(int j=0; j<5; j++) {
			if(distBuf1[i]-distBuf1[j] > 2) {
				errorPos[i] ++;
			}
		}
	}
	
	
	  Find the index with the maximum error possibility. this value obviously
	 is a noise at the average.
	
	maxErrorPosIndex = 0;
	for(int i=0; i<4; i++) {
		if(errorPos[i]>errorPos[i+1]) {
			maxErrorPosIndex = i;
		}
	}
	
}*/
/****************************************
	Quadcopter moves
****************************************/
void moveFront() {
	// ...
	
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
/*
* This function calculates the four distances and filter the results so the 
* value will be from 3 to 200 cm.
*/
void calculateDistance() {
	DS1Dur = calculateDistance(TRIG1, ECHO1);
	DS1Dur = constrain(DS1Dur, 3, 200);

	DS2Dur = calculateDistance(TRIG2, ECHO2);	
	DS2Dur = constrain(DS2Dur, 3, 200);
		
	DS3Dur = calculateDistance(TRIG3, ECHO3);
	DS3Dur = constrain(DS3Dur, 3, 200);
		
	DS4Dur = calculateDistance(TRIG4, ECHO4);
	DS4Dur = constrain(DS4Dur, 3, 200);
}

void steadySpeed(int speedx) {
  
      M1.write(speedx);
      M2.write(speedx);
      M3.write(speedx);
      M4.write(speedx);
}

void controlEngine(int incomingValue) {
      if( incomingValue >) { 
        LFMV = incomingValue;
        RFMV = incomingValue;
        RBMV = incomingValue;
        LBMV = incomingValue;
      }
}


void loop() {
        char buffer[] = {' ',' ',' '};   // Receive up to 3 bytes
        //while (!Serial.available());                     // Wait for characters
        Serial.readBytesUntil('n', buffer, 3);
        if() {
          LFMV = atoi(buffer);
          RFMV = atoi(buffer);
          RBMV = atoi(buffer);
          LBMV = atoi(buffer);
        }
        
        //Serial.println(incomingValue);
        controlEngine(incomingValue);
        
  	// Calculates the 4 distance from HC-SR04 sensors
	calculateDistance();	
		
	// Stabilizes the quad according to the distance
	stabilize(DS1Dur, DS2Dur, DS3Dur, DS4Dur);			
		
	engine();
	//virtualEngine();
        //steadySpeed(95);
	printStatus();
     
	//Delay 50ms before next reading.
	delay(50);
}