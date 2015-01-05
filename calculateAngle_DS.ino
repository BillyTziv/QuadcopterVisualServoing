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

#define echoPin1 8 		// Echo Pin 1
#define trigPin1 9 		// Trigger Pin 1

#define echoPin2 6 		// Echo Pin 2
#define trigPin2 7 		// Trigger Pin 2

#define echoPin3 4 		// Echo Pin 3
#define trigPin3 5 		// Trigger Pin 3

#define echoPin4 2 		// Echo Pin 4
#define trigPin4 3 		// Trigger Pin 4

#define ERROR_RANGE 3	// number of error samples

int maximumRange = 200; 	// Maximum range needed
int minimumRange = 0;		// Minimum range needed

long duration1, duration2, duration3, duration4;
long distance1, distance2, distance3, distance4;

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

void printDistance() {
	/* Send the distance to the computer using Serial protocol, and
	turn LED OFF to indicate successful reading. */
	Serial.print(distance1);
	Serial.print(" ");
	Serial.print(distance2);
	Serial.print(" ");
	Serial.print(distance3);
	Serial.print(" ");
	Serial.print(distance4);
	Serial.println();
}

void fixError() {
	// to do...
}

void loop() {
	distance1 = calculateDistance(trigPin1, echoPin1);
	distance2 = calculateDistance(trigPin2, echoPin2);
	distance3 = calculateDistance(trigPin3, echoPin3);
	distance4 = calculateDistance(trigPin4, echoPin4);
        
    fixError();
        
	printDistance();

	//Delay 50ms before next reading.
	delay(20);
}
