/*
* Arduino code for calculating distance from the ground using
* distance ultrasonic sensors.
*
* Developer: Vasilis Tzivaras
* Contact me at vtzivaras(at)gmail.com
*/

#define echo 7
#define trig 8

int maximumRange = 200;		// Maximum range needed
int minimumRange = 0;		// Minimum range needed
long duration, distance;	// Duration used to calculate distance

void setup() {
	Serial.begin(9600);

	pinMode(trig, OUTPUT);
	pinMode(echo, OUTPUT);
}

void loop() {
	digitalWrite(trigPin, LOW); 
	delayMicroseconds(2); 

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10); 
	    
	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);
	   
	distance = duration/58.2;
	if (distance >= maximumRange || distance <= minimumRange){
	 	/* Send a negative number to computer and Turn LED ON 
	 	to indicate "out of range" */
		
		Serial.println("Out of range!");
	    digitalWrite(LEDPin, HIGH); 
	}else {
		/* Send the distance to the computer using Serial protocol, and
		 turn LED OFF to indicate successful reading. */
		Serial.print("Distance :: ");
		Serial.println(distance);
		digitalWrite(LEDPin, LOW); 
	}
			    
	//Delay 50ms before next reading.
	delay(50);
}
