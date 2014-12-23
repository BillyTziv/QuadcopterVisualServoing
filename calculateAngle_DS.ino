

#define echoPin1 7 // Echo Pin
#define trigPin1 8 // Trigger Pin

#define echoPin2 2 // Echo Pin
#define trigPin2 4 // Trigger Pin


int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration1, duration2, distance1, distance2; // Duration used to calculate distance

void setup() {
 Serial.begin (9600);
 pinMode(trigPin1, OUTPUT);
 pinMode(echoPin1, INPUT);
 pinMode(trigPin2, OUTPUT);
 pinMode(echoPin2, INPUT);

}

void loop() {
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin1, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin1, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin1, LOW);
 duration1 = pulseIn(echoPin1, HIGH);
 
 digitalWrite(trigPin2, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin2, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin2, LOW);
 duration2 = pulseIn(echoPin2, HIGH);
  
 //Calculate the distance (in cm) based on the speed of sound.
 distance1 = duration1/58.2;
 distance2 = duration2/58.2;
 
 /* Send the distance to the computer using Serial protocol, and
 turn LED OFF to indicate successful reading. */
 Serial.println(distance1);
 Serial.println(" ");
 Serial.println(distance2);
 Serial.println();


 
 //Delay 50ms before next reading.
 delay(50);
}
