#include <Servo.h>

Servo M1;
Servo M2;
Servo M3;
Servo M4;

int throttle = 0;

/*
* Inialize the motor pins and write the min value (arm the motors)
*/
void initMotors() {

}

void setup() {
    Serial.println("Initializing Jarvis");
    Serial.begin(9600);
   
    M1.attach(9);
    M2.attach(8);
    M3.attach(10);
    M4.attach(11);
   
    M1.write(20);
    M2.write(20);
    M3.write(20);
    M4.write(20);
       
        delay(8000);    // 3 seconds
       
    Serial.println("I am online sir...");
}

/****************************************
	Quadcopter moves
****************************************/
void moveFront() {

}

void moveLeft() {

}

void moveRight() {

}

void moveBack() {

}

void turnCW() {

}

void turnCCW() {

}

/***************************************
	Extra functions
***************************************/
void stabilize() {

}

void autoLanding() {

}

void loop() {
	/*for(throttle=55; throttle<= 70; throttle+=5){
            M1.write(throttle);
            M2.write(throttle);
            M3.write(throttle);
            M4.write(throttle);
           
            Serial.println(throttle);
	}*/
    throttle = 60;   
    M1.write(throttle);
    M2.write(throttle);
    M3.write(throttle);
    M4.write(throttle);
}
