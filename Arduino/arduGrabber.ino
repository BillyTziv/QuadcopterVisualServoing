#include <Servo.h>

#define switchPin 12
#define LEDPin 13

unsigned long switchDur;
Servo grabber;

void setup() {
	Serial.begin(9600);

	pinMode(switchPin, INPUT);
	grabber.attach(11);
}

void loop() {
        switchDur = pulseIn(switchPin, HIGH);
	//thrVal = map(thrDuration, 1400, 2400, 0, 180);
	
       /* if( abs(oldthrVal-thrVal) > 8) {
	  oldthrVal = thrVal;
	}else {
	  thrVal = oldthrVal;
	}*/

        Serial.print("Switch :\t");
        Serial.print(switchDur);
        Serial.print("\n");
        
        if(switchDur > 1500) {
          // Open the grabber
          Serial.print("OPEN\t");
          grabber.write(150);
        }else {
          // close the grabber
          Serial.print("CLOSE\t");
          grabber.write(30);
        } 
}
