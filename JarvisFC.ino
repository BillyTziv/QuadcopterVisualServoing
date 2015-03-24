/*********************************************************8*
*   This scetch is used to read values from an TX 
*   (Graupner MX-12 is tested) using an arduino (Uno and 
*   Mega tested)
*
*   Developer:: Tzivaras Vasilis
*   Computer science and Engineering
*   University of Ioannina
*   contact me at vtzivaras@gmail.com
***********************************************************/

#include <Servo.h>

// You can change these values if you want according to your
// pins and you TX
#define THROTTLE_PIN  2
#define AILERON_PIN  3
#define ELEVATOR_PIN  4
#define RUDDER_PIN  5

/*
* These values will be passed to the flight controller (as four servo)
*/
Servo THROTTLE_TO_FC;
Servo AILERON_TO_FC;
Servo ELEVATOR_TO_FC;
Servo RUDDER_TO_FC;

int thrVal, ailVal, eleVal, rudVal, oldthrVal, oldailVal, oldeleVal, oldrudVal;
unsigned long scaledThrDur, scaledAilDur, scaledEleDur, scaledRudDur;
unsigned long filteredThr=0, filteredAil=0, filteredEle=0, filteredRud=0;

unsigned long thrDuration;
unsigned long ailDuration;
unsigned long eleDuration;
unsigned long rudDuration;
unsigned long modeDuration;

int thrQueue[5]={0, 0, 0, 0, 0};
int ailQueue[5]={0, 0, 0, 0, 0};
int eleQueue[5]={0, 0, 0, 0, 0};
int rudQueue[5]={0, 0, 0, 0, 0};

const byte thrInPin = THROTTLE_PIN;


const byte ailInPin = AILERON_PIN;


const byte eleInPin = ELEVATOR_PIN;


const byte rudInPin = RUDDER_PIN;


void setup() {
  Serial.begin(9600);
  
  pinMode(thrInPin, INPUT);
  pinMode(ailInPin, INPUT);
  pinMode(eleInPin, INPUT);
  pinMode(rudInPin, INPUT);
  
  THROTTLE_TO_FC.attach(8);
  AILERON_TO_FC.attach(9);  
  ELEVATOR_TO_FC.attach(10);
  RUDDER_TO_FC.attach(11);
  
  arm();
}

void arm() {
   // Arm flight controller
   THROTTLE_TO_FC.writeMicroseconds(3);  // MIN Throttle
   RUDDER_TO_FC.writeMicroseconds(0);  // Full Left Rudder (use 2000 for Full Right Rudder)
   delay(4000); // Wait for the flight controller to recognize the ARM command
   RUDDER_TO_FC.writeMicroseconds(90);  // Return rudder to center position. 
}
  
int filter(int newValue, int mode) {
    int sum=0;
    int mo=0;
    int i;
    int queue[5];
    
    if(mode == 1) {
      for(i=0; i<5; i++) {
         thrQueue[i] = queue[i]; 
      }
    }else if(mode == 2) {
      for(i=0; i<5; i++) {
         ailQueue[i] = queue[i]; 
      }
    }else if(mode == 3) {
      for(i=0; i<5; i++) {
         eleQueue[i] = queue[i]; 
      }
    }else if(mode == 4) {
      for(i=0; i<5; i++) {
         rudQueue[i] = queue[i]; 
      }
    }
      
    if(queue[0] == 0) {
      // there is no element in the array
      // so insert the first element
      queue[0] = newValue;
    }else if(queue[4] != 0) {
      for(i=1; i<5; i++) {
        queue[i-1] = queue[i];
      }
      queue[4] = newValue;
    }else {
      for(i=1; i<5; i++) {
        if(queue[i] == 0) {
           queue[i] = newValue; 
        }
      }
    }
    
    for(i=1; i<5; i++) {
      sum = sum + queue[i];
    }
    mo = sum/5;
    
    if(mode == 1) {
      for(i=0; i<5; i++) {
         queue[i] = thrQueue[i]; 
      }
    }else if(mode == 2) {
      for(i=0; i<5; i++) {
         queue[i] = ailQueue[i]; 
      }
    }else if(mode == 3) {
      for(i=0; i<5; i++) {
         queue[i] = eleQueue[i]; 
      }
    }else if(mode == 4) {
      for(i=0; i<5; i++) {
         queue[i] = rudQueue[i]; 
      }
    }
    
    return mo;
}

void read_from_tx() {
  thrDuration = pulseIn(thrInPin, HIGH);
  scaledThrDur = map(thrDuration, 1400, 2400, 40, 120);
  filteredThr = filter(scaledThrDur, 1);
  
  ailDuration = pulseIn(ailInPin, HIGH);
  scaledAilDur = map(ailDuration, 1400, 2400, 40, 120);
  filteredAil = filter(scaledAilDur, 2);
  
  eleDuration = pulseIn(eleInPin, HIGH);
  scaledEleDur = map(eleDuration, 1400, 2400, 40, 120);
  filteredEle = filter(scaledEleDur, 3);
  
  rudDuration = pulseIn(rudInPin, HIGH);
  scaledRudDur = map(rudDuration, 1400, 2400, 40, 120);
  filteredRud = filter(scaledRudDur, 4);
  
}

void printStatus() {
  Serial.print("Throttle: \t");
  Serial.print(thrDuration);
  Serial.print("  ");
  Serial.print(scaledThrDur);
  Serial.print("  ");
  Serial.print(filteredThr);
  
  Serial.print("\tAileron: \t");
  Serial.print(ailDuration);
  Serial.print("  ");
  Serial.print(scaledAilDur);
  Serial.print("  ");
  Serial.print(filteredAil);
  
  Serial.print("\tElevator: \t");
  Serial.print(eleDuration);
  Serial.print("  ");
  Serial.print(scaledEleDur);
  Serial.print("  ");
  Serial.print(filteredEle);
  
  Serial.print("\tRudder: \t");
  Serial.print(rudDuration);
  Serial.print("  ");
  Serial.print(scaledRudDur);
  Serial.print("  ");
  Serial.print(filteredRud);
  
  Serial.print("\tMode: \t");
  Serial.print(modeDuration);
  
  Serial.print("\n");    
}

void write_to_FC() {
  THROTTLE_TO_FC.write(filteredThr);
  AILERON_TO_FC.write(filteredAil);
  ELEVATOR_TO_FC.write(filteredEle);
  RUDDER_TO_FC.write(filteredRud);
}

void writeDuration() {
  THROTTLE_TO_FC.write(thrDuration);
  AILERON_TO_FC.write(ailDuration);
  ELEVATOR_TO_FC.write(eleDuration);
  RUDDER_TO_FC.write(rudDuration);
}

void writeMappedValues() {
  THROTTLE_TO_FC.write(scaledThrDur);
  AILERON_TO_FC.write(scaledAilDur);
  ELEVATOR_TO_FC.write(scaledEleDur);
  RUDDER_TO_FC.write(scaledRudDur);
}

void loop() {
  read_from_tx();
  //write_to_FC();
  //writeDuration();
  writeMappedValues();
  printStatus();
}




