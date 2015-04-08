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

// You can change these values if you want according to your
// pins and you TX
#define THROTTLE_PIN  9
#define AILERON_PIN  10
#define ELEVATOR_PIN  11
#define RUDDER_PIN  12

unsigned long thrDuration;
unsigned long ailDuration;
unsigned long eleDuration;
unsigned long rudDuration;
unsigned long switchDuration;

const byte thrInPin = THROTTLE_PIN;
const byte thrOutPin = 6;

const byte ailInPin = AILERON_PIN;
const byte ailOutPin = 6;

const byte eleInPin = ELEVATOR_PIN;
const byte eleOutPin = 6;

const byte rudInPin = RUDDER_PIN;
const byte rudOutPin = 6;

const byte switchInPin = 8;
const byte switchOutPin = 2;

//int autolandPin = 8;
//int val = 0;

void setup() {
	Serial.begin(9600);

        pinMode(thrInPin, INPUT);
        pinMode(ailInPin, INPUT);
        pinMode(eleInPin, INPUT);
        pinMode(rudInPin, INPUT);
        pinMode(switchInPin, INPUT);
        
        
        pinMode(thrOutPin, OUTPUT);
        pinMode(ailOutPin, OUTPUT);
        pinMode(eleOutPin, OUTPUT);
        pinMode(rudOutPin, OUTPUT);
}

void printStatus() {
  thrDuration = pulseIn(thrInPin, HIGH);
  thrDuration = map(thrDuration, 1000, 2000, 980, 2020);
  Serial.print("Throttle: \t");
  Serial.print(thrDuration);
  
  ailDuration = pulseIn(ailInPin, HIGH);
  ailDuration = map(ailDuration, 1000, 2000, 980, 2020);
  Serial.print("\tAileron: \t");
  Serial.print(ailDuration);
  
  eleDuration = pulseIn(eleInPin, HIGH);
  eleDuration = map(eleDuration, 1000, 2000, 980, 2020);
  Serial.print("\tElevator: \t");
  Serial.print(eleDuration);
  
  rudDuration = pulseIn(rudInPin, HIGH);
  rudDuration = map(rudDuration, 1000, 2000, 980, 2020);
  Serial.print("\tRudder: \t");
  Serial.print(rudDuration);
  
  printSwitchState();
  
  Serial.print("\n");    
}

void printSwitchState() {
  switchDuration = pulseIn(switchInPin, HIGH);
  Serial.print("\tSwitch: \t");
  
  if( switchDuration < 2000 ) {
    Serial.print("ON");
    // code when switch is activated (top)
  }else {
    Serial.print("OFF");
    // code when switch is deactivated (bot)
  }
  //Serial.print(switchDuration);
}

/*
 * Main function
*/void loop() {
  printStatus();
}





