/*
 * This sketch READ, PRINT and FORWARD all the four basic signals
 * from a common RC Receiver. It was tested using Turnigy 9XR Pro
 * transmiter and a FLY sKY FS-RS602 module. I also used an arduino
 * UNO which is connected to the receiver.
 * 
 * Contact me at vtzivaras@gmail.com for any issues or suggestions.
 * 
 */

#include <Servo.h>
#include <stdio.h>

#define THROTTLE_IN_PIN 3
#define AILERON_IN_PIN 4
#define ELEVATOR_IN_PIN 5
#define RUDDER_IN_PIN 6

Servo THROTTLE_TO_FC;
Servo AILERON_TO_FC;
Servo ELEVATOR_TO_FC;
Servo RUDDER_TO_FC;

unsigned long thrDuration;
unsigned long ailDuration;
unsigned long eleDuration;
unsigned long rudDuration;

// Input pins
const byte thrInPin = THROTTLE_IN_PIN;
const byte ailInPin = AILERON_IN_PIN;
const byte eleInPin = ELEVATOR_IN_PIN;
const byte rudInPin = RUDDER_IN_PIN;

boolean PARSE = false;
boolean EXEC = false;    

char myStr[64];
int bufPos = 0;

unsigned long lastMessage;

void clearBuffer() {
  for(int j=0; j<64; j++) {
    myStr[j] = '\0';
  }
  bufPos = 0;
}
void processInput () {
  while (Serial.available ()) {
    char c = Serial.read ();
    
    if( c == '<') {
      PARSE = true;
    }else if( c == '>') {
      PARSE = false;
      EXEC = true;
    }
    
    if( ( PARSE == true ) && ( c != '<' ) ) {
      myStr[bufPos] = c;
      bufPos++;
    }
  }  // end of while loop
}  // end of processInput

/*
* This method reads RC receiver incoming signals. First 
* it read a PPM pulse and then it uses the map function 
* to change the range (optional).
*/
void readRCInput() {
  thrDuration = pulseIn(thrInPin, HIGH);
  //thrDuration = map(thrDuration, 1000, 2000, 980, 2020);

  ailDuration = pulseIn(ailInPin, HIGH);
  //ailDuration = map(ailDuration, 1000, 2000, 980, 2020);

  eleDuration = pulseIn(eleInPin, HIGH);
  //eleDuration = map(eleDuration, 1000, 2000, 980, 2020);

  rudDuration = pulseIn(rudInPin, HIGH);
  //rudDuration = map(rudDuration, 1000, 2000, 980, 2020);
}

/*
* This method prints all the values of the RC receiver.
*/
void printRCInput() {
  Serial.print(thrDuration);
  Serial.print("\t");
  Serial.print(ailDuration);
  Serial.print("\t");
  Serial.print(eleDuration);
  Serial.print("\t");
  Serial.println(rudDuration);
}

/*
 * This method forward all the values to the flight controller
 */
void writeRCOutput(int thrD, int ailD, int eleD, int rudD) {
  THROTTLE_TO_FC.writeMicroseconds(thrD);
  AILERON_TO_FC.writeMicroseconds(ailD);
  ELEVATOR_TO_FC.writeMicroseconds(eleD);
  RUDDER_TO_FC.writeMicroseconds(rudD);
}

void setup() {
  // Set the baud rate to 115200
	Serial.begin(9600);
  while (!Serial);
  
  // Define the in pinouts
	pinMode(thrInPin, INPUT);
	pinMode(ailInPin, INPUT);
	pinMode(eleInPin, INPUT);
	pinMode(rudInPin, INPUT);

  THROTTLE_TO_FC.attach(8);
  AILERON_TO_FC.attach(9);
  ELEVATOR_TO_FC.attach(10);
  RUDDER_TO_FC.attach(11);

  // Arm the quadrotor by turning the rudder to the max
  // value for about 3 seconds (max value=1971)
  //Serial.print("Arming the quadrotor...");
  writeRCOutput(965, 1464, 1464, 1971);
  delay(3000);
  writeRCOutput(1080, 1464, 1464, 1464);
  //Serial.println("OK");
}

/*
* Dont stop the looping!
*/
void loop() {
  // Read any PPM signal coming from the Receiver
	readRCInput();
    
  // Print the PPM signals that you have just read (optional)
	//printRCInput();

  // Check if Serial data is open and read the incoming data
  if (Serial.available ()) {
    lastMessage = millis ();  // remember when we last got input
    processInput ();          // now handle the input
    //Serial.print("Incoming message:");
    //Serial.println(myStr);
  }

  // EXEC will be true when the symbol '>' is received
  if (EXEC == true) {
    // Turn the state false for the next round
    EXEC = false;
    
    //Serial.print("Execution stage :: Command found:");
    //Serial.println(myStr);

    // To handle more than one incoming string you need to
    // write some code here ...
    
    int incThr = atoi(myStr);
    //int incAil = 
    //int incEle = 
    //int incRud = 
    
    thrValue = 1080+thrValue*1.5;
    
    // Forward those PPM signals to the flight controller board.
    writeRCOutput(thrValue, 1464, 1464, 1464);
    
    clearBuffer();
  }else {
    EXEC = false;
    //Serial.println("Waiting...");
  }
}
