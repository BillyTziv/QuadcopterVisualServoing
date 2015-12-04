#include <Servo.h>

Servo motor;

int speed;

void setup() {
  Serial.begin(9600);
  
  motor.attach(10);
  motor.write(40);
  
  delay(3000);
}

void loop() {
  if( Serial.available() > 0 ) {
    speed = Serial.parseInt();
    motor.write(speed);
    Serial.println(speed);
    
    
  }
    
  delay(15);
}
