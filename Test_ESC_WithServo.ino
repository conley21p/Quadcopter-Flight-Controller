#include <Servo.h>
int speed = 1000;
int count = 0;
Servo motors[4];

void setup() {

//   delay(4000);
  for(int i =6; i <10;i++){
    motors[count].attach(i);
    motors[count++].writeMicroseconds(1000); 
  }
  Serial.begin(57600);
  Serial.println("begninning");

delay(1000);
}

void loop() {
  count =0;
  for(int i =6; i <10;i++){
    motors[count].attach(i);
    motors[count++].writeMicroseconds(speed); 
  }
  if(speed >= 1800){
    speed = 1000;
  }else {speed+=100;}
delay(1500);
}
