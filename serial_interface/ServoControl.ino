#include <Servo.h>

Servo myservo;
const int servo_input = 13;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(0);
  pinMode(servo_input, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int input = digitalRead(servo_input);
  if(input == LOW){
    myservo.write(0);
  }else{
    myservo.write(90);
  }
}
