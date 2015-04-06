#include <TimerThree.h>
#include <TimerOne.h>

//led and sonar pins
const int led = 11;
const int trig1 = 0;
const int trig2 = 1;
const int trig3 = 2;
const int echo1 = 5;
const int echo2 = 6;
const int echo3 = 7;

//motor pins
const int m_enable = 21;
const int left_pwm = 12;
const int left_dira = 13;
const int left_dirb = 14;
const int right_pwm = 15;
const int right_dira = 16;
const int right_dirb = 17;

//global variables for sonar
/*
  0 -  WAIT FOR ECHO status
  1 -  IN ECHO status
  2 -  ECHO COMPLETE status
*/
volatile char sonar_status;
volatile char curr_sonar = 1;
volatile char curr_isr; 
volatile char curr_trig;
volatile char curr_echo;

volatile char timeout_status = 0;

//global sonar results
volatile unsigned long curr_ten_micro = 0;
volatile unsigned long ten_micro1 = 0;
volatile unsigned long ten_micro2 = 0;
volatile unsigned long ten_micro3 = 0;

//global motor variables
char motor_state = 0;
unsigned long last_motor_write = 0;

void both_forward(){
  digitalWrite(left_dira, HIGH);
  digitalWrite(left_dirb, LOW);
  digitalWrite(right_dira, HIGH);
  digitalWrite(right_dirb, LOW);
}
void left_forward_right_backward(){
  digitalWrite(left_dira, HIGH);
  digitalWrite(left_dirb, LOW);
  digitalWrite(right_dira, LOW);
  digitalWrite(right_dirb, HIGH);
}
void left_backward_right_forward(){
  digitalWrite(left_dira, LOW);
  digitalWrite(left_dirb, HIGH);
  digitalWrite(right_dira, HIGH);
  digitalWrite(right_dirb, LOW);
}
void both_backward(){
  digitalWrite(left_dira, LOW);
  digitalWrite(left_dirb, HIGH);
  digitalWrite(right_dira, LOW);
  digitalWrite(right_dirb, HIGH);
  
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  pinMode(led, OUTPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  
  pinMode(m_enable, OUTPUT);
  pinMode(left_dira, OUTPUT);
  pinMode(left_dirb, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_dira, OUTPUT);
  pinMode(right_dirb, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  
  Serial.print("BEGIN PROGRAM\n");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  setSonar(curr_sonar);
  curr_ten_micro = 0;
  int i = 1;
  fireSonar();
  //wait sonar is not done or timout
  while(sonar_status != 2){ 
    i++;
    if(i > 5000){ //about 1.5 m
      Timer1.stop();
      sonar_status = 2; //manually change the status to "ECHO COMPLETE"
      timeout_status = 1; 

    }
  }    

  Timer1.stop();
  
  //output result
  
  if(curr_sonar == 1){
    ten_micro1 = curr_ten_micro;
    curr_sonar++;
  }else if(curr_sonar == 2){
    ten_micro2 = curr_ten_micro;
    curr_sonar++;
  }else{
    ten_micro3 = curr_ten_micro;
    Serial.write(ten_micro1*10/58);
    Serial.write(ten_micro2*10/58);
    Serial.write(ten_micro3*10/58);
    Serial.print("\n");
    curr_sonar = 1;
    timeout_status = 0;
    
    byte response[4] = {255, 0, 0, '\n'};
    Serial.setTimeout(250); //half a second timeout
    Serial.readBytesUntil('\n', response, 4);
    //read response. If dir = 0, 1, 2, or 3, write to motors    
    digitalWrite(m_enable, HIGH);
    if(response[0] == 0){
      both_forward();
      analogWrite(left_pwm, response[1]);
      analogWrite(right_pwm, response[2]);
      digitalWrite(led, HIGH);
    }else if(response[0] == 1){
      left_forward_right_backward();
      analogWrite(left_pwm, response[1]);
      analogWrite(right_pwm, response[2]);
      digitalWrite(led, HIGH);
    }else if(response[0] == 2){
      left_backward_right_forward();
      analogWrite(left_pwm, response[1]);
      analogWrite(right_pwm, response[2]);
      digitalWrite(led, HIGH);
    }else if(response[0] == 3){
      both_backward();
      analogWrite(left_pwm, response[1]);
      analogWrite(right_pwm, response[2]);
      digitalWrite(led, HIGH);
    }else{
      //error handling or no response received
      both_forward();
      analogWrite(left_pwm, 0);
      analogWrite(right_pwm, 0);
      digitalWrite(led, LOW);
    }
    
  }
  
  delay(20);
  
}

void setSonar(int sonar){
  if(sonar == 1){
    curr_trig = trig1;
    curr_echo = echo1;
    curr_isr = 0;
  }
  if(sonar == 2){
    curr_trig = trig2;
    curr_echo = echo2;
    curr_isr = 1;
  }
  if(sonar == 3){
    curr_trig = trig3;
    curr_echo = echo3;
    curr_isr = 2;
  }
}

void fireSonar(){
  //prep for echo
  attachInterrupt(curr_isr, echoChange, CHANGE); 
  sonar_status = 0; //"wait for echo status"
  
  digitalWrite(curr_trig, HIGH); //start 10 us trigger
  delayMicroseconds(10); 
  digitalWrite(curr_trig, LOW); //end trigger pulse 
  
  curr_ten_micro = 0;
}

void echoChange(){
  //check if this is start or end of echo
  if(sonar_status != 2){ //ignore if already timed out
  
    int echo_status = digitalRead(curr_echo); 
    if(echo_status == HIGH){
      //start of echo; begin timing
      curr_ten_micro = 0;
      //myTimer.begin(countTenMicro, 10); //fire every ten us
      Timer1.initialize(10); //fire every ten us
      Timer1.attachInterrupt(countTenMicro);
      Timer1.start();
      sonar_status = 1; //set "IN ECHO" status
    }else{
      //end of echo; stop timer
      Timer1.stop();
      sonar_status = 2; //set "ECHO COMPLETE" status
    }
  }
  
}

void countTenMicro(void){
  curr_ten_micro++;
}
