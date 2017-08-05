#include <ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <PID_v1.h>
#include "euclid_rover5.h"

// Wrappers for encoder interrupt handler
void leftRearInterrupt() {
  encoderHandler(LeftRear);
}

void rightRearInterrupt() {
  encoderHandler(RightRear);
}

void leftFrontInterrupt() {
  encoderHandler(LeftFront);
}

void rightFrontInterrupt() {
  encoderHandler(RightFront);
}


// Actual encoder interrupt handler
void encoderHandler(uint8_t wheel) {        
  if (digitalRead(EncBPin[wheel]) == encoder[wheel].forward) {
    encoder[wheel].count++;
    encoder[wheel].state = Forward;
  }
  else {
    encoder[wheel].count--;
    encoder[wheel].state = Reverse;
  }
}
  
void setIsReady(bool status){
  isReady = status;
}    

void setMotor(uint8_t wheel, int16_t pwm) {
  int16_t p;
  
  if (pwm != motor[wheel].pwm) {
    motor[wheel].pwm = pwm;
    if (pwm > 0) {
      p = min(pwm, 255);
      analogWrite(PwmPin[wheel], p);
      digitalWrite(DirPin[wheel], motor[wheel].forward);
    }
    else {
      p = min(-pwm, 255);
      analogWrite(PwmPin[wheel], p);
      digitalWrite(DirPin[wheel], motor[wheel].reverse);
    }
  }
}
  

void setLeft( const std_msgs::Int16& msg) {  
  gotMessage = true;
  setMotor(LeftFront, msg.data);
  setMotor(LeftRear, msg.data);
}

void setRight( const std_msgs::Int16& msg) {  
  gotMessage = true;
  setMotor(RightFront, msg.data);
  setMotor(RightRear, msg.data);
}


void stopMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    analogWrite(PwmPin[i], 0);
  }
}


void setup() {

  for (uint8_t i = 0; i < 4; i++) {

    // configure I/O pins
    pinMode(DirPin[i], OUTPUT);
    pinMode(PwmPin[i], OUTPUT);
    analogWrite(PwmPin[i], 0);
    pinMode(EncAPin[i], INPUT);
    pinMode(EncBPin[i], INPUT);

    // initialize arrays
    encoder[i].state = Off;
    encoder[i].count = 0;
    encoder[i].dCount = 0.0;

    motor[i].state = Off ;
    motor[i].setpoint = 0;
    motor[i].pwm = 0.0;  

    pid[i].Setup(&encoder[i].dCount, &motor[i].pwm, &motor[i].setpoint, Kp, Ki, Kd, DIRECT);
  
  }

  //
  // Double check encoder directions
  // On my Rover 5, with color matching of wires to pins
  // One set of encoders were wired backwards
  //
  encoder[LeftRear].forward = HIGH;
  encoder[RightRear].forward = HIGH;
  encoder[LeftFront].forward = LOW;
  encoder[RightFront].forward = HIGH;

  motor[LeftFront].forward = LOW;
  motor[LeftRear].forward = HIGH;
  motor[RightFront].forward = LOW;
  motor[RightRear].forward = HIGH;

  motor[LeftFront].reverse = HIGH;
  motor[LeftRear].reverse = LOW;
  motor[RightFront].reverse = HIGH;
  motor[RightRear].reverse = LOW;


  nh.initNode(); //Initialize the node (needed by ROS)
  setIsReady(false); //Initialize ready status to false

  //Advertise topics
  nh.advertise(mArduinoStatusPub);
  nh.advertise(mLeftRearCount);
  nh.advertise(mLeftFrontCount);
  nh.advertise(mRightRearCount);
  nh.advertise(mRightFrontCount);

  //Subscribe topics
  nh.subscribe(sSetLeft);
  nh.subscribe(sSetRight);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(EncAPin[LeftRear]), leftRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[LeftFront]), leftFrontInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[RightRear]), rightRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[RightFront]), rightFrontInterrupt, RISING);

  setIsReady(true);

}

void registerArduino(bool amReady){
    arduinoRegisterMessage.data = amReady;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
}

void sendEncoders(){
    for (uint8_t i = 0; i < 4; i++) {
      countMessage[i].data = encoder[i].count;      
      encoder[i].dCount = (double)encoder[i].count;
    }
    mLeftRearCount.publish(&countMessage[LeftRear]);
    mRightRearCount.publish(&countMessage[RightRear]);
    mLeftFrontCount.publish(&countMessage[LeftFront]);
    mRightFrontCount.publish(&countMessage[RightFront]);
}


void loop() {
  static unsigned long readyTimer = 0;
  static unsigned long messageTimer = 0;
  static unsigned long sensorMessageTimer = 0;
  static bool ledState = false;
  unsigned long now;

  now = millis();
    
  //Register the arduino in ROS and wait for the system to be ready.
  if (now >= readyTimer) {
    registerArduino(isReady);
    readyTimer = now + 1000;
    ledState = !ledState;
    if (ledState) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);      
    }
  }

  if (now >= messageTimer) {
    if (gotMessage) {
      gotMessage = false;
    }
    else {
      stopMotors();
    }
    messageTimer = now + 5000;
  }

  if (now >= sensorMessageTimer) {
    sendEncoders();
    sensorMessageTimer = now + SensorUpdateTime;
  }

  nh.spinOnce();  //Allows ROS to run and to send/receive new messages.

}

