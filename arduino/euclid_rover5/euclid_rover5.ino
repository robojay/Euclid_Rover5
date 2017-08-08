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

void setMotor(uint8_t wheel, const std_msgs::Int16& msg) {
  
  WheelState w;
  
  gotMessage = true;
  motor[wheel].setpoint = (double)(abs(msg.data));
  if (msg.data > 0) {
    w = Forward;  
  }
  else if (msg.data < 0) {
    w = Reverse;
  }
  else {
    w = Off;
  }
  motor[wheel].state = w;
}


void setLeft( const std_msgs::Int16& msg) {  
  setMotor(LeftFront, msg);
  setMotor(LeftRear, msg);
}

void setRight( const std_msgs::Int16& msg) {  
  setMotor(RightFront, msg);
  setMotor(RightRear, msg);
}

void stopMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    analogWrite(PwmPin[i], 0);
  }
}

void registerArduino(bool amReady){
    arduinoRegisterMessage.data = amReady;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
}

void sendEncoders(){
    mUpdateTime.publish(&updateTime);
    mLeftRearCount.publish(&countMessage[LeftRear]);
    mRightRearCount.publish(&countMessage[RightRear]);
    mLeftFrontCount.publish(&countMessage[LeftFront]);
    mRightFrontCount.publish(&countMessage[RightFront]);
}

void handleEncoders() {
  static int32_t c[4];

  // doing these is separate steps to minimize delay skews
  // between channels
  
  for (uint8_t i = 0; i < 4; i++) {
    c[i] = encoder[i].count;
  }

  for (uint8_t i = 0; i < 4; i++) {
    countMessage[i].data = c[i];      
    encoder[i].dCount = (double)(abs(c[i] - encoder[i].lastCount));
    encoder[i].lastCount = c[i];
  }  

}

void handleMotor(uint8_t wheel) {
  int16_t p;

  p = (int16_t)(motor[wheel].pwm);
  p = min(p, 255);
  analogWrite(PwmPin[wheel], p);
  
  if (motor[wheel].state ==  Forward) {
    digitalWrite(DirPin[wheel], motor[wheel].forward);
  }
  else if (motor[wheel].state == Reverse) {
    digitalWrite(DirPin[wheel], motor[wheel].reverse);
  }
  else {
    analogWrite(PwmPin[wheel], 0);      
  }
}
  
void handleMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    handleMotor(i);
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

    updateTime.data = UpdateTime;

    pid[i].Setup(&encoder[i].dCount, &motor[i].pwm, &motor[i].setpoint, Kp, Ki, Kd, DIRECT);
    pid[i].SetSampleTime(UpdateTime);
    pid[i].SetOutputLimits(0, 255);
  
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
  nh.advertise(mUpdateTime);

  nh.advertise(mDebug);

  //Subscribe topics
  nh.subscribe(sSetLeft);
  nh.subscribe(sSetRight);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(EncAPin[LeftRear]), leftRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[LeftFront]), leftFrontInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[RightRear]), rightRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(EncAPin[RightFront]), rightFrontInterrupt, RISING);


  for (uint8_t i = 0; i < 4; i++) {
    pid[i].SetMode(AUTOMATIC);
  }
  
  setIsReady(true);

}


void loop() {
  static unsigned long readyTimer = 0;
  static unsigned long rosTimer = 0;
  static unsigned long updateTimer = 0;
  static bool ledState = false;
  unsigned long now;
  boolean doMotors = false;

  now = millis();

  if (now >= updateTimer) {
    handleEncoders();
    sendEncoders();
    updateTimer = now + UpdateTime;
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (pid[i].Compute()) {
      doMotors = true;
    }
  }

  if (doMotors) {
    handleMotors();
    //debug.data = (uint32_t)encoder[LeftFront].dCount;
    debug.data = (uint32_t)motor[LeftFront].pwm;
    mDebug.publish(&debug);
  }

    
  //Register the arduino in ROS and wait for the system to be ready.
  if (now >= readyTimer) {
    registerArduino(isReady);
    ledState = !ledState;
    if (ledState) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);      
    }
    readyTimer = now + StatusTime;
  }

  if (now >= rosTimer) {
    if (gotMessage) {
      gotMessage = false;
    }
    else {
      stopMotors();
    }
    rosTimer = now + RosTime;
  }


  nh.spinOnce();  //Allows ROS to run and to send/receive new messages.

}

