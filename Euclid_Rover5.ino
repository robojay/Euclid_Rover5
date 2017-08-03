#include <ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


const uint8_t LeftRearPwm = 4;
const uint8_t LeftRearDir = 5;
const uint8_t RightRearPwm = 6;
const uint8_t RightRearDir = 7;
const uint8_t LeftFrontPwm = 8;
const uint8_t LeftFrontDir = 9;
const uint8_t RightFrontPwm = 10;
const uint8_t RightFrontDir = 11;

const uint8_t LeftRearEncA = 2;
const uint8_t RightRearEncA = 3;
const uint8_t LeftFrontEncA = 18; 
const uint8_t RightFrontEncA = 19;

const uint8_t LeftRearEncB = 22;
const uint8_t RightRearEncB = 23;
const uint8_t LeftFrontEncB = 24; 
const uint8_t RightFrontEncB = 25;

const unsigned long SensorUpdateTime = 20;

ros::NodeHandle  nh;

volatile int32_t leftRearCount = 0;
volatile int32_t leftFrontCount = 0;
volatile int32_t rightRearCount = 0;
volatile int32_t rightFrontCount = 0;

volatile uint32_t leftRearDt = 0;
volatile uint32_t leftFrontDt = 0;
volatile uint32_t rightRearDt = 0;
volatile uint32_t rightFrontDt = 0;

volatile uint32_t leftRearTime = 0;
volatile uint32_t leftFrontTime = 0;
volatile uint32_t rightRearTime = 0;
volatile uint32_t rightFrontTime = 0;

volatile uint32_t leftRearNow = 0;
volatile uint32_t leftFrontNow = 0;
volatile uint32_t rightRearNow = 0;
volatile uint32_t rightFrontNow = 0;

enum MotorState {Off, Forward, Reverse};

volatile MotorState leftRearState = Off;
volatile MotorState leftFrontState = Off;
volatile MotorState rightRearState = Off;
volatile MotorState rightFrontState = Off;

//
// Double check encoder directions
// On my Rover 5, with color matching of wires to pins
// One set of encoders were wired backwards
//

void leftRearInterrupt() {
  leftRearNow = micros();
  leftRearDt = leftRearNow - leftRearTime;
  leftRearTime = leftRearNow;
      
  if (digitalRead(LeftRearEncB) == HIGH) {
    leftRearCount++;
    leftRearState = Forward;
  }
  else {
    leftRearCount--;
    leftRearState = Reverse;
  }
}

void leftFrontInterrupt() {
  leftFrontNow = micros();
  leftFrontDt = leftFrontNow - leftFrontTime;
  leftFrontTime = leftFrontNow;

  if (digitalRead(LeftFrontEncB) == LOW) {
    leftFrontCount++;
    leftFrontState = Forward;
  }
  else {
    leftFrontCount--;
    leftFrontState = Reverse;
  }
}

void rightRearInterrupt() {
  rightRearNow = micros();
  rightRearDt = rightRearNow - rightRearTime;
  rightRearTime = rightRearNow;

  if (digitalRead(RightRearEncB) == HIGH) {
    rightRearCount++;
    rightRearState = Forward;
  }
  else {
    rightRearCount--;
    rightRearState = Reverse;
  }
}

void rightFrontInterrupt() {
  rightFrontNow = micros();
  rightFrontDt = rightFrontNow - rightFrontTime;
  rightFrontTime = rightFrontNow;

  if (digitalRead(RightFrontEncB) == HIGH) {
    rightFrontCount++;
    rightFrontState = Forward;
  }
  else {
    rightFrontCount--;
    rightFrontState = Reverse;
  }
}

bool isReady = false;
void setIsReady(bool status){
  isReady = status;
}    

bool gotMessage = false;

void setLeft( const std_msgs::Int16& msg) {
  static int16_t lastLeft = 0;
  int16_t leftSpeed;
  
  gotMessage = true;
  
  leftSpeed = msg.data;

  if (leftSpeed != lastLeft) {
    lastLeft = leftSpeed;
    if (leftSpeed > 0) {
      leftSpeed = min(leftSpeed, 255);
      analogWrite(LeftFrontPwm, leftSpeed);
      analogWrite(LeftRearPwm, leftSpeed);
      digitalWrite(LeftFrontDir, LOW);
      digitalWrite(LeftRearDir, HIGH);
    }
    else {
      leftSpeed = min(-leftSpeed, 255);
      analogWrite(LeftFrontPwm, leftSpeed);
      analogWrite(LeftRearPwm, leftSpeed);
      digitalWrite(LeftFrontDir, HIGH);
      digitalWrite(LeftRearDir, LOW);
    }
  }
}

void setRight( const std_msgs::Int16& msg) {
  static int16_t lastRight = 0;
  int16_t rightSpeed;

  gotMessage = true;
  
  rightSpeed = msg.data;

  if (rightSpeed != lastRight) {
    lastRight = rightSpeed;  
    if (rightSpeed > 0) {
      rightSpeed = min(rightSpeed, 255);
      analogWrite(RightFrontPwm, rightSpeed);
      analogWrite(RightRearPwm, rightSpeed);
      digitalWrite(RightFrontDir, LOW);
      digitalWrite(RightRearDir, HIGH);
    }
    else {
      rightSpeed = min(-rightSpeed, 255);
      analogWrite(RightFrontPwm, rightSpeed);
      analogWrite(RightRearPwm, rightSpeed);
      digitalWrite(RightFrontDir, HIGH);
      digitalWrite(RightRearDir, LOW);
    }
  }
}


void stopMotors() {
  analogWrite(LeftRearPwm, 0);     
  analogWrite(LeftFrontPwm, 0);     
  analogWrite(RightRearPwm, 0);     
  analogWrite(RightFrontPwm, 0);
}

//Declaration of the publisher
std_msgs::Bool arduinoRegisterMessage;
ros::Publisher mArduinoStatusPub("/rover5/ready", &arduinoRegisterMessage);

std_msgs::Int32 leftRearCountMessage;
ros::Publisher mLeftRearCount("/rover5/left/rear/count", &leftRearCountMessage);
std_msgs::Int32 leftFrontCountMessage;
ros::Publisher mLeftFrontCount("/rover5/left/front/count", &leftFrontCountMessage);
std_msgs::Int32 rightRearCountMessage;
ros::Publisher mRightRearCount("/rover5/right/rear/count", &rightRearCountMessage);
std_msgs::Int32 rightFrontCountMessage;
ros::Publisher mRightFrontCount("/rover5/right/front/count", &rightFrontCountMessage);

std_msgs::UInt32 leftRearDtMessage;
ros::Publisher mLeftRearDt("/rover5/left/rear/dt", &leftRearDtMessage);
std_msgs::UInt32 leftFrontDtMessage;
ros::Publisher mLeftFrontDt("/rover5/left/front/dt", &leftFrontDtMessage);
std_msgs::UInt32 rightRearDtMessage;
ros::Publisher mRightRearDt("/rover5/right/rear/dt", &rightRearDtMessage);
std_msgs::UInt32 rightFrontDtMessage;
ros::Publisher mRightFrontDt("/rover5/right/front/dt", &rightFrontDtMessage);

//subscribers declaration for the gain settings
ros::Subscriber<std_msgs::Int16> sub("/rover5/left_motor", setLeft );
ros::Subscriber<std_msgs::Int16> sub1("/rover5/right_motor", setRight );


void setup() {

  pinMode(LeftRearDir, OUTPUT);
  pinMode(LeftFrontDir, OUTPUT);
  pinMode(RightRearDir, OUTPUT);
  pinMode(RightFrontDir, OUTPUT);
  pinMode(LeftRearPwm, OUTPUT);
  pinMode(LeftFrontPwm, OUTPUT);
  pinMode(RightRearPwm, OUTPUT);
  pinMode(RightFrontPwm, OUTPUT);

  pinMode(LeftRearEncA, INPUT);
  pinMode(RightRearEncA, INPUT);
  pinMode(LeftFrontEncA, INPUT);
  pinMode(RightFrontEncA, INPUT);
  pinMode(LeftRearEncB, INPUT);
  pinMode(RightRearEncB, INPUT);
  pinMode(LeftFrontEncB, INPUT);
  pinMode(RightFrontEncB, INPUT);

  analogWrite(LeftRearPwm, 0);     
  analogWrite(LeftFrontPwm, 0);     
  analogWrite(RightRearPwm, 0);     
  analogWrite(RightFrontPwm, 0);

  nh.initNode(); //Initialize the node (needed by ROS)
  setIsReady(false); //Initialize ready status to false

  //Advertise the topic
  nh.advertise(mArduinoStatusPub);
  nh.advertise(mLeftRearCount);
  nh.advertise(mLeftFrontCount);
  nh.advertise(mRightRearCount);
  nh.advertise(mRightFrontCount);
  nh.advertise(mLeftRearDt);
  nh.advertise(mLeftFrontDt);
  nh.advertise(mRightRearDt);
  nh.advertise(mRightFrontDt);

  //Subscribing to the different topics, defined above
  nh.subscribe(sub);
  nh.subscribe(sub1);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(LeftRearEncA), leftRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(LeftFrontEncA), leftFrontInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RightRearEncA), rightRearInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RightFrontEncA), rightFrontInterrupt, RISING);

  setIsReady(true);

}

void registerArduino(bool amReady){
    arduinoRegisterMessage.data = amReady;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
}

void sendEncoders(){
    leftRearCountMessage.data = leftRearCount;
    mLeftRearCount.publish(&leftRearCountMessage);
    leftFrontCountMessage.data = leftFrontCount;
    mLeftFrontCount.publish(&leftFrontCountMessage);

    rightRearCountMessage.data = rightRearCount;
    mRightRearCount.publish(&rightRearCountMessage);
    rightFrontCountMessage.data = rightFrontCount;
    mRightFrontCount.publish(&rightFrontCountMessage);

    leftRearDtMessage.data = leftRearDt;
    mLeftRearDt.publish(&leftRearDtMessage);
    leftFrontDtMessage.data = leftFrontDt;
    mLeftFrontDt.publish(&leftFrontDtMessage);
    rightRearDtMessage.data = rightRearDt;
    mRightRearDt.publish(&rightRearDtMessage);
    rightFrontDtMessage.data = rightFrontDt;
    mRightFrontDt.publish(&rightFrontDtMessage);

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

