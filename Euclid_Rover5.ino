/******************************************************************************
  Copyright (c) 2016, Intel Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <ros.h>
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

ros::NodeHandle  nh;

//Published Message

//Arduino periodicaly tries to connect to ROS, When a handshake is recieved it stops.
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

  //Subscribing to the different topics, defined above
  nh.subscribe(sub);
  nh.subscribe(sub1);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  setIsReady(true);

}

void registerArduino(bool amReady){
    arduinoRegisterMessage.data = amReady;
    mArduinoStatusPub.publish(&arduinoRegisterMessage);
}

void loop() {
  static unsigned long readyTimer = 0;
  static unsigned long messageTimer = 0;
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

  if (now > messageTimer) {
    if (gotMessage) {
      gotMessage = false;
    }
    else {
      stopMotors();
    }
    messageTimer = now + 5000;
  }

  nh.spinOnce();  //Allows ROS to run and to send/receive new messages.

}

