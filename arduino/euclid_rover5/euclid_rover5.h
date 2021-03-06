#ifndef _EUCLID_ROVER5_H_

// Constants

enum WheelState {Off, Forward, Reverse};
enum {LeftRear = 0, RightRear = 1, LeftFront = 2, RightFront = 3};

const uint8_t PwmPin[4] = {4, 6, 8, 10};
const uint8_t DirPin[4] = {5, 7, 9, 11};
const uint8_t EncAPin[4] = {2, 3, 18, 19};
const uint8_t EncBPin[4] = {22, 23, 24, 25};

const uint8_t UpdateFrequency = 10; // Hertz 
const uint32_t UpdateTime = 1000 / UpdateFrequency; // milliseconds
const uint32_t RosTime = 5000;
const uint32_t StatusTime = 1000;

const double Kp = 6.0;
const double Ki = 0.5;
const double Kd = 0.0;

struct Encoder {
  volatile WheelState state;
  volatile int32_t count;
  int32_t lastCount;
  double dCount;
  uint8_t forward;
};

struct Motor {
  WheelState state;
  double setpoint;
  double pwm;
  uint8_t forward;
  uint8_t reverse;
};

// Globals

ros::NodeHandle nh;

Encoder encoder[4];
Motor motor[4];

PID pid[4];

bool gotMessage = false;
bool isReady = false;

//
// ROS publishers and subscribers
//
//Declaration of the publisher
std_msgs::Bool arduinoRegisterMessage;
ros::Publisher mArduinoStatusPub("/rover5/ready", &arduinoRegisterMessage);

std_msgs::Int32 countMessage[4];

ros::Publisher mLeftRearCount("/rover5/left/rear/count", &countMessage[LeftRear]);
ros::Publisher mLeftFrontCount("/rover5/left/front/count", &countMessage[LeftFront]);
ros::Publisher mRightRearCount("/rover5/right/rear/count", &countMessage[RightRear]);
ros::Publisher mRightFrontCount("/rover5/right/front/count", &countMessage[RightFront]);

std_msgs::UInt32 updateTime;;

ros::Publisher mUpdateTime("/rover5/update_time", &updateTime);

std_msgs::UInt32 debug;
ros::Publisher mDebug("/rover5/debug", &debug);


void setLeft( const std_msgs::Int16& msg);
void setRight( const std_msgs::Int16& msg);

ros::Subscriber<std_msgs::Int16> sSetLeft("/rover5/left_motor", setLeft );
ros::Subscriber<std_msgs::Int16> sSetRight("/rover5/right_motor", setRight );


#endif
