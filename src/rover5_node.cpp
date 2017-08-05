#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <math.h>

using namespace std;

class Rover5 {
	ros::NodeHandle nh_;

	ros::Subscriber robotTwist_;

	ros::Publisher leftMotor_;
	ros::Publisher rightMotor_;

	string twistTopic_, leftMotorTopic_, rightMotorTopic_;
	float leftRpmMax_;
	float rightRpmMax_;
	float leftWheelDiameterMm_;
	float rightWheelDiameterMm_;
	float wheelSpacingMm_;
	int pwmLimit_;	
	
	static const float PI = 3.1415927;

public:
	Rover5(ros::NodeHandle &nh) {

		nh_ = nh;

		twistTopic_ = "/rover5/cmd_vel";
		leftMotorTopic_ = "/rover5/left_motor";
		rightMotorTopic_ = "/rover5/right_motor";
		leftRpmMax_ = 90.0;
		rightRpmMax_ = 90.0;
		leftWheelDiameterMm_ = 65.0;
		rightWheelDiameterMm_ = 65.0;
		wheelSpacingMm_ = 105.0;
		pwmLimit_ = 255; 	


  		// Load parameters from bot2020.yaml

		try {
			nh.getParam("twist_topic", twistTopic_);
			nh.getParam("left_motor_topic", leftMotorTopic_);
			nh.getParam("right_motor_topic", rightMotorTopic_);
			nh.getParam("left_rpm_max", leftRpmMax_);
			nh.getParam("right_rpm_max", rightRpmMax_);
			nh.getParam("left_wheel_diameter_mm", leftWheelDiameterMm_);
			nh.getParam("right_wheel_diameter_mm", rightWheelDiameterMm_);
			nh.getParam("wheel_spacing_mm", wheelSpacingMm_);
			nh.getParam("pwm_limit", pwmLimit_); 	

			ROS_INFO("Parameters loaded");
		}

		catch(int e) {
   			ROS_WARN("Parameters not loaded, using defaults");
		}

		// Subscribe to bot2020 twist
		robotTwist_ = nh_.subscribe(twistTopic_, 1, &Rover5::twistCallback, this);

		// Advertise left and right motor controls
		leftMotor_ = nh_.advertise<std_msgs::Int16>(leftMotorTopic_, 5);
		rightMotor_ = nh_.advertise<std_msgs::Int16>(rightMotorTopic_, 5);

	}

	~Rover5() {
		stopMotors();
	}

	void stopMotors() {
		std_msgs::Int16 left, right;

		// turn off the motors and make sure to send message
		ROS_INFO("Stopping motors");

		left.data = 0;
		right.data = 0;
		leftMotor_.publish(left);
		rightMotor_.publish(right);
	}

	void twistCallback(const geometry_msgs::Twist& twist) {
		ROS_INFO("twistCallback");

		ROS_INFO("linear x = %f", twist.linear.x);
		ROS_INFO("angular z = %f", twist.angular.z);

		// Twist units are m/s and rad/s
		// need to convert to left and right PWM values
		// given the known geometry of the robot
		// and the motor characteristics
		//
		// hints here: http://moorerobots.com/blog/post/4

		float vl, vr;
		float vc, wc;

		vc = twist.linear.x;
		wc = twist.angular.z;

		vr = ((2.0 * vc) + (wc * wheelSpacingMm_) / 1000.0) / 2.0;
		vl = ((2.0 * vc) - (wc * wheelSpacingMm_) / 1000.0) / 2.0;

		ROS_INFO("Velocity Left = %f", vl);
		ROS_INFO("Velocity Right = %f", vr);

		float pwmLeft, pwmRight;

		pwmLeft = round((vl * pwmLimit_ * 1000.0 * 60.0) / (leftRpmMax_ * leftWheelDiameterMm_ * PI));
		pwmRight = round((vr * pwmLimit_ * 1000.0 * 60.0) / (rightRpmMax_ * rightWheelDiameterMm_ * PI));

		ROS_INFO("PWM Left = %f", pwmLeft);
		ROS_INFO("PWM Right = %f", pwmRight);

		int cmdLeft, cmdRight;

		// clamp to the limit

		if (pwmLeft >= 0.0) {
			cmdLeft = std::min(int(pwmLeft), pwmLimit_);
		}
		else {
			cmdLeft = std::max(int(pwmLeft), -pwmLimit_);
		}


		if (pwmRight >= 0.0) {
			cmdRight = std::min(int(pwmRight), pwmLimit_);
		}
		else {
			cmdRight = std::max(int(pwmRight), -pwmLimit_);
		}

		ROS_INFO("Command Left = %d", cmdLeft);
		ROS_INFO("Command Right = %d", cmdRight);

		std_msgs::Int16 left, right;

		left.data = cmdLeft;
		right.data = cmdRight;
		leftMotor_.publish(left);
		rightMotor_.publish(right);

		
	}


};


int main(int argc, char** argv) {
	ros::init(argc, argv, "Rover5");
	ros::NodeHandle nh;
	Rover5 myBot(nh);

	myBot.stopMotors();
	ros::spin();

	return 0;
}
