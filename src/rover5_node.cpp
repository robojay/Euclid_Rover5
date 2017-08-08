#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <math.h>

using namespace std;

class Rover5 {
	ros::NodeHandle nh_;

	ros::Subscriber robotTwist_;
	ros::Subscriber updateTime_;

	ros::Publisher leftMotor_;
	ros::Publisher rightMotor_;

	string twistTopic_, leftMotorTopic_, rightMotorTopic_, updateTimeTopic_;
	float wheelDiameterMm_;
	float wheelSpacingMm_;
	float countsPerRotation_;
	float maxSpeedMetersPerSecond_;

	int pwmLimit_;
	uint32_t updateTimeMilliseconds_; 	
	
	static const float PI = 3.1415927;

public:
	Rover5(ros::NodeHandle &nh) {

		nh_ = nh;

		twistTopic_ = "/rover5/cmd_vel";
		leftMotorTopic_ = "/rover5/left_motor";
		rightMotorTopic_ = "/rover5/right_motor";
		wheelDiameterMm_ = 60.0;
		wheelSpacingMm_ = 189.0;
		countsPerRotation_ = 83.3;
		maxSpeedMetersPerSecond_ = 0.25;


  		// Load parameters from bot2020.yaml

		try {
			nh.getParam("twist_topic", twistTopic_);
			nh.getParam("left_motor_topic", leftMotorTopic_);
			nh.getParam("right_motor_topic", rightMotorTopic_);
			nh.getParam("update_time_topic", updateTimeTopic_);
			nh.getParam("wheel_diameter_mm", wheelDiameterMm_);
			nh.getParam("wheel_spacing_mm", wheelSpacingMm_);
			nh.getParam("counts_per_rotation", countsPerRotation_); 	
			nh.getParam("max_speed_m_per_s", maxSpeedMetersPerSecond_); 	

			ROS_INFO("Parameters loaded");
		}

		catch(int e) {
   			ROS_WARN("Parameters not loaded, using defaults");
		}

		// Subscribe to twist
		robotTwist_ = nh_.subscribe(twistTopic_, 1, &Rover5::twistCallback, this);

		// Subsribe to update time (milliseconds per encoder count sample)
		updateTime_ = nh_.subscribe(updateTimeTopic_, 1, &Rover5::updateTimeCallback, this);

		updateTimeMilliseconds_ = 0;


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

	void updateTimeCallback(const std_msgs::UInt32& utime) {
		ROS_INFO("updateTimeCallback");

		updateTimeMilliseconds_ = utime.data;

		ROS_INFO("Update Time = %d", updateTimeMilliseconds_);
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

		// clamp to limits

		if (vr > maxSpeedMetersPerSecond_) {
			vr = maxSpeedMetersPerSecond_;
		}
		else if (vr < -maxSpeedMetersPerSecond_) {
			vr = -maxSpeedMetersPerSecond_;
		}

		if (vl > maxSpeedMetersPerSecond_) {
			vl = maxSpeedMetersPerSecond_;
		}
		else if (vl < -maxSpeedMetersPerSecond_) {
			vl = -maxSpeedMetersPerSecond_;
		}


		ROS_INFO("Velocity Left = %f", vl);
		ROS_INFO("Velocity Right = %f", vr);

		// convert velocity to counts per update time
		// speed = ((wheel diamater * pi) / (counts per rev)) * (counts / time)
		// taking care of units, we get...

		int cmdLeft, cmdRight;
		cmdLeft = (vl * updateTimeMilliseconds_ * countsPerRotation_) / (wheelDiameterMm_ * PI); 
		cmdRight = (vr * updateTimeMilliseconds_ * countsPerRotation_) / (wheelDiameterMm_ * PI); 

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
