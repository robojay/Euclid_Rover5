#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <euclid_rover5_pkg/Encoder.h>
#include <math.h>
#include <limits.h>

using namespace std;

class Rover5 {
	ros::NodeHandle nh_;

	ros::Subscriber robotTwist_;
	ros::Subscriber updateTime_;
	ros::Subscriber encoder_;

	ros::Publisher leftMotor_;
	ros::Publisher rightMotor_;

	ros::Publisher odometry_;
	tf::TransformBroadcaster odometryTransform_;

	string twistTopic_, leftMotorTopic_, rightMotorTopic_, updateTimeTopic_;
	string odomTopic_, odomFrameId_, baseFrameId_, encoderTopic_;
	float wheelDiameterMm_;
	float wheelSpacingMm_;
	float countsPerRotation_;
	float maxSpeedMetersPerSecond_;

	int pwmLimit_;
	uint32_t updateTimeMilliseconds_; 	
	
	static const float PI = 3.1415927;
	int debugLevel_ ;

	int encoderCount[4]; // LeftRear, RightRear, LeftFront, RightFront
	double lastLeftCount;
	double lastRightCount;
	double lastEncoderTime;
	double pose[3]; // X, Y, Theta
	nav_msgs::Odometry odometryMessage;
	geometry_msgs::TransformStamped odometryTransformMessage;

	enum {LeftRear = 0, RightRear = 1, LeftFront = 2, RightFront = 3};
	enum {X = 0, Y = 1, Theta = 2};

public:
	Rover5(ros::NodeHandle &nh) {

		nh_ = nh;

		twistTopic_ = "/rover5/cmd_vel";
		leftMotorTopic_ = "/rover5/left_motor";
		rightMotorTopic_ = "/rover5/right_motor";
		updateTimeTopic_ = "/rover5/update_time";
		odomTopic_ = "odom";
		odomFrameId_ = "odom";
		baseFrameId_ = "base_link";
		encoderTopic_ = "/rover5/encoder";
		wheelDiameterMm_ = 60.0;
		wheelSpacingMm_ = 189.0;
		countsPerRotation_ = 83.3;
		maxSpeedMetersPerSecond_ = 0.25;
		debugLevel_ = 0;

		for (int i = 0; i < 4; i++) {
			encoderCount[i] = 0;
		}

		lastEncoderTime = 0.0;

		for (int i = 0; i < 3; i++) {
			// x, y, theta
			pose[i] = 0.0;
		}

  		// Load parameters from rover5.yaml

		try {
			nh.getParam("twist_topic", twistTopic_);
			nh.getParam("left_motor_topic", leftMotorTopic_);
			nh.getParam("right_motor_topic", rightMotorTopic_);
			nh.getParam("update_time_topic", updateTimeTopic_);
			nh.getParam("odom_topic", odomTopic_);
			nh.getParam("odom_frame_id", odomFrameId_);
			nh.getParam("base_frame_id", baseFrameId_);
			nh.getParam("encoder_topic", encoderTopic_);
			nh.getParam("wheel_diameter_mm", wheelDiameterMm_);
			nh.getParam("wheel_spacing_mm", wheelSpacingMm_);
			nh.getParam("counts_per_rotation", countsPerRotation_); 	
			nh.getParam("max_speed_m_per_s", maxSpeedMetersPerSecond_); 	
			nh.getParam("debug_level", debugLevel_);

			ROS_INFO("Parameters loaded");
		}

		catch(int e) {
   			ROS_WARN("Parameters not loaded, using defaults");
		}

		// Subscribe to twist
		robotTwist_ = nh_.subscribe(twistTopic_, 1, &Rover5::twistCallback, this);

		// Subscribe to update time (milliseconds per encoder count sample)
		updateTime_ = nh_.subscribe(updateTimeTopic_, 1, &Rover5::updateTimeCallback, this);

		// Subscribe to encoder counts
		encoder_ = nh_.subscribe(encoderTopic_, 1, &Rover5::updateEncoderCallback, this);

		updateTimeMilliseconds_ = 0;

		// Advertise left and right motor controls
		leftMotor_ = nh_.advertise<std_msgs::Int16>(leftMotorTopic_, 5);
		rightMotor_ = nh_.advertise<std_msgs::Int16>(rightMotorTopic_, 5);

		// Advertise odometry and odometry transform
		odometry_ = nh_.advertise<nav_msgs::Odometry>(odomTopic_, 10);

	}

	~Rover5() {
		stopMotors();
	}


	void stopMotors() {
		std_msgs::Int16 left, right;

		// turn off the motors and make sure to send message
		if (debugLevel_ > 0) ROS_INFO("Stopping motors");

		left.data = 0;
		right.data = 0;
		leftMotor_.publish(left);
		rightMotor_.publish(right);
	}

	void updateTimeCallback(const std_msgs::UInt32& utime) {
		if (debugLevel_ > 0) ROS_INFO("updateTimeCallback");

		updateTimeMilliseconds_ = utime.data;

		if (debugLevel_ > 0) ROS_INFO("Update Time = %d", updateTimeMilliseconds_);
	}

	void updateEncoderCallback(const euclid_rover5_pkg::Encoder& enc) {
		double encoderTime;
		double leftCount;
		double rightCount;
		double dt;

		if (debugLevel_ > 0) ROS_INFO("updateEncoderCallback");

		encoderTime = enc.header.stamp.toSec();
		dt = float(encoderTime - lastEncoderTime);
		if (debugLevel_ > 0) ROS_INFO("  Time = %f", encoderTime);
		if (debugLevel_ > 0) ROS_INFO("  dt = %f", dt);

		for (int i = 0; i < 4; i++) {
			encoderCount[i] = enc.count[i];
		}
		if (debugLevel_ > 0) ROS_INFO("  LeftRear = %d", encoderCount[LeftRear]);
		if (debugLevel_ > 0) ROS_INFO("  RightRear = %d", encoderCount[RightRear]);
		if (debugLevel_ > 0) ROS_INFO("  LeftFront = %d", encoderCount[LeftFront]);
		if (debugLevel_ > 0) ROS_INFO("  RightFront = %d", encoderCount[RightFront]);

		// average the encoders on each side
		leftCount = (encoderCount[LeftFront] + encoderCount[LeftRear]) / 2.0;
		rightCount = (encoderCount[RightFront] + encoderCount[RightRear]) / 2.0;

		if ((lastEncoderTime != 0.0) && (dt > 0.0)) {
			// only process after one valid message has been received

			double dLeft = leftCount - lastLeftCount;
			double dRight = rightCount - lastRightCount;

			double leftMm = (PI * wheelDiameterMm_ * dLeft) / countsPerRotation_;
			double rightMm = (PI * wheelDiameterMm_ * dRight) / countsPerRotation_;

			double dDistanceMm = (rightMm + leftMm) / 2.0;
			double dTheta = (rightMm - leftMm) / wheelSpacingMm_;

			pose[Theta] += dTheta;

			double dX = dDistanceMm * cos(pose[Theta]);
			double dY = dDistanceMm * sin(pose[Theta]);

			pose[X] += dX;
			pose[Y] += dY;

			double xLinearVelocityMmPerS = dX / dt;
			double yLinearVelocityMmPerS = dY / dt;
			double zAngularVelocityRadPerS = dTheta / dt;

			if (debugLevel_ > 0) ROS_INFO("  dX = %f", dX);
			if (debugLevel_ > 0) ROS_INFO("  dY = %f", dY);
			if (debugLevel_ > 0) ROS_INFO("  dTheta = %f", dTheta);

			geometry_msgs::Quaternion odomQuaternion = tf::createQuaternionMsgFromYaw(pose[Theta]);


			odometryMessage.header.stamp = enc.header.stamp;
			odometryMessage.header.frame_id = odomFrameId_;
			odometryMessage.child_frame_id = baseFrameId_;

			odometryMessage.pose.pose.position.x = pose[X] / 1000.0;
			odometryMessage.pose.pose.position.y = pose[Y] / 1000.0;
			odometryMessage.pose.pose.position.z = 0.0;
			odometryMessage.pose.pose.orientation = odomQuaternion;
			odometryMessage.twist.twist.linear.x = xLinearVelocityMmPerS / 1000.0;
			odometryMessage.twist.twist.linear.y = yLinearVelocityMmPerS / 1000.0;			
			odometryMessage.twist.twist.angular.z = zAngularVelocityRadPerS;

			odometry_.publish(odometryMessage);

			odometryTransformMessage.header.stamp = enc.header.stamp;
			odometryTransformMessage.header.frame_id = odomFrameId_;
			odometryTransformMessage.child_frame_id = baseFrameId_;
			odometryTransformMessage.transform.translation.x = pose[X];
			odometryTransformMessage.transform.translation.y = pose[Y];
			odometryTransformMessage.transform.translation.z = 0.0;
			odometryTransformMessage.transform.rotation = odomQuaternion;

			odometryTransform_.sendTransform(odometryTransformMessage);


		}

		lastEncoderTime = encoderTime;
		lastLeftCount = leftCount;
		lastRightCount = rightCount;

	}


	void twistCallback(const geometry_msgs::Twist& twist) {
		if (debugLevel_ > 0) ROS_INFO("twistCallback");

		if (debugLevel_ > 0) ROS_INFO("linear x = %f", twist.linear.x);
		if (debugLevel_ > 0) ROS_INFO("angular z = %f", twist.angular.z);

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


		if (debugLevel_ > 0) ROS_INFO("Velocity Left = %f", vl);
		if (debugLevel_ > 0) ROS_INFO("Velocity Right = %f", vr);

		// convert velocity to counts per update time
		// speed = ((wheel diamater * pi) / (counts per rev)) * (counts / time)
		// taking care of units, we get...

		int cmdLeft, cmdRight;
		cmdLeft = (vl * updateTimeMilliseconds_ * countsPerRotation_) / (wheelDiameterMm_ * PI); 
		cmdRight = (vr * updateTimeMilliseconds_ * countsPerRotation_) / (wheelDiameterMm_ * PI); 

		if (debugLevel_ > 0) ROS_INFO("Command Left = %d", cmdLeft);
		if (debugLevel_ > 0) ROS_INFO("Command Right = %d", cmdRight);

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
