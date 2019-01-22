#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <iostream>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

double angular, linear;
//odom vars
double posX, posY, yaw;
double pi=3.1416;
//bumper vars
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
//laser vars
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;

void bumperCallback(const /*kobuki_msgs::BumperEvent::ConstPtr&*/ kobuki_msgs::BumperEvent msg){
	//fill with your code
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min) {
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++) {
			if(laserRange > msg->ranges[i] {
				laserRange = msg->ranges[i];
			}
		}
	}
	else {
		for(int i = 0; i < laserSize; i++) {
			if (laserRange > msg->ranges[i])
				laserRange = msg->ranges[i];
			}
		}
	}
	if (laserRange == 11)
		laserRange = 0;

	//ROS_INFO("Size of laser scan array: %i and size of offset %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.",  posX, posY, yaw, yaw*180/pi);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener"); //ros node initiated
	ros::NodeHandle nh; //declare for later subscriptions
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);
	
	//only allowable publisher!!!! 
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0; // the timer just started, so we know it is less than 480, no need to check.

	while(ros::ok() && secondsElapsed <= 480)
    {
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);
		if(posX < 0.5 && yaw < pi/12 && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7) {
			angular = 0.0;
			linear = 0.2;
		}
		else if(posX > 0.4 && yaw < 2*pi && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.5) {
			angular = pi/12;
			linear = 0.0;
		}
		else if(laserRange > 1.0 && !bumperRight && !bumperCenter && !bumperLeft) {
			if(yaw < 17*pi/36 || posX > 0.6) {			
				angular = pi/12;
				linear = 0.1;
			}
			else if(yaw > 19*pi/36 || posX < 0.4) {			
				angular = -pi/12;
				linear = 0.1;
			}
			else {			
				angular = 0;
				linear = 0.1;
			}
		}
		else {
			angular = 0.0;
			linear = 0.0;
		}

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);

        // The last thing to do is to update the timer.
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
