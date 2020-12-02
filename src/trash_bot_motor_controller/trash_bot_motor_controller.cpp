//Required include files
#include "ros/ros.h"
#include "ros/console.h"
#include <geometry_msgs/Twist.h>

// #include <stdio.h>
// #include <iostream>
// #include "pubSysCls.h"

#include "differential_drive.h"

using namespace sFnd;	


void cmdVelCallback(const geometry_msgs::Twist& msg)
{
    ROS_INFO_STREAM("Subscriber velocities:" << " linear=(" << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z <<") angular=("<< msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << ")");
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "trash_bot_motor_controller");
    ros::NodeHandle nh("~");
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmdVelCallback);

    std::string cmd_vel;
    
    double max_motor_velocity;
    double max_motor_acceleration;
    double max_motor_torque;
    
    int number_of_motors;  
    std::vector<int> gear_ratio;
    double wheel_base;
    double wheel_radius;
    

    
    if(!nh.getParam("cmd_vel", cmd_vel)) {
	nh.setParam("cmd_vel", "cmd_vel");
	nh.getParam("cmd_vel", cmd_vel);
    }
    
    if(!nh.getParam("motor_parameters/max_motor_velocity", max_motor_velocity)) {
	nh.setParam("motor_parameters/max_motor_velocity", 0);
	nh.getParam("motor_parameters/max_motor_velocity", max_motor_velocity);
    }
    
    if(!nh.getParam("motor_parameters/max_motor_acceleration", max_motor_acceleration)) {
	nh.setParam("motor_parameters/max_motor_acceleration", 0);
	nh.getParam("motor_parameters/max_motor_acceleration", max_motor_acceleration);
    }
    
    if(!nh.getParam("motor_parameters/max_motor_torque_percentage", max_motor_torque)) {
	nh.setParam("motor_parameters/max_motor_torque_percentage", 0);
	nh.getParam("motor_parameters/max_motor_torque_percentage", max_motor_torque);
    }
    
    if(!nh.getParam("vehicle_parameters/number_of_motors", number_of_motors)) {
	nh.setParam("vehicle_parameters/number_of_motors", 2);
	nh.getParam("vehicle_parameters/number_of_motors", number_of_motors);
    }  
    
    if(!nh.getParam("vehicle_parameters/gear_ratio", gear_ratio)) {
	nh.setParam("vehicle_parameters/gear_ratio", 0);
	nh.getParam("vehicle_parameters/gear_ratio",  gear_ratio);
    }
    
    if(!nh.getParam("vehicle_parameters/wheel_base", wheel_base)) {
	nh.setParam("vehicle_parameters/wheel_base", 0);
	nh.getParam("vehicle_parameters/wheel_base",  wheel_base);
    }
    
    if(!nh.getParam("vehicle_parameters/wheel_radius", wheel_radius)) {
	nh.setParam("vehicle_parameters/wheel_radius", 0);
	nh.getParam("vehicle_parameters/wheel_radius",  wheel_radius);
    }
    
    
    ROS_INFO_STREAM("max_motor_velocity: " << max_motor_velocity);
    ROS_INFO_STREAM("max_motor_acceleration: " << max_motor_acceleration);
    ROS_INFO_STREAM("max_motor_torque: " << max_motor_torque);
    
    ROS_INFO_STREAM("number_of_motors: " << number_of_motors);
    ROS_INFO_STREAM("gear_ratio: " << gear_ratio[0] << ":" << gear_ratio[1]);
    ROS_INFO_STREAM("wheel_base: " << wheel_base);
    ROS_INFO_STREAM("wheel_radius: " << wheel_radius);
    
    ros::spin();
    
    return 0; //End program
}
//


