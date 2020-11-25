//Required include files
#include "ros/ros.h"
#include "ros/console.h"
#include <geometry_msgs/Twist.h>

#include <stdio.h>	
#include <iostream>
#include "pubSysCls.h"

using namespace sFnd;	


void cmdVelCallback(const geometry_msgs::Twist& msg)
{
  if (msg.angular.z > 0){
    ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
  }
}


int main(int argc, char* argv[]){
  ros::init(argc, argv, "trash_bot_motor_controller");
  ros::NodeHandle nh("~");
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmdVelCallback);

  int max_motor_velocity;
  int max_motor_acceleration;
  int max_motor_torque;

  std::vector<int> gear_ratio;
  int number_of_motors;
  
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


  
  ROS_INFO_STREAM("max_motor_velocity: " << max_motor_velocity);
  ROS_INFO_STREAM("max_motor_acceleration: " << max_motor_acceleration);
  ROS_INFO_STREAM("max_motor_torque: " << max_motor_torque);

  ROS_INFO_STREAM("number_of_motors: " << number_of_motors);
  ROS_INFO_STREAM("gear_ratio: " << gear_ratio[0] << ":" << gear_ratio[1]);
  
  ros::spin();
  
  return 0; //End program
}
//
