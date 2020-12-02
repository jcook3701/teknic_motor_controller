/**
   @file    differential_drive.h
   @author  jcook3701
   @brief   
   
   This module implements a ...
*/

#ifndef __DIFFERENTIAL_DRIVE_H__
#define __DIFFERENTIAL_DRIVE_H__

#include <geometry_msgs/Twist.h>

class differential_drive {
 private:
    // Configuration Variables
    int number_of_motors_;
    std::vector<int> gear_ratio_;
    double wheel_base_;
    double wheel_radius_;

 public:
    // Constructor & Destructor
    differential_drive();
    ~differential_drive();
    
    double unicycle_differential_drive(geometry_msgs::Twist& msg);

    // differential drive  getters and setters
    void set_number_of_motors(int _number_of_motors);
    void set_gear_ratio(std::vector<int> _gear_ratio);
    void set_wheel_base(double _wheel_base);
    void set_wheel_radius(double _wheel_radius);

    int get_number_of_motors();
    std::vector<int> get_gear_ratio();
    double get_wheel_base();
    double get_wheel_radius();
    
    
}
#endif
