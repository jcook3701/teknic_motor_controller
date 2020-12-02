#include "differential_drive.h"

#include <math.h>

/*-------------------------------------------------------------------------*/
/**
   @brief    differential_drive constructor
   @param    
   @return   differential_drive
*/
/*--------------------------------------------------------------------------*/
differential_drive::differential_drive(time_delay){

    // Configuration Variables
    this->number_of_motors;
    this->gear_ratio;
    this->wheel_base;
    this->wheel_radius;

    // ----- necessary variables
    // right wheel data
    this->V_right = 0.0; // linear speed of right wheel
    this->W_right = 0.0; // angular speed virtual right wheel
    this->enc_speed_right = 0.0; // encoder speed command
    this->wheel_R_ang_pos = 0.0; // wheel encoder position
    this->wheel_R_ang_vel = 0.0; // wheel encoder velocity

    // left wheel data
    this->V_left = 0.0; // linear speed of left wheel
    this->W_left = 0.0; // angular speed virtual left wheel
    this->enc_speed_left = 0.0; // encoder speed command
    this->wheel_L_ang_pos = 0.0; // wheel encoder position
    this->wheel_L_ang_vel = 0.0; // wheel encoder velocity

    // robot dimension data
    // this->wheel_base; // disance between robot left and right wheels, inches converted to meters
    // this->wheel_radius = 5.0 * 0.0254; // wheel radius, inches converted to meters
    this->enc_res = 2844; // encoder tics per wheel revolution
    this->tire_deflection = 1.042; // deformation of tire at the point of contact
    this->diameter_mod = 1.0; // need to look into whether this is necessary, don't understand why used by ROSbot

    // time data
    this->time_delay = time_delay; // time for looping to make calculations and adjust velocity commands

    // robot odom data
    this->robot_angular_vel = 0.0;
    this->robot_angular_pos = 0.0;
    this->robot_x_vel = 0.0;
    this->robot_y_vel = 0.0;
    this->robot_x_pos = 0.0;
    this->robot_y_pos = 0.0;
}


/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
differential_drive::~differential_drive(){}


/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    msg   geometry_msgs::Twist
   @return   struct<vr, vl>
   
   This function returns the number of sections found in a dictionary.
   The test to recognize sections is done on the string stored in the
   dictionary: a section name is given as "section" whereas a key is
   stored as "section:key", thus the test looks for entries that do not
   contain a colon.
   
   This clearly fails in the case a section name contains a colon, but
   this should simply be avoided.
   
   This function returns -1 in case of error.
*/
/*--------------------------------------------------------------------------*/
double differential_drive::unicyle_differential_drive(geometry_msgs::Twist& msg){

    // speed: si unit m/s

    float64 dx = msg.linear.x;
    float dy = msg.linear.y;
    float dr = msg.angular.z;

	
    
    // angular velocity
    msg.angular.z;

    /*
      vr = (2v + wL)/2R
      vl = (2v - wL)/2R
    */
    
  
}


// differential drive getters and setters

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
void differential_drive::set_number_of_motors(int _number_of_motors){
    this->number_of_motors = _number_of_motors; 
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
void differential_drive::set_gear_ratio(std::vector<int> _gear_ratio){
    this->gear_ratio = _gear_ratio;
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
void differential_drive::set_wheel_base(doulbe _wheel_base){
    this->wheel_base = _wheel_base; 
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
void differential_drive::set_wheel_radius(double _wheel_radius){
    this->wheel_radius = _wheel_radius; 
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
int differential_drive::get_number_of_motors(){
    return this->number_of_motors;
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
std::vector<int> differential_drive::get_gear_ratio(){
    return this->gear_ratio;
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
double differential_drive::get_wheel_base(){
    return this->wheel_base;
}

/*-------------------------------------------------------------------------*/
/**
   @brief    
   @param    
   @return   
*/
/*--------------------------------------------------------------------------*/
double differential_drive::get_wheel_radius(){
    return this->wheel_radius;
}
