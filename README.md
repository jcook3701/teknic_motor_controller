# Teknic Motor Controller

## Design
1. The current design is to take the output from either move_base or key_teleop (cmd_vel - geometry_msg/twist) and convert that into motor velocities for the left and right side motors.  

__Note:__ When we use move_base (nav_ros - TrajectoryPlannerROS) we will have to make sure that the linear velocity and acceleration parameters (min and max) correlate correctly with the motor angular velocity and acceleration parameters(max).  

## Configuration Files

__Note:__  The following assumptions are being made when using this motor controller.  
1. when using only two motors chains are hooked up that the vehicle has two chains per motor.  
2. Motor orientation is the following: (L)  (R)  
                                     ---[]  []---  
__Vehicle Parameters:__  
1. Number of Motors
   - Number of motors being used on your vehicle.  
2. Gear Ratio
   - Gear ratio between motor and single tire rotation.  
4. max velocity
   - Max velocity that the motor may reach.  
5. max acceleration
   - Max acceleration that the motor may reach.  
6. max torque
   - Max torque that the motor may reach.  
