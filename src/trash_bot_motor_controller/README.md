# Trash Bot Motor Controller

## Design
The current design is to take the output from either move_base or key_teleop (cmd_vel - geometry_msg/twist) and convert that into motor velocities for the left and right side motors.  

__Note:__ When we use move_base (nav_ros - TrajectoryPlannerROS) we will have to make sure that the linear velocity and acceleration parameters (min and max) correlate correctly with the motor angular velocity and acceleration parameters(max).  
