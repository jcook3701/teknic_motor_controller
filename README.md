# Teknic Motor Controller

## Design
The current design is to take the output from either move_base or key_teleop (cmd_vel - geometry_msg/twist) and convert that into motor velocities for the left and right side motors.  

__Note:__ When we use move_base (nav_ros - TrajectoryPlannerROS) we will have to make sure that the linear velocity and acceleration parameters (min and max) correlate correctly with the motor angular velocity and acceleration parameters(max).  

## Setup
1. Create ROS Workspace.  
```
$ mdir -p ~/Documents/ros_workspace/trash_bot_workspace/src  
```

2. Move to workspace.  
```
$ cd ~/Documents/ros_workspace/trash_bot_workspace/src  
```

3. Clone this repository into newly created Trash Bot Workspace.  
```
$ git clone git@github.com:jcook3701/teknic_motor_controller.git  
```

4. Move to the head of the project workspace and build the ROS Teknic Motor Controller.  
```
$ cd .. && catkin_make  
```

## Utilization
1. Source the environment file from the head of the project workspace.  
```
$ source devel/setup.bash  
```

2. Use roslaunch to start the motor controller.  
```
$ roslaunch teknic_motor_controller trash_bot_motor_controller.launch  
```

## Configuration Files Breakdown

__Note:__  The following assumptions are being made when using this motor controller.  
1. Each motor is driving two wheels.  Each wheel has its own chain that connects it to either the left or the right motor.  
2. Motor orientation is the following:
```
  (L)  (R)  
---[]  []---  
```
3. There are exactly four wheels.  This could be modified in the future with an additional parameter to the vehicle configuration file.  

__Vehicle Parameters:__  
1. Number of Motors
   - __Description:__ Number of motors being used on vehicle.  
2. Gear Ratio
   - __Description:__ Gear ratio between motor and single tire rotation.  

__Motor Parameters:__  
1. Max Motor Velocity
   - __Description:__ Max velocity that the motor may reach during operations.  
2. Max Motor Acceleration
   - __Description:__ Max acceleration that the motor may reach when ramping to the specified velocity.  
3. Max Motor Torque Percentage
   - __Description:__ Max percentage of available torque that the motor may use when operating.  
