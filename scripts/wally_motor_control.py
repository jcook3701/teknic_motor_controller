import numpy as np
import time

# make believe motor class
import teknic_motor_control

class wally_motor_model():
    def __init__(self, time_delay):

        # ----- necessary variables
        # right wheel data
        self.V_right = 0.0 # linear speed of right wheel
        self.W_right = 0.0 # angular speed virtual right wheel
        self.enc_speed_right = 0.0 # encoder speed command
        self.wheel_R_ang_pos = 0.0 # wheel encoder position
        self.wheel_R_ang_vel = 0.0 # wheel encoder velocity

        # left wheel data
        self.V_left = 0.0 # linear speed of left wheel
        self.W_left = 0.0 # angular speed virtual left wheel
        self.enc_speed_left = 0.0 # encoder speed command
        self.wheel_L_ang_pos = 0.0 # wheel encoder position
        self.wheel_L_ang_vel = 0.0 # wheel encoder velocity

        # robot dimension data
        self.robot_width = 34 * 0.0254 # disance between robot left and right wheels, inches converted to meters
        self.wheel_radius = 5.0 * 0.0254 # wheel radius, inches converted to meters
        self.enc_res = 2844 # encoder tics per wheel revolution
        self.tire_deflection = 1.042 # deformation of tire at the point of contact
        self.diameter_mod = 1.0 # need to look into whether this is necessary, don't understand why used by ROSbot

        # time data
        self.time_delay = time_delay # time for looping to make calculations and adjust velocity commands

        # robot odom data
        self.robot_angular_vel = 0.0
        self.robot_angular_pos = 0.0
        self.robot_x_vel = 0.0
        self.robot_y_vel = 0.0
        self.robot_x_pos = 0.0
        self.robot_y_pos = 0.0

    def set_robot_speed(self, linear_vel, angular_vel):

        # left wheel calculations
        self.V_left = linear_vel - (angular_vel * self.robot_width / 2.0) # wheel linear velocity
        self.W_left = self.V_left / self.wheel_radius # wheel angular velocity
        self.enc_speed_left = self.enc_res * self.W_left / (2.0 * np.pi)

        # right wheel calculations
        self.V_right = linear_vel + (angular_vel * self.robot_width / 2.0)   # wheel linear velocity
        self.W_right = self.V_right / self.wheel_radius # wheel angular velocity
        self.enc_speed_right = self.enc_res * self.W_right / (2.0 * np.pi)

        return self.enc_speed_left, self.enc_speed_right

    def update_odometer(self, enc_right, enc_left):

        # virtual left wheel distance calculation in encoders
        wheel_FL_ang_pos = 2.0 * np.pi * enc_left / self.enc_res
        wheel_RL_ang_pos = 2.0 * np.pi * enc_left / self.enc_res
        enc_L = enc_left / self.tire_deflection

        # virtual right wheel distance calculation in encoders
        wheel_FR_ang_pos = 2.0 * np.pi * enc_right / self.enc_res
        wheel_RR_ang_pos = 2.0 * np.pi * enc_right / self.enc_res
        enc_R = enc_right / self.tire_deflection

        # calculating wheel angular velocity
        self.wheel_L_ang_vel = ((2.0 * np.pi * enc_L / self.enc_res) - self.wheel_L_ang_pos) / self.time_delay
        self.wheel_R_ang_vel = ((2.0 * np.pi * enc_R / self.enc_res) - self.wheel_R_ang_pos) / self.time_delay

        # calculating new wheel postion
        self.wheel_L_ang_pos = 2.0 * np.pi * enc_L / self.enc_res
        self.wheel_R_ang_pos = 2.0 * np.pi * enc_R / self.enc_res

        # odom data for storage and querying
        self.robot_angular_vel = (((self.wheel_R_ang_pos - self.wheel_L_ang_pos) * self.wheel_radius / (self.robot_width *  self.diameter_mod)) - self.robot_angular_pos) / self.time_delay
        self.robot_angular_pos = (self.wheel_R_ang_pos - self.wheel_L_ang_pos) * self.wheel_radius / (self.robot_width *  self.diameter_mod)
        self.robot_x_vel = (self.wheel_L_ang_vel * self.wheel_radius + self.robot_angular_vel * self.robot_width / 2) * np.cos(self.robot_angular_pos)
        self.robot_y_vel = (self.wheel_L_ang_vel * self.wheel_radius + self.robot_angular_vel * self.robot_width / 2) * np.sin(self.robot_angular_pos)
        self.robot_x_pos = self.robot_x_pos + self.robot_x_vel * self.time_delay
        self.robot_y_pos = self.robot_y_pos + self.robot_y_vel * self.time_delay



# main loop for example
if __name__ == "__main__":
    
    # initialize wally motor model
    # time we wait in-between velocity commands and odometer updates when in ROS should be built in and asychronous
    time_loop = 0.1
    wally = wally_motor_model(time_loop)
    
    # example motor class, obviously much different
    motor = Teknic_motor_control()

    # decide desired velocities to command
    desired_linear_vel = 1.0 # desired linear velocity in m/s
    desired_angular_vel = 0.5 # desired angular velocity in m/s

    '''
    # example if extracting from twist command in ROS
    
    # message being passed and recieved by motor control node
    wally_commanded_vel = Twist()
    wally_commanded_vel.linear.x = linear_vel
    wally_commanded_vel.angular.z = angular_vel

    # actual extraction of desired velocities
    desired_linear_vel = wally_commanded_vel.linear.x # desired linear velocity in m/s
    desired_angular_vel = wally_commanded_vel.angular.z # desired angular velocity in m/s
    '''

    for i in range(100): # looping through for 10 seconds
        
        # calculating motor encoder velocity
        enc_speed_left, enc_speed_right = wally.set_robot_speed(desired_linear_vel, desired_angular_vel)

        # example motor commands, likely different of course
        motor.left_motor_enc_vel_command(enc_speed_left)
        motor.right_motor_enc_vel_command(enc_speed_right)

        # sleeping for 0.1 seconds
        time.sleep(time_loop)

        # current section needs information from motor itself
        enc_left = motor.extract_left_encoder() # encoder count for left motor
        enc_right = motor.extract_right_encoder() # encoder count for right motor

        # updating odom information
        wally.update_odometer(enc_left, enc_right)

        print('Current x position in NED coordinates: ' + str(wally.robot_x_pos))
        print('Current y position in NED coordinates: ' + str(wally.robot_y_pos))




    









    