#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sympy as smp
from scipy.integrate import quad
from scipy.linalg import expm
from sympy import symbols 
import numpy as np

from tf_transformations import euler_from_quaternion
import time

from math import sqrt
import math as m


class WardiController(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.motor_constant_ = 0.00000584
        self.motor_velocity_armed_ = 100
        self.motor_input_scaling_ = 1000.0

        self.u0 = np.array([[self.get_throttle_command_from_force(0), 0, 0, 0]]).T
        # print(f"originalu0: {self.u0}")
        #Create Publisher for the final Control Input and Instantiate Message 
        self.rates_publisher_ = self.create_publisher(Float64MultiArray, 'rates_wardi', 10)
        self.pub_u = Float64MultiArray()


        #Set up QoS Profiles for receiving odometry data
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        self.odometry_subscriber_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.odometry_subscriber_#prevent unused variable warning


  
        timer_period = 0.20# seconds
        # timer_period = 0.02# seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.timer_#prevent unused variable warning


    def odom_callback(self, msg):
        # print("AT ODOM CALLBACK")
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = -1 * msg.position[2]

        self.u = msg.velocity[0]
        self.v = msg.velocity[1]
        self.w = -1 * msg.velocity[2]

        (self.yaw, self.pitch, self.roll) = euler_from_quaternion(msg.q)


        self.stateVector = np.array([[self.x, self.y, self.z, self.u, self.v ,self.w, self.roll, self.pitch, self.yaw]]).T

        # quaternion = np.array([msg.q[2], msg.q[1], msg.q[0], msg.q[3]])
        # self.roll2, self.pitch2, self.yaw2 = euler_from_quaternion(quaternion)
        # self.stateVector = np.array([[self.x, self.y, self.z, self.u, self.v ,self.w, self.roll2, self.pitch2, self.yaw2]]).T

        self.p = msg.angular_velocity[0]
        self.q = msg.angular_velocity[1]
        self.r = msg.angular_velocity[2]
        self.input = np.array([[self.p, self.q, self.r]]).T



    def timer_callback(self):
        print("In timer callback")

        self.control()
  

    def control(self):
        try:
            # ~~ SET-UP PD+ Backup ~~
            equilibrium_vector = np.array([[2, 1, 3, 0, 0, 0, 0, 0, np.pi/2]]).T
            equilibrium_vector = np.array([[0, 0, 3, 0, 0, 0, 0, 0, np.pi/2]]).T
            # print(f"OG_yaw: {self.yaw}")
            if self.yaw < 0:
                self.stateVector[-1] = self.yaw + (2 * np.pi)
            # print(f"NEW_yaw: {self.stateVector[-1]}")
            error_from_equilibium = self.stateVector - equilibrium_vector
            error_x = error_from_equilibium[0][0]
            error_y = error_from_equilibium[1][0]
            error_z = error_from_equilibium[2][0]

            error_Vx = error_from_equilibium[3][0]
            error_Vy = error_from_equilibium[4][0]
            error_Vz = error_from_equilibium[5][0]


            # ~~ PD+ CONTROLLERS ~~
        

            # ~~ Throttle ~~
            # kp_z= 1
            # kd_z = 1.7321
            # print(f"error_z: {error_z}, error_Vz: {error_Vz}")
            # print(f"Vzerr:{error_Vz}")
            # print(f"Vz: {self.w}")
            kp_z = -5.0
            kd_z = -3.5

            # print(f"pos_z: {kp_z * error_z}")
            # print(f"vel_z: {kd_z * error_Vz}")
            u_z = kp_z * error_z + (kd_z) * (error_Vz)
            # print(f"u_z: {u_z}")
            u_z += 9.81
            # print(f"u_z_grav: {u_z}")
            # print(f"thrust: {u_z}")
            throttle = float(self.get_throttle_command_from_force(u_z))
            # print(f"throttle: {throttle}")
            # throttle = float(0.0)


            # ~~ X-Axis ~~
            kp_x = -.018 #-.015
            kd_x = -.2
            k_ang_x = -0.03
            error_roll = error_from_equilibium[6]

            # print(f"pitch: {self.pitch}")
            print(f"roll: {self.roll}")
            print(f"x: {self.x}")
            print(f"Vx: {self.u}")
            print(f"err_x: {error_x}")
            print(f"err_roll: {error_from_equilibium[6]}")
            # print(f"Vx: {self.u}, x: {self.x}, err_roll: {error_from_equilibium[6]}")
            # print(f"errVx: {error_Vx}, errx: {error_x}")
            u_roll = -1*float(kp_x * error_x + kd_x * error_Vx + k_ang_x * error_roll)
            print(f"u_x: {u_roll}")




            # ~~ Y-Axis ~~
            # kp_y = -.03
            # kd_y = -.3
            # k_ang_y = -0.001

            # kp_y = -.035
            # kd_y = -.3
            # k_ang_y = -0.08

            kp_y = -.018 #-.015
            kd_y = -.2
            k_ang_y = -0.03
            error_pitch = error_from_equilibium[7]

            # print(f"pitch: {self.pitch}")
            # print(f"Vy: {self.v}, y: {self.y}, err_pitch: {error_from_equilibium[7]}")
            # print(f"errVy: {error_Vy}, erry: {error_y}")
            u_pitch = -1*float(kp_y * error_y + kd_y * error_Vy + k_ang_y * error_pitch)
            print(f"u_y: {u_pitch}")



            # -- Yaw --
            kp_yaw = -.2
            # print(f"yaw:{self.stateVector[8]}, des_yaw: {equilibrium_vector[8]}")
            error_yaw = error_from_equilibium[8]
            # print(f"error_yaw: {error_yaw}")
            # error_yaw = self.yaw - desired_yaw GET FROM VECTOR
            u_yaw = -1*float(kp_yaw* error_yaw)
            print(f"u_yaw: {u_yaw}")


            print(f"xyz_state: {self.stateVector[0:3]}")
            print(f"yaw: {self.yaw}")
            print(f"xyz_equilibium: {equilibrium_vector[0:3]}")
            print(f"yaw_equilibium: {equilibrium_vector[8]}")
            print(f"error_xyz: {error_from_equilibium[0:3]}")
            print(f"error_yaw: {error_from_equilibium[8]}")

        
            final = [throttle, u_roll, u_pitch, u_yaw]
            print(f"final: {final}")
            # current_input_save = np.array(final).reshape(-1, 1)
            # self.u0 = current_input_save
            self.pub_u.data = final
            print('-----------------------------------')
            self.rates_publisher_.publish(self.pub_u)

        except:
            print("no state vector yet")


    
    def PD_controller(self, error_x, error_y, error_z, error_Vx, error_Vy, error_Vz, error_from_equilibium):
            # ~~ PD+ CONTROLLERS ~~
        

            # ~~ Throttle ~~
            # kp_z= 1
            # kd_z = 1.7321
            # print(f"error_z: {error_z}, error_Vz: {error_Vz}")
            # print(f"Vzerr:{error_Vz}")
            # print(f"Vz: {self.w}")
            kp_z = -5.0
            kd_z = -3.5

            # print(f"pos_z: {kp_z * error_z}")
            # print(f"vel_z: {kd_z * error_Vz}")
            u_z = kp_z * error_z + (kd_z) * (error_Vz)
            # print(f"u_z: {u_z}")
            u_z += 9.81
            # print(f"u_z_grav: {u_z}")
            # print(f"thrust: {u_z}")
            throttle = float(self.get_throttle_command_from_force(u_z))
            print(f"throttle: {throttle}")
            # throttle = float(0.0)


            # ~~ X-Axis ~~
            kp_x = -.018 #-.015
            kd_x = -.2
            k_ang_x = -0.03
            error_roll = error_from_equilibium[6]

            # print(f"pitch: {self.pitch}")
            print(f"roll: {self.roll}")
            print(f"Vx: {self.u}, x: {self.x}, err_roll: {error_from_equilibium[6]}")
            print(f"errVx: {error_Vx}, errx: {error_x}")
            u_roll = float(kp_x * error_x + kd_x * error_Vx + k_ang_x * error_roll)
            print(f"u_x: {u_roll}")




            # ~~ Y-Axis ~~
            # kp_y = -.03
            # kd_y = -.3
            # k_ang_y = -0.001

            kp_y = -.035
            kd_y = -.3
            k_ang_y = -0.08
            error_pitch = error_from_equilibium[7]

            # print(f"pitch: {self.pitch}")
            # print(f"Vy: {self.v}, y: {self.y}, err_pitch: {error_from_equilibium[7]}")
            # print(f"errVy: {error_Vy}, erry: {error_y}")
            u_pitch = float(kp_y * error_y + kd_y * error_Vy + k_ang_y * error_pitch)
            # print(f"u_y: {u_pitch}")



            # -- Yaw --
            kp_yaw = -.2
            # print(f"yaw:{self.stateVector[8]}, des_yaw: {equilibrium_vector[8]}")
            error_yaw = error_from_equilibium[8]
            # print(f"error_yaw: {error_yaw}")
            # error_yaw = self.yaw - desired_yaw GET FROM VECTOR
            u_yaw = float(kp_yaw* error_yaw)

            return throttle, u_roll, u_pitch, u_yaw
  
            
    def get_throttle_command_from_force(self, collective_thrust):
        motor_speed = sqrt(collective_thrust / (4.0 * self.motor_constant_))
        throttle_command = (motor_speed - self.motor_velocity_armed_) / self.motor_input_scaling_
        return throttle_command
    
    def get_force_from_throttle_command(self, thrust_command):

        motor_speed = (thrust_command * self.motor_input_scaling_) + self.motor_velocity_armed_
        collective_thrust = 4.0 * self.motor_constant_ * motor_speed ** 2
        return collective_thrust
    
    def angle_wrapper(self, ang):
        if ang < 0:
            # print('neg ang')
            ang += 2*m.pi
        return ang




def main(args=None):
    rclpy.init(args=args)

    wardi_controller = WardiController()

    rclpy.spin(wardi_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wardi_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()