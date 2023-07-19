#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
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

        self.old_time = time.time()
        self.elapsed_time = 0
        self.old_error = 0
        self.old_yaw = 0

        self.T0 = time.time()

        self.g = 9.806
        self.m = 1.535
        self.grav_comp_mass = 1.35
        self.T_lookahead = 0.8


        self.motor_constant_ = 0.00000584
        self.motor_velocity_armed_ = 100
        self.motor_input_scaling_ = 1000.0

        self.u0 = np.array([[self.hardware_get_throttle_command_from_force(self.grav_comp_mass*self.g), 0, 0, 0]]).T

        #print(f"originalu0: {self.u0}")
        #Create Publisher for the final Control Input and Instantiate Message 
        self.rates_publisher_ = self.create_publisher(Float64MultiArray, 'rates_wardi', 10)
        self.pub_u = Float64MultiArray()

        self.state_input_publisher_ = self.create_publisher(Float64MultiArray, 'state_input_log', 10)
        self.state_input_log_msg = Float64MultiArray()


        #Set up QoS Profiles for receiving odometry data
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        self.odometry_subscriber_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.odometry_subscriber_#prevent unused variable warning


        self.eAT, self.int_eATB, self.int_eAT, self.C = self.linearized_model()
        #print(f"self.int_eATB: {self.int_eATB}")
        self.jac_inv = np.linalg.inv(self.getyorai_gJac_linear_predict2())
        # #print(f"Jac_inv: {Jac_inv}")
        #Create Function @ 20Hz to Calculate Control Input
        timer_period = 0.01# seconds
        # timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.timer_#prevent unused variable warning

    def odom_callback(self, msg):
        # #print("AT ODOM CALLBACK")
        (self.yaw, self.roll, self.pitch) = euler_from_quaternion(msg.q)
        # (self.pitch, self.roll, self.yaw) = euler_from_quaternion(msg.q)
        # #print(f"roll:{self.roll}, self.pitch:{self.pitch}, self.yaw:{self.yaw}")


        self.p = msg.angular_velocity[0]
        self.q = msg.angular_velocity[1]
        self.r = msg.angular_velocity[2]

        self.u = msg.velocity[0]
        self.v = msg.velocity[1]
        self.w = -1 * msg.velocity[2]

        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = -1 * msg.position[2]

        self.yaw = self.angle_wrapper(self.yaw)
        # self.stateVector = np.array([[self.pitch, self.roll, self.yaw, self.p, self.q, self.r, self.u, self.v, self.w, self.x, self.y, self.z]]).T
        # self.stateVector = np.array([[self.x, self.y, self.z, self.u, self.v ,self.w, self.roll, self.pitch, self.angle_wrapper(self.yaw)]]).T
        self.stateVector = np.array([[self.x, self.y, self.z, self.u, self.v ,self.w, self.roll, self.pitch, self.yaw]]).T

        self.input = np.array([[self.p, self.q, self.r]]).T

        # self.stateVector = np.array([[self.w, self.z]]).T
        # #print(self.stateVector.shape)
        # #print(self.stateVector)

    def timer_callback(self):
        #print("In timer callback")
        new_time = time.time()
        self.elapsed_time = new_time - self.old_time
        #print(f"timel: {self.elapsed_time}")
        self.old_time = new_time

        self.control()

    def control(self):
        try:
            # ~~ Set-Up PID Controller ~~


            final = [new_u_final[0][0], pitch_rate_final, roll_rate_final, yaw_rate_final]
            current_input_save = np.array(final).reshape(-1, 1)
            print(f"final control input: {final}")
            self.u0 = current_input_save
            self.pub_u.data = final
            #print('-----------------------------------')
            self.rates_publisher_.publish(self.pub_u)

            # Log state and input
            self.state_input_log_msg.data = [float(self.x), float(self.y), float(self.z), float(self.yaw), float(final[0]), float(final[1]), float(final[2]), float(final[3])]
            self.state_input_publisher_.publish(self.state_input_log_msg)

        except:
            print("no state vector yet")



    def hover_ref_func(self, t, num):
        hover_dict = {
            1: np.array([[0, 0, 4, 0]]).T,
            2: np.array([[0, 0, 4, np.pi]]).T,
            3: np.array([[0, 0, 4, np.pi / 2]]).T,
            4: np.array([[1, 0, 4, np.pi / 2]]).T,
            5: np.array([[0, 1, 4, np.pi / 2]]).T,
            6: np.array([[1, 1, 4, np.pi / 2]]).T,
            7: np.array([[3, 4, 5, np.pi / 2]]).T,
            8: np.array([[1, 1, 3, 0]]).T,
        }
        if num > len(hover_dict):
            print(f"hover1- #{num} not found")
            return np.array([[0, 0, 0, self.yaw]]).T
        
        print(f"hover1- #{num}")
        return hover_dict.get(num)
    
    def hover_ref_func2(self, t, num):
        hover_dict = {
            1: np.array([[1, 0, 4, np.pi / 2]]).T,
            2: np.array([[1, 3, 4, np.pi / 2]]).T,
            3: np.array([[3, 3, 4, np.pi / 2]]).T,
        }
        if num > len(hover_dict):
            print(f"hover2- #{num} not found")
            return np.array([[1, 1, 4, self.yaw]]).T
        
        print(f"hover2- #{num}")
        return hover_dict.get(num)
    
    def circle_vert_ref_func(self, t):
        print("circle_vert_ref_func")
        w=1;
        r = np.array([[np.cos(w*t), 0, np.sin(w*t)+2, np.pi/2]]).T
        return r
    
    def circle_horz_ref_func(self, t):
        print("circle_horz_ref_func")
        w=1;
        r = np.array([[np.cos(w*t), np.sin(w*t), 3, np.pi/2]]).T
        return r
    
    def fig8_horz_ref_func(self, t):
        print("fig8_horz_ref_func")
        r = np.array([[np.sin(t/2), np.sin(2*t/2), 3, np.pi/2]]).T
        return r
    
    def fig8_vert_ref_func_short(self, t):
        print(f"fig8_vert_ref_func_short")
        r = np.array([[np.sin(t/2), 0, np.sin(2*t/2)+3, np.pi/2]]).T
        return r
    
    def fig8_vert_ref_func_tall(self, t):
        print(f"fig8_vert_ref_func_tall")
        r = np.array([[np.sin(2*t/2), 0, np.sin(t/2)+2, np.pi/2]]).T
        return r
     

    def hardware_get_throttle_command_from_force(self, collective_thrust):
        a = 2.821541634028107
        b = 16.149489488767884
        c = 2.525758187432767

        # equation form is a*x + b*sqrt(x) + c = y
        throttle_command = a*collective_thrust + b*sqrt(collective_thrust) + c
        return throttle_command


    def hardware_get_force_from_throttle_command(self, thrust_command):
        a = .0004811579185520395
        b = 0.104616790588235
        c = -1.798382556108580

        # equation form is a*x^2 + b*x + c = y
        collective_thrust = a*thrust_command**2 + b*thrust_command + c
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