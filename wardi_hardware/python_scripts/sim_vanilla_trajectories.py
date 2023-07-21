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


        self.T0 = time.time()
        self.T_lookahead = 0.8


        #Create Publisher for the final Control Input and Instantiate Message 
        self.trajectory_publisher_ = self.create_publisher(Float64MultiArray, '/trajectory_pub', 10)
        self.traj_msg = Float64MultiArray()

        self.state_input_publisher_ = self.create_publisher(Float64MultiArray, 'state_input_log', 10)
        self.state_input_log_msg = Float64MultiArray()


        #Set up QoS Profiles for receiving odometry data
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        self.odometry_subscriber_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.odometry_subscriber_#prevent unused variable warning



        #Create Function @ 20Hz to Calculate Control Input
        timer_period = 0.01# seconds
        # timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.timer_#prevent unused variable warning

    def odom_callback(self, msg):
        # print("AT ODOM CALLBACK")
        (self.yaw, self.roll, self.pitch) = euler_from_quaternion(msg.q)
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = -1 * msg.position[2]
        self.yaw = self.angle_wrapper(self.yaw)

    def timer_callback(self):
        print("In timer callback")
        self.control()

    def control(self):
        try:
            timefromstart = time.time()-self.T0
            print(f"timefromstart: {time.time()-self.T0}")

            T_lookahead = self.T_lookahead
            # reffunc = self.hover_ref_func(timefromstart+T_lookahead, 4)
            # reffunc = self.hover_ref_func2(timefromstart+T_lookahead, 1)
            # reffunc = self.circle_vert_ref_func(timefromstart+T_lookahead)
            # reffunc = self.circle_horz_ref_func(timefromstart+T_lookahead)
            # reffunc = self.fig8_horz_ref_func(timefromstart+T_lookahead)
            # reffunc = self.fig8_vert_ref_func_short(timefromstart+T_lookahead)
            reffunc = self.fig8_vert_ref_func_tall(timefromstart+T_lookahead)
            print(f"reffunc: {reffunc}")

            x_des = reffunc[0][0]
            y_des = reffunc[1][0]
            z_des = reffunc[2][0]
            yaw_des= reffunc[3][0]
            final = [x_des, y_des, z_des, yaw_des]

            # Publish final trajectory
            self.traj_msg.data = final
            self.trajectory_publisher_.publish(self.traj_msg)

            # Log state and input
            self.state_input_log_msg.data = [float(self.x), float(self.y), float(self.z), float(self.yaw), float(x_des), float(y_des), float(z_des), float(yaw_des)]
            self.state_input_publisher_.publish(self.state_input_log_msg)

            print("---------------------------------")

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