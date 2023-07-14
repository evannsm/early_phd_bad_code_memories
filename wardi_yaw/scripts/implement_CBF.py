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
        self.T_lookahead = 0.8


        self.motor_constant_ = 0.00000584
        self.motor_velocity_armed_ = 100
        self.motor_input_scaling_ = 1000.0

        self.u0 = np.array([[self.get_throttle_command_from_force(self.m*self.g), 0, 0, 0]]).T
        print(f"originalu0: {self.u0}")
        #Create Publisher for the final Control Input and Instantiate Message 
        self.rates_publisher_ = self.create_publisher(Float64MultiArray, 'rates_wardi', 10)
        self.pub_u = Float64MultiArray()

        self.xyzyaw_publisher_ = self.create_publisher(Float64MultiArray, 'xyzyaw_wardi', 10)
        self.pub_xyzyaw = Float64MultiArray()


        #Set up QoS Profiles for receiving odometry data
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        self.odometry_subscriber_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.odometry_subscriber_#prevent unused variable warning


        self.eAT, self.int_eATB, self.int_eAT, self.C = self.linearized_model()
        print(f"self.int_eATB: {self.int_eATB}")
        self.jac_inv = np.linalg.inv(self.getyorai_gJac_linear_predict2())
        # print(f"Jac_inv: {Jac_inv}")
        #Create Function @ 20Hz to Calculate Control Input
        timer_period = 0.01# seconds
        # timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.timer_#prevent unused variable warning


    def odom_callback(self, msg):
        # print("AT ODOM CALLBACK")
        (self.yaw, self.roll, self.pitch) = euler_from_quaternion(msg.q)
        # (self.pitch, self.roll, self.yaw) = euler_from_quaternion(msg.q)
        # print(f"roll:{self.roll}, self.pitch:{self.pitch}, self.yaw:{self.yaw}")


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
        # print(self.stateVector.shape)
        # print(self.stateVector)


    def timer_callback(self):
        print("In timer callback")
        new_time = time.time()
        self.elapsed_time = new_time - self.old_time
        print(f"timel: {self.elapsed_time}")
        self.old_time = new_time

        self.control()
  
        # self.pub_u.data = list(np.squeeze(self.u0, 1))
        # print('--------------------')
        # print(f"FINAL PUB: {self.pub_u.data}")
        # print('--------------------')
        # self.rates_publisher_.publish(self.pub_u)

    def control(self):
        try:
            self.pub_xyzyaw.data = [float(self.x), float(self.y), float(self.z), float(self.yaw)]
            self.xyzyaw_publisher_.publish(self.pub_xyzyaw)
        
            # ~~ Set-Up Wardi Controller ~~
            currstate = np.array([[self.x, self.y, self.z, self.yaw]]).T
            print(f"currstate: {currstate}")
            throttle = self.u0[0]
            # print(f"throttle: {throttle}")
            force = self.get_force_from_throttle_command(self.u0[0][0])
            # print(f"force: {force}")
            # print(f"u0_1:end : {self.u0[1:]}")
            lastinput = np.vstack([force, self.u0[1:]])
            # print(f"lastinput: {lastinput}")
            step = self.elapsed_time


            # print(f"T0: {self.T0}")
            # print(f"time: {time.time()}")
            T_lookahead = self.T_lookahead
            timefromstart = time.time()-self.T0
            print(f"timefromstart: {time.time()-self.T0}")

            # reffunc = self.hover_ref_func(timefromstart+T_lookahead, 4)
            # reffunc = self.hover_ref_func2(timefromstart+T_lookahead, 1)
            reffunc = self.circle_vert_ref_func(timefromstart+T_lookahead)
            # reffunc = self.circle_horz_ref_func(timefromstart+T_lookahead)
            # reffunc = self.fig8_horz_ref_func(timefromstart+T_lookahead)
            # reffunc = self.fig8_vert_ref_func_short(timefromstart+T_lookahead)
            # reffunc = self.fig8_vert_ref_func_tall(timefromstart+T_lookahead)
            print(f"reffunc: {reffunc}")

            # print(f"roll: {self.roll}")
            # print(f"pitch: {self.pitch}")
            # print(f"yaw: {self.yaw}")

            # error = currstate - reffunc
            # print(f"error: {error}")

            new_u = self.input_update_linear_predict(currstate, lastinput, T_lookahead, reffunc)
            # print(f"new_u: {new_u}")
            force = new_u[0][0]
            throttle = self.get_throttle_command_from_force(force)
            # throttle = throttle if throttle < 1.0 else 1.0
            print(f"throttle: {throttle}")
            new_u_final = np.vstack([throttle, new_u[1:]])
            # print(f"new_u_final: {new_u_final}")

            pitch_rate = new_u_final[1][0]
            roll_rate = new_u_final[2][0]
            yaw_rate = new_u_final[3][0]

            # maxrate_abs = float('inf')
            maxrate_abs = .80 # THIS WORKS TOO IDK



            # print(f"force:{force}")
            # print(f"roll_rate: {roll_rate}")
            roll_rate = min(max(roll_rate, -maxrate_abs), maxrate_abs)
            # print(f"roll_rate: {roll_rate}")
            # print(f"pitch_rate: {pitch_rate}")
            pitch_rate = min(max(pitch_rate, -maxrate_abs), maxrate_abs)
            # print(f"pitch_rate: {pitch_rate}")
            yaw_rate = min(max(yaw_rate, -maxrate_abs), maxrate_abs)

            roll_rate_final = roll_rate
            # roll_rate_final = 0.0
            pitch_rate_final = -pitch_rate
            yaw_rate_final = -yaw_rate

            final = [new_u_final[0][0], pitch_rate_final, roll_rate_final, yaw_rate_final]
            current_input_save = np.array(final).reshape(-1, 1)
            print(f"final: {final}")
            self.u0 = current_input_save
            self.pub_u.data = final
            print('-----------------------------------')
            self.rates_publisher_.publish(self.pub_u)

        except:
            print("no state vector yet")




    def input_update_linear_predict(self, currstate, lastinput, T_lookahead, reffunc):

        pred = self.getyorai_g_linear_predict(currstate,lastinput,T_lookahead)
        err = reffunc-pred
        dgdu_inv = self.jac_inv
        NR = dgdu_inv @ err

        # Set up CBF parameters
        curr_thrust = lastinput[0][0]
        curr_roll_rate = lastinput[1][0]
        curr_pitch_rate = lastinput[2][0]
        curr_yaw_rate = lastinput[3][0]

        phi_thrust = NR[0][0]
        phi_roll_rate = NR[1][0]
        phi_pitch_rate = NR[2][0]
        phi_yaw_rate = NR[3][0]

        # CBF for Thrust
        v_thrust = 0.0
        gamma = 1.0
        thrust_max = 28.0
        thrust_min = 0.0

        if curr_thrust >= 0:
            zeta = gamma * (thrust_max - curr_thrust) - phi_thrust
            if zeta < 0:
                v_thrust = zeta

        if curr_thrust < 0:
            zeta = -gamma * (-thrust_min + curr_thrust) - phi_thrust
            if zeta > 0:
                v_thrust = zeta

        
        phi = NR
        v = np.array([[v_thrust, 0, 0, 0]]).T

        udot1 = phi + v
        print(f"phi: {phi}")
        print(f"v: {v}")
        print(f"udot1: {udot1}")






        alpha=np.array([[20,30,30,30]]).T# PLAY WITH UNIFORM ALPHA 
        u = lastinput + alpha * udot1 * 0.02
        # udot = alpha * NR
        # u=lastinput+0.02*udot

        print(f"u: {u}")
        return u

    def getyorai_g_linear_predict(self, currstate,currinput,T):
        gravity = np.array([[self.m*self.g,0,0,0]]).T
        lin_pred = self.eAT@self.stateVector + self.int_eATB @ (currinput - gravity)
        yorai_g = self.C @ lin_pred
        return yorai_g

    def getyorai_gJac_linear_predict(self, currstate,lastinput,T_lookahead):
        Jac = np.concatenate((self.int_eATB[0:3, :], self.int_eATB[-1:, :]), axis=0)
        return Jac
    
    def getyorai_gJac_linear_predict2(self):
        Jac = np.concatenate((self.int_eATB[0:3, :], self.int_eATB[-1:, :]), axis=0)
        return Jac

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
    

    def linearized_model(self):
        g = self.g
        m = self.m
        T_lookahead = self.T_lookahead

        A = smp.Matrix(
            [
                [0, 0, 0,    1, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 1, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 1,     0,   0, 0],

                [0, 0, 0,    0, 0, 0,     0,-1*g, 0],
                [0, 0, 0,    0, 0, 0,     g,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],

                [0, 0, 0,    0, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],

            ]
            )
        # print(f"A:\n {A}")
        
        eAT = smp.exp(A*T_lookahead)
        # print(f"eAT: \n {eAT}")


        B = smp.Matrix(
            [
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],

                [1/m, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
            )
        # print(f"B: \n{B}")
        # print(f"B_shape:{B.shape}")
        
        int_eAT = np.zeros_like(A)
        # print(int_eAT.shape[0])
        rowcol = int_eAT.shape[0]
        for row in range(rowcol):
            for column in range(rowcol):
                f = lambda x: expm(A*(T_lookahead-x))[row,column]
                int_eAT[row,column] = quad(f, 0, T_lookahead)[0]
        
        
        int_eATB = int_eAT @ B

        eAT = np.array(eAT).astype(np.float64)
        int_eATB = np.array(int_eATB).astype(np.float64)
        # print(f"eAT: {eAT}")
        # print(f"eATb: {int_eATB}")

        C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1]])

        return eAT, int_eATB, int_eAT, C
    

         
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