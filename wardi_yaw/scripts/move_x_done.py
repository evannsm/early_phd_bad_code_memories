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

        self.old_time = time.time()
        self.elapsed_time = 0
        self.old_error = 0
        self.old_yaw = 0

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
        #Create Function @ 10Hz to Calculate Control Input
        timer_period = 0.02# seconds
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
            # ~~ SET-UP PD+ Backup ~~
            # equilibrium_vector = np.array([[0, 0, 3, 0, 0, 0, 0, 0, 0]]).T
            # error_from_equilibium = self.stateVector - equilibrium_vector
            # error_x = error_from_equilibium[0][0]
            # error_y = error_from_equilibium[1][0]
            # error_z = error_from_equilibium[2][0]

            # error_Vx = error_from_equilibium[3][0]
            # error_Vy = error_from_equilibium[4][0]
            # error_Vz = error_from_equilibium[5][0]
            # throttle, u_roll, u_pitch, u_yaw = self.PD_controller(error_x, error_y, error_z, error_Vx, error_Vy, error_Vz, error_from_equilibium)
            # print(f"xyz_state: {self.stateVector[0:3]}")
            # print(f"xyz_equilibium: {equilibrium_vector[0:3]}")
            # print(f"error_xyz: {error_from_equilibium[0:3]}")

        
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
            t = 0
            T_lookahead = self.T_lookahead
            reffunc = self.hover_ref_func(t+T_lookahead)
            # print(f"reffunc: {reffunc}")
            error = currstate - reffunc
            # print(f"error: {error}")

            new_u = self.input_update_linear_predict(currstate,lastinput,step,t,T_lookahead,reffunc)
            # print(f"new_u: {new_u}")
            force = new_u[0][0]
            throttle = self.get_throttle_command_from_force(force)
            throttle = throttle if throttle < 1.0 else 1.0
            # print(f"throttle: {throttle}")
            new_u_final = np.vstack([throttle, new_u[1:]])
            # print(f"new_u_final: {new_u_final}")

            pitch_rate = new_u_final[1][0]
            roll_rate = new_u_final[2][0]
            yaw_rate = new_u_final[3][0]

            maxrate_abs = float('inf')
            # maxrate_abs = .99 # THIS WORKS TOO IDK

            # print(f"force:{force}")
            # print(f"roll_rate: {roll_rate}")
            roll_rate = min(max(roll_rate, -maxrate_abs), maxrate_abs)
            # print(f"roll_rate: {roll_rate}")
            # print(f"pitch_rate: {pitch_rate}")
            pitch_rate = min(max(pitch_rate, -maxrate_abs), maxrate_abs)
            # print(f"pitch_rate: {pitch_rate}")
            yaw_rate = min(max(yaw_rate, -maxrate_abs), maxrate_abs)

            roll_rate_final = roll_rate
            # pitch_rate_final = roll_rate
            pitch_rate_final = 0.0
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




    def input_update_linear_predict(self, currstate,lastinput,step,t,T_lookahead,reffunc):
        alpha=np.array([[20,30,30,30]]).T
        # alpha = np.array([[20,60,60,60]]).T
        pred = self.getyorai_g_linear_predict(currstate,lastinput,T_lookahead)
        print(f"pred: {pred}")
        print(f"ref: {reffunc}")
        err = reffunc-pred
        print(f"err: {err}")
        weighted_jac_inv = alpha*self.jac_inv
        print(f"weighted_inv: {weighted_jac_inv}")

        udot = weighted_jac_inv @ err
        print(f"udot: {udot}")
        # print(f"update: {step*udot}")
        print(f"update: {0.02*udot}")
        print(f"lastin: {lastinput}")

        # u=lastinput+step*udot
        u=lastinput+0.02*udot

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

    def hover_ref_func(self, t):
        hover1 = np.array([[0, 0, 4, 0]]).T
        hover2 = np.array([[0, 0, 4, np.pi]]).T
        hover3 = np.array([[0, 0, 4, np.pi/2]]).T
        hover4 = np.array([[1, 0, 4, np.pi/2]]).T
        return hover4

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