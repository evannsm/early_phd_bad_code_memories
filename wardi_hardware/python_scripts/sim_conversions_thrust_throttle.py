#!/usr/bin/env python3
   
motor_constant_ = 0.00000584
motor_velocity_armed_ = 100
motor_input_scaling_ = 1000.0
from math import sqrt

def get_throttle_command_from_force(collective_thrust):
    motor_speed = sqrt(collective_thrust / (4.0 * motor_constant_))
    throttle_command = (motor_speed - motor_velocity_armed_) / motor_input_scaling_
    return throttle_command

def get_force_from_throttle_command(thrust_command):

    motor_speed = (thrust_command * motor_input_scaling_) + motor_velocity_armed_
    collective_thrust = 4.0 * motor_constant_ * motor_speed ** 2
    return collective_thrust


x = get_force_from_throttle_command(1)
print(x)

y = get_throttle_command_from_force(9.806 * 1.35)
print(y)
