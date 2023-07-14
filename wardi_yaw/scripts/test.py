#!/usr/bin/env python3
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
import numpy as np

roll = 0.0
pitch = 0.0
yaw = 1.5949279739570763

quaternion = quaternion_from_euler(roll, pitch, yaw)
print(quaternion)

quaternion = np.array([0.716543972492218, 0.0026474478654563427, 0.006124007981270552, 0.6975100040435791])
quaternion2 = np.array([0.006124007981270552, 0.0026474478654563427, 0.716543972492218, 0.6975100040435791])
quaternion3 = np.array([0.6975100040435791, 0.006124007981270552, 0.0026474478654563427, 0.716543972492218])
euler = euler_from_quaternion(quaternion)
print(euler)
euler = euler_from_quaternion(quaternion2)
print(euler)
euler = euler_from_quaternion(quaternion3)
print(euler)

print('hi')