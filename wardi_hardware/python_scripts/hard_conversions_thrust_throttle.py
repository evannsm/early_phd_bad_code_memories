from math import sqrt


def hardware_get_throttle_command_from_force(collective_thrust):
    # a = 0.02767085
    # b = 1.59928876
    # c = 2.52575818
    a = 2.821541634028107
    b = 16.149489488767884
    c = 2.525758187432767



    # equation form is a*x + b*sqrt(x) + c = y
    throttle_command = a*collective_thrust + b*sqrt(collective_thrust) + c
    return throttle_command


def hardware_get_force_from_throttle_command(thrust_command):
    # a =    0.04906270
    # b =   10.66756302
    # c = -183.37744020
    a = .0004811579185520395
    b = 0.104616790588235
    c = -1.798382556108580

    # equation form is a*x^2 + b*x + c = y
    collective_thrust = a*thrust_command**2 + b*thrust_command + c
    return collective_thrust



throttle = .95
throttle_pct = throttle * 100.0
y = hardware_get_force_from_throttle_command(throttle_pct)
print(f"throttle->force(N): {throttle}->{y}")
print(f"mass carried at throttle of {throttle_pct}%: {y/9.806}kg")

print("")

newtons_needed = y
x = hardware_get_throttle_command_from_force(newtons_needed)
print(f"force(N)->throttle(%): {newtons_needed}->{x}%")