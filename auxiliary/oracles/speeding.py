import math


def calculate_speed(linear_velocity):
    '''
    Calculate the speed from linear velocity
    '''
    x = linear_velocity.x
    y = linear_velocity.y
    return math.sqrt(x**2 + y**2)
