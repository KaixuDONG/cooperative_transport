import math


def limit_angular(input_angle_val):
    # to limit the input angle into (-pi, pi]
    if input_angle_val >= 0:
        output_angle_val = input_angle_val % (2*math.pi)
    else:
        output_angle_val = -((-input_angle_val) % (2*math.pi))

    if output_angle_val >= math.pi:
        output_angle_val = output_angle_val - 2*math.pi
    elif output_angle_val < -math.pi:
        output_angle_val = output_angle_val + 2*math.pi

    return output_angle_val

