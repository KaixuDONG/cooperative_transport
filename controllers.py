# this file mainly includes the controller and relative functions which is used for the flight control
# and the leader-follower formation cooperative transport

import math


def SaturationFunction(original_val, lower_bound, upper_bound):
    if lower_bound > upper_bound:
        raise Exception("the lower bound must below the upper bound")
    if original_val <= lower_bound:
        saturated_val = lower_bound
    elif original_val >= upper_bound:
        saturated_val = upper_bound
    else:
        saturated_val = original_val

    return saturated_val


# basic PID controller for one single channel
class PID_ControllerBasic(object):
    def __init__(self, sample_time, kp, ki, kd, constant_val):
        self.sample_time = sample_time
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.constant_val = constant_val

        self.error_val = 0
        self.integrated_error_val = 0
        self.integration_threshold = 1.50
        self.integration_enable_flag = True
        self.integration_saturation_flg = False
        self.integration_saturation_lower_bound = 0
        self.integration_saturation_upper_bound = 0

        self.desired_val = 0

    def update_reference(self, desired_val):
        self.desired_val = desired_val

    def update_error(self, feedback_val):
        error_previous = self.error_val
        self.error_val = self.desired_val - feedback_val

        # differential term
        dot_error_val = (self.error_val - error_previous)/self.sample_time
        d_term = self.kd * dot_error_val

        # proportional term
        p_term = self.kp * self.error_val

        # integration term
        if self.integration_enable_flag:
            # using integration term only when the error is not large to decrease the overshoot
            if math.fabs(self.error_val) <= self.integration_threshold:
                self.integrated_error_val += (error_previous + self.error_val)*self.sample_time/2.0
                if self.integration_saturation_flg:
                    self.integrated_error_val = SaturationFunction(self.integrated_error_val,
                                                                   self.integration_saturation_lower_bound,
                                                                   self.integration_saturation_upper_bound)
        i_term = self.ki * self.integrated_error_val
        # control output value
        control_output = p_term + i_term + d_term + self.constant_val

        return control_output

    def integration_disable(self):
        self.integration_enable_flag = False

    def integration_saturation(self, lower_bound, upper_bound):
        self.integration_saturation_flg = True
        self.integration_saturation_lower_bound = lower_bound
        self.integration_saturation_upper_bound = upper_bound


class PID_ControllerThreeAixs(object):
    def __init__(self, sample_time,
                 kp_x, ki_x, kd_x, constant_x,
                 kp_y, ki_y, kd_y, constant_y,
                 kp_z, ki_z, kd_z, constant_z):
        self.pid_x = PID_ControllerBasic(sample_time, kp_x, ki_x, kd_x, constant_x)
        self.pid_y = PID_ControllerBasic(sample_time, kp_y, ki_y, kd_y, constant_y)
        self.pid_z = PID_ControllerBasic(sample_time, kp_z, ki_z, kd_z, constant_z)

    def update_reference(self, desired_x, desired_y, desired_z):
        self.pid_x.update_reference(desired_x)
        self.pid_y.update_reference(desired_y)
        self.pid_z.update_reference(desired_z)

    def update_error(self, feedback_x, feedback_y, feedback_z):
        ctrl_out_ux = self.pid_x.update_error(feedback_x)
        ctrl_out_uy = self.pid_y.update_error(feedback_y)
        ctrl_out_uz = self.pid_z.update_error(feedback_z)

        return  ctrl_out_ux, ctrl_out_uy, ctrl_out_uz

    def integration_disable(self):
        self.pid_x.integration_disable()
        self.pid_y.integration_disable()
        self.pid_z.integration_disable()

    def integration_saturation(self, lower_bound_x, upper_bound_x,
                                     lower_bound_y, upper_bound_y,
                                     lower_bound_z, upper_bound_z):
        self.pid_x.integration_saturation(lower_bound_x, upper_bound_x)
        self.pid_y.integration_saturation(lower_bound_y, upper_bound_y)
        self.pid_z.integration_saturation(lower_bound_z, upper_bound_z)











