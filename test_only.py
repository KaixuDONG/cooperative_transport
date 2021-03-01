import controllers
import math
import wheel_functions
# print(controllers.SaturationFunction(12.6, 15, 100))
#
#
# kk = controllers.PID_ControllerThreeAixs(1000, 1, 1, 1, 1,
#                                          1, 1, 1, 1,
#                                          1, 1, 1, 10)
#
# print(kk.pid_x.kp)
# print(kk.pid_x.integration_saturation_flg)


print(wheel_functions.limit_angular(0*math.pi)/math.pi)

# aa = ((-0.2*math.pi) % (2*math.pi)) / math.pi
# print(aa)