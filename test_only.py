import controllers

print(controllers.SaturationFunction(12.6, 15, 100))


kk = controllers.PID_ControllerThreeAixs(1000, 1, 1, 1, 1,
                                         1, 1, 1, 1,
                                         1, 1, 1, 1)

print(kk.pid_x.kp)
print(kk.pid_x.integration_saturation_flg)
