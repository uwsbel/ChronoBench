import pychrono


system = pychrono.ChSystemSMC()


ground_body = pychrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False)
ground_body.SetPos(pychrono.ChVectorD(0, -0.1, 0))
system.AddBody(ground_body)

viper_rover = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
viper_rover.SetPos(pychrono.ChVectorD(0, 0.1, 0))
system.AddBody(viper_rover)


viper_rover.SetPos_x(0)
viper_rover.SetPos_y(0.1)
viper_rover.SetPos_z(0)