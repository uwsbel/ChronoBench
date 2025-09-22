import pychrono


system = pychrono.ChSystemSMC()


ground_body = pychrono.ChBodyEasyPlane(10, 10, 1000, True, False)
system.AddBody(ground_body)

viper_rover = pychrono.ChRover()
viper_rover.SetChassisFixed(False)
viper_rover.SetInitPosition(pychrono.ChVectorD(0, 0.5, 0))
viper_rover.SetInitForwardVelocity(pychrono.ChVectorD(0, 0, 0))
system.Add(viper_rover)

driver = pychrono.ChIrrDriver()
driver.SetWindowSize(800, 600)
driver.SetWindowTitle('Viper Rover Simulation')
driver.SetShowInfos(True)
driver.SetShowFPS(True)
driver.SetShowLogo(False)
driver.SetShowTime(True)


system.Set_G_acc(pychrono.ChVectorD(0, -9.81, 0))


for i in range(100):
    
    steering_angle = i / 100.0
    viper_rover.SetSteering(steering_angle)

    
    system.DoStepDynamics(0.01)
    driver.Render()