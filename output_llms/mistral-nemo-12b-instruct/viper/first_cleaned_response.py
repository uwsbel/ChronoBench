import pychrono as chrono
import numpy as np


chrono.setChronoDataPath("path/to/chrono_data")  
chrono.setChronoLogLevel(chrono.LOG_LEVEL_WARNING)


sys = chrono.ChSystemNSC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, 1000, 1000)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


rover = chrono.ChBody()
rover.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(rover)




vis = chrono.ChVisualSystemIrrlicht()
vis.Attachsys(sys)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChCamera(), chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChLightDirectional(), chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, -1))


t = 0
steering_angle = 0
dt = 0.01
total_time = 5

while t < total_time:
    
    steering_angle += 0.1
    if steering_angle > 1:
        steering_angle = 1

    
    sys.DoStepDt(dt)

    
    vis.Render()
    vis.WaitForFrameEnd()

    t += dt