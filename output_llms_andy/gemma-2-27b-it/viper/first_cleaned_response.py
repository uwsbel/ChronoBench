import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math







system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


rover = chrono.ChBody()



driver = chrono.ChBody()








vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChCamera(chrono.ChVectorD(0, 5, -10)))
vis.GetCamera().SetPolarCoord(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


vis.AddLight(chrono.ChLight(chrono.ChVectorD(5, 5, 5), chrono.ChColor(1, 1, 1)))







simulation_time = 10  
timestep = 0.01  

steering_angle_start = 0  
steering_angle_end = math.radians(45)  
steering_change_time = 5  

for i in range(int(simulation_time / timestep)):
    time = i * timestep

    
    if time <= steering_change_time:
        steering_angle = steering_angle_start + (steering_angle_end - steering_angle_start) * time / steering_change_time
    else:
        steering_angle = steering_angle_end

    

    system.DoStepDynamics(timestep)
    vis.Render()

vis.Deinitialize()