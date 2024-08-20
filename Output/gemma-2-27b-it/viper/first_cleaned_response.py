import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math








system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)








rover = chrono.ChWheeledVehicle("viper_rover.obj")


rover.SetPos(chrono.ChVectorD(0, 1, 0))
rover.SetRot(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1)))


system.Add(rover)

















vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Viper Rover Simulation")


camera = vis.GetCamera()
camera.SetLocation(chrono.ChVectorD(5, 5, 3))
camera.SetLookAt(chrono.ChVectorD(0, 1, 0))
vis.AddLight(chrono.ChLightPoint(chrono.ChVectorD(5, 5, 5)))








timestep = 0.01
simulation_time = 10


initial_steering = 0
final_steering = math.pi / 4  
steering_change_time = 5


for i in range(int(simulation_time / timestep)):
    time = i * timestep

    
    if time <= steering_change_time:
        steering_angle = initial_steering + (final_steering - initial_steering) * time / steering_change_time
    else:
        steering_angle = final_steering

    
    rover.SetSteeringAngle(steering_angle)

    
    system.DoStepDynamics(timestep)

    
    vis.Render()


vis.GetDevice().close()