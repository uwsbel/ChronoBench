import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


viper = veh.Viper()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
viper.SetTireType(veh.TireModelType_RIGID)
viper.Initialize()


powertrain = veh.SimpleMapPowertrain(chrono.GetChronoDataFile("vehicle/powertrain/powertrain_map.json"))
viper.InitializePowertrain(powertrain)


driver = veh.ViperDriver(viper)


viper.GetSystem().Add(ground)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -8), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


step_size = 1e-3
simulation_time = 10.0
steering_time = 5.0
max_steering = 0.5


time = 0
while time < simulation_time:
    
    if time < steering_time:
        steering = (time / steering_time) * max_steering
    else:
        steering = max_steering
    
    
    throttle = 0.5  
    braking = 0.0   
    
    
    driver.SetInputs(steering, throttle, braking)
    viper.Synchronize(time, driver.GetInputs(), system.GetContactContainer())
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    time += step_size


vis.GetDevice().closeDevice()