import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetVisualModel().AddBox(10, 0.1, 10, chrono.ChVectorD(0, -0.1, 0))
ground.GetCollisionModel().AddBox(10, 0.1, 10, chrono.ChVectorD(0, -0.1, 0))
my_system.Add(ground)


viper = chrono.ChViperWheeledVehicle(my_system)
viper.Initialize(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
viper.GetChassis().SetBodyFixed(False)
my_system.Add(viper.GetChassis())


driver = chrono.ChViperDriver(viper)
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetUseSkyBox(True)


time_step = 0.01
sim_time = 0
max_time = 10.0

while vis.Run() and sim_time < max_time:
    
    steering_angle = 0.5 * math.sin(2 * math.pi * sim_time / 5)  
    throttle = 0.3  

    driver.SetSteering(steering_angle)
    driver.SetThrottle(throttle)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    my_system.DoStepDynamics(time_step)
    sim_time += time_step

    
    print(f"Simulation time: {sim_time:.2f}s")