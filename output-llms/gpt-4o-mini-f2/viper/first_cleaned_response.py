import pychrono as chrono
import pychrono.robot as viper
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetFixed(True)  
ground.SetName("ground")
system.Add(ground)


rover = viper.Viper(system)  
rover.Initialize()  


driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)  


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40)





time_step = 1e-3


time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if time < 2.0:
        driver.SetSteering(0.5 * math.pi * time / 2.0)  
    elif time < 4.0:
        driver.SetSteering(0.5 * math.pi * (4.0 - time) / 2.0)  
    else:
        driver.SetSteering(0.5 * math.pi)  

    rover.Update()  

    
    time += time_step

    
    system.DoStepDynamics(time_step)