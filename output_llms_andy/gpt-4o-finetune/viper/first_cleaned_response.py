import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr





chrono.SetChronoDataPath(chrono.GetChronoDataPath())

out_dir = chrono.GetChronoOutputPath() + "VIPER"


system = chrono.ChSystemNSC()

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSCH)

system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  

ch_contact_model = chrono.ChContactMethodNSC
system.SetContactForceModel(ch_contact_model)


inerter_mat = chrono.ChContactMaterialNSC()

inerter_mat.SetFriction(0.9)

inerter_mat.SetRestitution(0.01)



ground = chrono.ChBody()
system.Add(ground)
ground.SetFixed(True)  


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)  
terrain = chrono.ChContactSurfaceNSC()
terrain.SetMaterialSurface(patch_mat)
terrain.AddTri(chrono.ChVector3d(-100, -100, 0), chrono.ChVector3d(-100, 100, 0), chrono.ChVector3d(100, 100, 0))
terrain.AddTri(chrono.ChVector3d(-100, -100, 0), chrono.ChVector3d(100, 100, 0), chrono.ChVector3d(100, -100, 0))
terrain.GetMesh().SetWireframe(True)
ground.GetAssets().Add(terrain)





rover = viper.Viper(system)
rover.Initialize(chrono.ChFrame3d(chrono.ChVector3d(0, 0, 0.3), chrono.ChQuaterniond(1, 0, 0, 0)))


driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 30, 512)





time_step = 1e-3

time = 0
time_end = 20  

render_step_size = 1.0 / 50  

render_steps = math.ceil(render_step_size / time_step)

step_number = 0
render_frame = 0
tire_force = chrono.ChVector3d()


driver.SetDrivingMode(viper.ViperDCMotorControl.Drivemode_ACTIVE)


while vis.Run():
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    if 1 < time < 4:
        driver.SetSteering((time - 1) / 3)
    
    rover.Update()
    
    system.DoStepDynamics(time_step)
    
    time += time_step
    step_number += 1