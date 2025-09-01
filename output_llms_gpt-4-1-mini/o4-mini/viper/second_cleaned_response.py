import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as viper
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(
    20.0,   
    20.0,   
    1.0,    
    1000,   
    True,   
    True,   
    ground_mat
)
ground.SetPos(chrono.ChVector3d(0, 0, -1.0))
ground.SetBodyFixed(True)

ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)




rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

initial_frame = chrono.ChFrameD(init_rot, init_pos)
rover.Initialize(initial_frame)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()

vis.AddCamera(
    chrono.ChVector3d(0, 2.5, 1.5),
    chrono.ChVector3d(0, 0, 1)
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)




manager = sens.ChSensorManager(system)
intensity = 1.0

manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0
)






q_pitch = chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
offset_pose = chrono.ChFrameD(q_pitch, chrono.ChVector3d(1.0, 0, 1.45))

cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),  
    15,                            
    offset_pose,                   
    720,                           
    480,                           
    1.408                          
)
cam.SetName("Third Person POV")

cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)




time_step = 1e-3
step_number = 0


render_step_size = 1.0 / 25.0
render_steps = math.ceil(render_step_size / time_step)




while vis.Run():
    
    t = system.GetChTime()

    
    max_steer = math.pi / 6.0
    steer = 0.0
    if 2.0 < t < 7.0:
        steer = max_steer * (t - 2.0) / 5.0
    elif 7.0 <= t < 12.0:
        steer = max_steer * (12.0 - t) / 5.0

    driver.SetSteering(steer)
    rover.Update()

    
    system.DoStepDynamics(time_step)

    
    manager.Update()

    
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    step_number += 1