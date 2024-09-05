import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)





manager = sens.ChSensorManager(system)


lidar_update_rate = 100.0
lidar_horizontal_samples = 1000
lidar_vertical_samples = 32
lidar_horizontal_fov = chrono.CH_C_PI
lidar_vertical_fov = chrono.CH_C_PI / 3


lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),
    update_rate=lidar_update_rate,
    horizontal_samples=lidar_horizontal_samples,
    vertical_samples=lidar_vertical_samples,
    horizontal_fov=lidar_horizontal_fov,
    vertical_fov=lidar_vertical_fov
)


lidar.AddFilter(sens.ChFilterShadows())
lidar.AddFilter(sens.ChFilterPointsRange(0.1, 100))


manager.AddSensor(lidar)


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)