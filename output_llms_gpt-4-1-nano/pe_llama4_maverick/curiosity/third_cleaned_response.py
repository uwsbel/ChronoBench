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
rover.Initialize(chrono.ChCoordsysd(init_pos, init_rot))


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


lidar_update_rate = 10  
lidar_horizontal_samples = 4500
lidar_vertical_samples = 32
lidar_field_of_view = 2 * chrono.CH_C_PI  
lidar_max_distance = 100  
lidar_noise_model = sens.ChNoiseNormal(0.01, 0.01, 0.01)


lidar = sens.ChLidarSensor(rover.GetChassisBody(), lidar_update_rate, chrono.ChFrame(chrono.ChVector3d(0, 0, 0.5), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_Y)),
                           lidar_horizontal_samples, lidar_vertical_samples, chrono.CH_C_PI / 2, lidar_field_of_view, lidar_max_distance, lidar_noise_model)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(256, 256, "Lidar Depth Data"))
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