import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVectorD(0, 0, 0.0))
box.SetBodyFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVectorD(-5, 0.0, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)


manager = sens.ChSensorManager(system)


lidar_update_rate = 5.0
lidar_horizontal_samples = 360
lidar_vertical_samples = 16
lidar_min_vert_angle = -0.26  
lidar_max_vert_angle = 0.26   
lidar_max_distance = 100.0
lidar_noise = 0.01  

offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0))

lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  
    lidar_update_rate,       
    offset_pose,             
    lidar_horizontal_samples,  
    lidar_vertical_samples,  
    chrono.CH_C_2PI,         
    lidar_min_vert_angle,    
    lidar_max_vert_angle,    
    lidar_max_distance       
)

lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar_noise_model = sens.ChLidarNoiseNormal(lidar_noise)
lidar.SetNoiseModel(lidar_noise_model)


manager.AddSensor(lidar)


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    driver.SetThrottle(0.5)
    driver.SetSteering(0.0)

    
    rover.Update()

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)