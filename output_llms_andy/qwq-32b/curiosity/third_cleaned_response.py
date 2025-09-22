import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()  
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


sensor_manager = sens.ChSensorManager(system)


lidar_params = sens.ChLidarParameters()
lidar_params.m_horizontal_samples = 1024
lidar_params.m_vertical_samples = 64
lidar_params.m_fov_horizontal = chrono.CH_C_PI  
lidar_params.m_fov_vertical = chrono.CH_C_PI / 4  
lidar_params.m_update_rate = 10.0  


chassis = rover.GetChassisBody()  
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetParent(chassis)
lidar.SetPos(chrono.ChVectorD(0, 0, 0.5))  
lidar.SetRot(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))  
lidar.SetParameters(lidar_params)
lidar.SetUpdateRate(lidar_params.m_update_rate)


distance_filter = sens.ChLidarDistanceFilter()
distance_filter.SetMinDistance(0.1)
distance_filter.SetMaxDistance(10.0)
lidar.AddFilter(distance_filter)


sensor_manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


time_step = 1e-3
time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    system.DoStepDynamics(time_step)

    
    sensor_manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()  
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVectorD(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVectorD(-5, 0.0, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  


sensor_manager = sens.ChSensorManager(system)


lidar_params = sens.ChLidarParameters()
lidar_params.m_horizontal_samples = 1024
lidar_params.m_vertical_samples = 64
lidar_params.m_fov_horizontal = chrono.CH_C_PI  
lidar_params.m_fov_vertical = chrono.CH_C_PI / 4  
lidar_params.m_update_rate = 10.0  


chassis = rover.GetChassisBody()  
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetParent(chassis)
lidar.SetPos(chrono.ChVectorD(0, 0, 0.5))  
lidar.SetRot(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))  
lidar.SetParameters(lidar_params)
lidar.SetUpdateRate(lidar_params.m_update_rate)


distance_filter = sens.ChLidarDistanceFilter()
distance_filter.SetMinDistance(0.1)
distance_filter.SetMaxDistance(10.0)
lidar.AddFilter(distance_filter)


sensor_manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)


time_step = 1e-3
time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    system.DoStepDynamics(time_step)

    
    sensor_manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()