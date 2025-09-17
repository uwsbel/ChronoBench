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


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, chrono.ChMaterialSurfaceBase.ContactMethod.NSC, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, chrono.ChMaterialSurfaceBase.ContactMethod.NSC, ground_mat)
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


sensor_manager = sens.ChSensorManager(system)


chassis = rover.GetChassis()
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))  
lidar.SetUpdatePeriod(0.1)  
lidar.SetHorizontalResolution(360)  
lidar.SetVerticalResolution(1)     
lidar.SetHorizontalFOV(360)        
lidar.SetVerticalFOV(20)           
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseStandardDeviation(0.01)
lidar.AddFilter(sens.ChLidarRangeClipFilter(0.1, 10.0))
lidar.AttachTo(chassis)
sensor_manager.AddSensor(lidar)


time_step = 1e-3
time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    sensor_manager.Update()