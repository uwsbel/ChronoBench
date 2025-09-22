import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
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


manager = sens.ChSensorManager(system)



lidar_update_rate = 10.0  
lidar_horz_samples = 512
lidar_vert_samples = 32
lidar_horz_fov = 2 * math.pi  
lidar_vert_fov = math.radians(30)  
lidar_max_range = 20.0  


chassis = rover.GetChassis()  


lidar_offset = chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.5), chrono.QUNIT)  


lidar = sens.ChLidarSensor(
    chassis,                
    lidar_update_rate,      
    lidar_offset,           
    lidar_horz_samples,     
    lidar_vert_samples,     
    lidar_horz_fov,         
    lidar_vert_fov,         
    lidar_max_range         
)


lidar.SetName("Lidar Sensor")


lidar.PushFilter(sens.ChFilterLidarAccess())  
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_points/"))  


manager.AddSensor(lidar)


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