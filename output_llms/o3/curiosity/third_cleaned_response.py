import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens                   
from   pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))             

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin  (0.0025)




ground_mat = chrono.ChContactMaterialNSC()




ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)




rover  = robot.Curiosity(system)

driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))        




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover – rigid terrain + LiDAR')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0,  3,  3),
              chrono.ChVector3d(0,  0,  0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)




manager = sens.ChSensorManager(system)


manager.scene.AddPointLight(chrono.ChVectorF(25, 25, 25),
                            chrono.ChColor(1.0, 1.0, 1.0), 800)





if hasattr(rover, "GetChassisBody"):
    lidar_parent = rover.GetChassisBody()
elif hasattr(rover, "GetChassis"):
    lidar_parent = rover.GetChassis()
else:
    lidar_parent = rover  


lidar_offset = chrono.ChFrameD( chrono.ChVector3d(0.0, 0.0, 1.2),
                                chrono.ChQuaterniond(1, 0, 0, 0))


lidar_update_rate    = 10.0                 
lidar_h_samples      = 720                  
lidar_v_samples      = 16                   
lidar_h_fov          = math.radians(360)    
lidar_v_fov          = math.radians(30)     
lidar_max_distance   = 100.0                


lidar = sens.ChLidarSensor(
            lidar_parent,                   
            lidar_update_rate,              
            lidar_offset,                   
            lidar_h_samples,                
            lidar_v_samples,                
            lidar_h_fov,                    
            lidar_v_fov,                    
            lidar_max_distance)             

lidar.SetName("Curiosity_LiDAR")
lidar.SetLag(0.0)                           
lidar.SetCollectionWindow(1.0/lidar_update_rate)


lidar.PushFilter(sens.ChFilterLidarXYZIS())          
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_output/"))


manager.AddSensor(lidar)




time_step = 1e-3

while vis.Run():
    
    driver.SetSteering(0.0)

    
    manager.Update()

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)