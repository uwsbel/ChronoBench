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
ground_mat.SetFriction(0.9) 
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box_mat = chrono.ChContactMaterialNSC() 
box_mat.SetFriction(0.7) 
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, box_mat)

box.SetPos(chrono.ChVector3d(0, 0, 0.125))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)



rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)



init_pos = chrono.ChVector3d(-5, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))


manager = sens.ChSensorManager(system)
manager.SetVerbose(False) 


update_rate = 10  
horizontal_samples = 180 
vertical_samples = 16    
horizontal_fov = math.pi   

vertical_fov_total = math.pi / 6  
max_distance = 100.0
lag = 0.0 
exposure_time = 0.0 



offset_pose = chrono.ChFrameD(chrono.ChVector3d(1.0, 0, 0.5), 
                              chrono.ChQuaterniond(1, 0, 0, 0))

lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  
    update_rate,             
    offset_pose,             
    horizontal_samples,      
    vertical_samples,        
    horizontal_fov,          
    vertical_fov_total,      
    max_distance             
    
)
lidar.SetName("Lidar Sensor")



lidar.PushFilter(sens.ChFilterPCfromDepth())


lidar.PushFilter(sens.ChFilterVisualizePointCloud(800, 600, 0.05, "Lidar Point Cloud"))


manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 6, 3), chrono.ChVector3d(0, 0, 0.5)) 
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)



time_step = 1e-3


time = 0
while vis.Run():
    current_time = system.GetChTime()
    

    
    driver.SetSteering(0.0)
    driver.SetThrottle(0.5) 

    
    rover.Update()

    
    system.DoStepDynamics(time_step)

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    
    
    vis.EndScene()


    if current_time > 20: 
        vis.GetDevice().closeDevice()


vis.EndLoop() 

print("Simulation finished.")