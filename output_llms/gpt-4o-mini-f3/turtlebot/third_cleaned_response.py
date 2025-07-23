import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import random


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


num_boxes = 5
for i in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 1.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(random.uniform(-5, 5), random.uniform(-5, 5), 0.25))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


manager = sens.ChSensorManager(system)



offset_pose = chrono.ChFramed(
        chrono.ChVector3d(2.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
 
lidar = sens.ChLidarSensor(
    robot.GetRobot(),              
    update_rate,                  
    offset_pose,                  
    horizontal_samples,           
    vertical_samples,             
    horizontal_fov,               
    max_vert_angle,               
    min_vert_angle,               
    100.0,                        
    sens.LidarMode_STANDARD       
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)


lidar.PushFilter(sens.ChFilterDIAttitudeBias())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))




manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


time = 0


def move(mode):
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi/2, LEFT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi/2, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    else:
        raise ValueError('Invalid mode')

while vis.Run():
    
    
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    else:
        move('right')
    
    
    
    time += time_step
    
    
    
    manager.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)