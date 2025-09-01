import os
import math
import numpy as np
import random
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr




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




init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()




box_mat = chrono.ChContactMaterialNSC()
box_size = [0.2, 0.2, 0.2]
for i in range(5):
    x = random.uniform(-4, 4)
    y = random.uniform(-4, 4)
    z = box_size[2] / 2 - 0.6  
    box = chrono.ChBodyEasyBox(box_size[0], box_size[1], box_size[2], 500, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(False)
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    system.Add(box)





manager = sens.ChSensorManager(system)


lidar_update_rate = 10  
lidar_horiz_samples = 180
lidar_vert_samples = 1
lidar_horiz_fov = chrono.CH_C_PI  
lidar_vert_fov = 0.0
lidar_max_dist = 10.0
lidar_noise_std = 0.01

lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0.0, 0.0, 0.15))  

lidar = sens.ChLidarSensor(
    robot.GetBody(),                
    lidar_update_rate,              
    lidar_offset_pose,              
    lidar_horiz_samples,            
    lidar_vert_samples,             
    lidar_horiz_fov,                
    lidar_vert_fov,                 
    lidar_max_dist                  
)
lidar.SetName("Lidar Sensor")


lidar.PushFilter(sens.ChFilterLidarNoise(lidar_noise_std))
lidar.PushFilter(sens.ChFilterLidarAccess())  

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





LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    
    speed = 1.0  
    turn_speed = 1.0  
    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(turn_speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(turn_speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)




time_step = 2e-3
time = 0.0

while vis.Run():
    
    move('straight')

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    time += time_step