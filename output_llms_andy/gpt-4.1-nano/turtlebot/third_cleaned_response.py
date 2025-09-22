import os
import math
import random
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  
robot.Initialize()  


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


sensor_manager = sens.ChSensorManager(system)


sensor_pos = chrono.ChVector3d(0, 0, 0.2)
sensor_rot = chrono.ChQuaternionD(1, 0, 0, 0)


lidar = sens.ChLidarSensor(
    robot.GetBody(),  
    sensor_pos,
    sensor_rot,
    100,             
    math.pi * 2,     
    10,              
    0.5,             
    0.2,             
    1.0,             
)

lidar.AddFilter(sens.ChFilterPointRemoveBelowZ(0))
sensor_manager.AddSensor(lidar)


boxes = []
for i in range(5):
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(-5, 5)
    pos_z = 0.5  
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 500, True, True)
    box.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))
    box.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    system.Add(box)
    boxes.append(box)


def move(mode):
    
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi/4, 0)  
        robot.SetMotorSpeed(math.pi/4, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(math.pi/4, 0)
        robot.SetMotorSpeed(-math.pi/4, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi/4, 0)
        robot.SetMotorSpeed(math.pi/4, 1)
    else:
        robot.SetMotorSpeed(0, 0)


time = 0
while vis.Run():
    
    if abs(time - 1.0) < 1e-4:
        move('left')
    elif abs(time - 2.0) < 1e-4:
        move('right')
    elif time > 3.0:
        move('straight')

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)