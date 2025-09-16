import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


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




sensor_manager = sens.ChSensorManager(system)
sensor_manager.AddSensor(robot)


lidar = sens.ChLidarSensor()
lidar.SetSensorMode(sens.ChLidarSensor.Mode_RANGE)
lidar.SetResolution(180)
lidar.SetRange(5.0)
lidar.SetNoise(0.01)
lidar.SetFilter(sens.ChLidarSensor.Filter_MEDIAN)
lidar.SetFilterSize(3)
lidar.Attach(robot, chrono.ChVector3d(0, 0, 0.1))
sensor_manager.AddSensor(lidar)


for i in range(5):
    box_size = 0.5
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(0, 1)))
    system.Add(box)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(0.5, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-0.5, 0)
        robot.SetMotorSpeed(0.5, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(-0.5, 1)
    else:
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(0, 1)


time_step = 2e-3


time = 0
while vis.Run():
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    
    if abs(time - 1.0) < 1e-4:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)

    
    if abs(time - 2.0) < 1e-4:
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)