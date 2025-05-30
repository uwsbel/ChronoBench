import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono.sensor import *


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





time_step = 2e-3


sensor_manager = sens.ChSensorManagerNSC()
system.Add(sensor_manager)


lidar_sensor = sens.ChLidarSensorNSC()
lidar_sensor.SetRange(10.0)
lidar_sensor.SetResolution(0.01)
lidar_sensor.SetFilter(0.5, 0.5)
lidar_sensor.SetFOV(45)
lidar_sensor.SetSensorPosition(chrono.ChVector3d(0.5, 1.0, 1.0))
lidar_sensor.SetSensorOrientation(chrono.ChQuaterniond(1, 0, 0, 0))
system.Add(lidar_sensor)


boxes = []
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, chrono.ChContactMaterialNSC())
    box_pos = chrono.ChVector3d(
        np.random.uniform(-10, 10),
        np.random.uniform(-10, 10),
        np.random.uniform(-10, 10)
    )
    box.SetPos(box_pos)
    boxes.append(box)
    system.Add(box)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    sensor_manager.Update()

    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    
    if abs(time - 1.0) < 1e-4:
        move('left')

    
    if abs(time - 2.0) < 1e-4:
        move('right')

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)