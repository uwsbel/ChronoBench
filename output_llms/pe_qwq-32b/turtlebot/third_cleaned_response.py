import os
import math
import numpy as np
import random
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)


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


for _ in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(-5, 5)
    box.SetPos(chrono.ChVectorD(pos_x, pos_y, 0.25))
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


sensor_mgr = sens.ChSensorManager(system)
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))
lidar.SetRotation(chrono.Q_from_Euler_123(0, 0, 0))
lidar.SetRange(10)
lidar.SetOpeningAngle(math.pi/2)
lidar.SetResolutionHorizontal(0.01)
lidar.SetResolutionVertical(0.01)
lidar.SetFovHorizontal(math.pi/2)
lidar.SetFovVertical(math.pi/4)
lidar.SetSamplingFrequency(100)

min_dist = 0.1
max_dist = 10
filter_min = sens.ChFilterDistance(min_dist)
filter_max = sens.ChFilterDistance(max_dist)
lidar.AddFilter(filter_min)
lidar.AddFilter(filter_max)

lidar.AttachTo(robot.GetBody())
sensor_mgr.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

time_step = 2e-3
time = 0

while vis.Run():
    
    if time < 1.0:
        move(robot, 'straight')
    elif 1.0 <= time < 2.0:
        move(robot, 'left')
    else:
        move(robot, 'right')

    
    sensor_mgr.Update()

    
    system.DoStepDynamics(time_step)
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()