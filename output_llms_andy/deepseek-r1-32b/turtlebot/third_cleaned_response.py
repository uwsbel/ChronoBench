import os
import math
import numpy as np
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


sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetVisualize(True)


lidar = sens.ChLidar()
lidar.SetPosition(chrono.ChVector3d(0, 0, 0.5), robot.GetBody())  
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
lidar.SetNumberOfRays(360)
lidar.SetFieldOfView(chrono.CH_C_PI / 2)
lidar.SetRange(5.0)
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseFloor(0.01)
lidar.SetVariance(0.001)
lidar.SetPointsCloudColor(chrono.ChColor(0.5, 0.5, 0.5))
lidar.SetRenderingType(sens.ChLidar.RENDERING_TYPE_POINTS)
sensor_manager.AddSensor(lidar)


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





for i in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetMaterialSurface(ground_mat)
    system.Add(box)


def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)


time_step = 2e-3


time = 0
while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update()

    
    move('straight')

    
    time += time_step

    
    system.DoStepDynamics(time_step)