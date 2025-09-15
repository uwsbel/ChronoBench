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


sensor_manager = sens.ChSensorManager(system)
robot.GetBody().AddSensor(sensor_manager)


lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVector3d(0, 0.15, 0.3))  
lidar.SetRange(10.0)  
lidar.SetHorizontalFOV(math.pi * 0.5)  
lidar.SetVerticalFOV(math.pi * 0.25)  
lidar.SetResolution(0.1)  
lidar.SetCoordinateSystem(chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))


lidar.AddFilter(sens.ChLidarFilterMinRange(0.1))
lidar.AddFilter(sens.ChLidarFilterMaxRange(10.0))
lidar.AddFilter(sens.ChLidarFilterPointsBehindSensor())

sensor_manager.AddSensor(lidar)


for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
    box.SetPos(chrono.ChVector3d(
        np.random.uniform(-8, 8),
        np.random.uniform(-8, 8),
        0.25
    ))
    box.SetMaterialSurface(ground_mat)
    system.Add(box)


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


def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    sensor_manager.Update()
    
    
    lidar_data = lidar.GetPoints()
    
    
    
    if time < 10:
        move('straight')
    elif time < 15:
        move('left')
    else:
        move('right')
    
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)