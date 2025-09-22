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
system.Add(sensor_manager)


lidar = sens.ChLidarSensor()
lidar.SetPosition(chrono.ChVector3d(0, 0.1, 0.5))  
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  
lidar.SetNumberOfRays(360)
lidar.SetFieldOfView(chrono.CH_C_PI / 2)
lidar.SetRange(5.0)
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseFloor(0.01)
lidar.SetResolution(0.01)
lidar.SetVariance(0.001)
lidar.SetRenderingType(sens.ChLidarSensor.RENDERING_TYPE_POINTS)
lidar.SetUpdateRate(1.0 / 30.0)
lidar.SetParent(robot.GetBody())
sensor_manager.AddSensor(lidar)


for i in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetMaterialSurface(chrono.ChMaterialSurface())
    box.GetMaterialSurface().SetFriction(0.5)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/wood.jpg"))
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
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    sensor_manager.Update()
    
    
    move('straight')
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    
    
    time += time_step