import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono.sensor import sensor


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


sensor_manager = sensor.ChSensorManager()
system.Add(sensor_manager)


lidar_sensor = sensor.ChLidarSensor()
lidar_sensor.SetPos(chrono.ChVector3d(0, 0, 1.5))
lidar_sensor.SetRot(chrono.ChQuaterniond(0, 0, 0, 1))
lidar_sensor.SetRange(10)
lidar_sensor.SetResolution(360)
lidar_sensor.SetNumPoints(100)
lidar_sensor.SetAngleStep(0.1)
lidar_sensor.SetDistanceFilter(0.1)
lidar_sensor.SetAngleFilter(0.1)
lidar_sensor.SetIntensityFilter(0.1)
lidar_sensor.SetNoiseFilter(0.1)
lidar_sensor.SetIntensityMin(0.1)
lidar_sensor.SetIntensityMax(1.0)
lidar_sensor.SetNoiseMin(0.1)
lidar_sensor.SetNoiseMax(1.0)
lidar_sensor.SetDistanceMin(0.1)
lidar_sensor.SetDistanceMax(10.0)
lidar_sensor.SetAngleMin(0.1)
lidar_sensor.SetAngleMax(360.0)
lidar_sensor.SetSensorType(sensor.ChSensorType_LIDAR)
lidar_sensor.SetSensorName("LidarSensor")
lidar_sensor.SetSensorDescription("Lidar sensor for Turtlebot")
lidar_sensor.SetSensorType(sensor.ChSensorType_LIDAR)
lidar_sensor.SetSensorName("LidarSensor")
lidar_sensor.SetSensorDescription("Lidar sensor for Turtlebot")
lidar_sensor.SetSensorType(sensor.ChSensorType_LIDAR)
lidar_sensor.SetSensorName("LidarSensor")
lidar_sensor.SetSensorDescription("Lidar sensor for Turtlebot")
sensor_manager.AddSensor(lidar_sensor)


box1 = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
box1.SetPos(chrono.ChVector3d(2, 2, 0))
box1.SetFixed(True)
system.Add(box1)

box2 = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
box2.SetPos(chrono.ChVector3d(3, 3, 0))
box2.SetFixed(True)
system.Add(box2)

box3 = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
box3.SetPos(chrono.ChVector3d(4, 4, 0))
box3.SetFixed(True)
system.Add(box3)

box4 = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
box4.SetPos(chrono.ChVector3d(5, 5, 0))
box4.SetFixed(True)
system.Add(box4)

box5 = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True)
box5.SetPos(chrono.ChVector3d(6, 6, 0))
box5.SetFixed(True)
system.Add(box5)


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
    if mode == 'straight':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)


time = 0
while vis.Run():
    
    if abs(time - 1.0) < 1e-4:
        move('left')
        
    
    if abs(time - 2.0) < 1e-4:
        move('right')
        
    
    if abs(time - 3.0) < 1e-4:
        move('straight')
        
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update(time_step)

    
    system.DoStepDynamics(time_step)