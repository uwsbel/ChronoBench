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


sensor_manager = sens.ChSensorManager()
lidar_sensor = sens.ChLidarSensor()
lidar_sensor.SetName("Lidar")
lidar_sensor.SetPosition(chrono.ChVector3d(0, 0.2, 0))
lidar_sensor.SetDirection(chrono.ChVector3d(0, 0, -1))
lidar_sensor.SetFieldOfView(math.pi)
lidar_sensor.SetMaxRange(10)
lidar_sensor.SetNumBeams(360)
lidar_sensor.SetVerticalResolution(1)
lidar_sensor.SetHorizontalResolution(1)
lidar_sensor.SetVerticalFOV(math.pi / 2)
lidar_sensor.SetHorizontalFOV(math.pi / 2)
lidar_sensor.SetVerticalFOVFilter(math.pi / 4)
lidar_sensor.SetHorizontalFOVFilter(math.pi / 4)
lidar_sensor.SetMaxRangeFilter(5)
sensor_manager.AddSensor(lidar_sensor)


box_materials = [chrono.ChMaterialSurfaceNSC() for _ in range(5)]
box_shapes = [chrono.ChVisualShapeBox(np.random.uniform(0.5, 1.5), np.random.uniform(0.5, 1.5), np.random.uniform(0.5, 1.5)) for _ in range(5)]
boxes = [chrono.ChBodyEasyBox(np.random.uniform(0.5, 1.5), np.random.uniform(0.5, 1.5), np.random.uniform(0.5, 1.5), 1000, True, True, box_materials[i]) for i in range(5)]
for i, box in enumerate(boxes):
    box.SetPos(chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(0, 1)))
    box.AddVisualShape(box_shapes[i])
    system.Add(box)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(0.5, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(-0.5, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(-0.5, 0)
        robot.SetMotorSpeed(0.5, 1)


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