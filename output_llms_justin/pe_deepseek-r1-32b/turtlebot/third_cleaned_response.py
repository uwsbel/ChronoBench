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
robot = turtlebot.TurtleBot(system, init_pos, init_rot, True, True)  
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





num_boxes = 5
for i in range(num_boxes):
    
    pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 1)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box.SetPos(pos)
    box.SetMass(10)
    system.Add(box)


sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetName("lidar_manager")


lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetRange(50.0)
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetFovHorizontal(180)
lidar.SetFovVertical(30)
lidar.SetPosition(chrono.ChVector3d(0, 0.5, 1.0))  
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  


range_filter = sens.ChLidarRangeFilter()
range_filter.SetMinRange(0.1)
range_filter.SetMaxRange(50.0)
lidar.AddFilter(range_filter)

intensity_filter = sens.ChLidarIntensityFilter()
intensity_filter.SetMinIntensity(0.1)
intensity_filter.SetMaxIntensity(1.0)
lidar.AddFilter(intensity_filter)

sensor_manager.AddSensor(lidar)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)


time_step = 2e-3


time = 0
while vis.Run():
    
    move('straight')

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)