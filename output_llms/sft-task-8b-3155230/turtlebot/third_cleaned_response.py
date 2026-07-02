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
ground = chrono.ChBodyEasyBox(20, 20, 0.6, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


sensor_manager = sens.ChSensorManager(system)
lidar = sens.ChLidar(
    system,
    chrono.ChVector3d(0, 0.2, 0.1),  
    chrono.ChVector3d(0, 0, 1),      
    100,                            
    0.05,                           
    0.01,                           
    0.01,                           
    0.05,                           
    0.01,                           
    0.01,                           
    0.01,                           
    sens.ChLidar::LIDAR_TYPE_TRIANGLE
)
lidar.SetName("Lidar Sensor")
lidar.SetUpdateRate(100)
lidar.SetRangeFilter(0.05, 10.0)
lidar.SetPointFilter(0.01, 0.01)
sensor_manager.AddSensor(lidar)


box_size = 0.3
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.5
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(True)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0.2)  
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


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, turtlebot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, turtlebot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0, turtlebot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, turtlebot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0.5, turtlebot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, turtlebot.RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    move('straight')  

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)