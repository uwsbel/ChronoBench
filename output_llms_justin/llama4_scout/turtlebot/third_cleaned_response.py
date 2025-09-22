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


lidar_params = sens.ChLidarSensor.Params()
lidar_params.num_horizontal_scan_lines = 16
lidar_params.horizontal_scan_angle = 2 * math.pi
lidar_params.vertical_scan_angle = 30 * math.pi / 180
lidar_params.scan_frequency = 10
lidar_params.min_range = 0.1
lidar_params.max_range = 10
lidar_sensor = sens.ChLidarSensor(robot.GetChassisBody(), lidar_params)
lidar_sensor.SetPosition(chrono.ChVector3d(0, 0, 0.5))
lidar_sensor.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
sensor_manager.AddSensor(lidar_sensor)


for _ in range(5):
    box_body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box_body)


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, 0)  
        robot.SetMotorSpeed(0, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi / 4, 0)
        robot.SetMotorSpeed(math.pi / 4, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi / 4, 0)
        robot.SetMotorSpeed(-math.pi / 4, 1)


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
    
    move('straight')  

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)