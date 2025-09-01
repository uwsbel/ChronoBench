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
lidar_sensor = sens.ChLidarSensor(system)
lidar_sensor.SetRange(5)
lidar_sensor.SetFov(math.pi)
lidar_sensor.SetNumBeams(360)
lidar_sensor.SetBeamAngle(0.01)
lidar_sensor.SetFilter(sens.ChSensorFilter.Type_NEAREST)
sensor_manager.AddSensor(lidar_sensor)


box_size = 0.5
for i in range(5):
    box_pos = chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), 0.5)
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 100, True, True, ground_mat)
    box.SetPos(box_pos)
    system.Add(box)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with obstacles')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)


time = 0
while vis.Run():
    
    sensor_manager.Update()

    
    move('straight')

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)