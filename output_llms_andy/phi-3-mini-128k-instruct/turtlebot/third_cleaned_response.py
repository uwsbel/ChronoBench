import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
import chronoirr


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


sensor_manager = sens.SensorManager()


lidar_sensor = sens.LidarSensor()
lidar_sensor.SetFilterMode(sens.LidarSensor.FilterMode_Disable)
lidar_sensor.SetRange(100)
lidar_sensor.SetResolution(20)
lidar_sensor.SetMaxRange(1000)
sensor_manager.AddSensor(lidar_sensor)
system.Add(sensor_manager)


for i in range(5):
    box_size = np.random.uniform(0.5, 1.5)
    box_pos = chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-5, 5), np.random.uniform(-2, 2))
    box = chrono.ChBodyEasyBox(box_size, box_size, 0.1)
    box.SetPos(box_pos)
    box.SetBodyFixed(True)
    system.Add(box)


init_pos = chrono.ChVector3d(0, 0.2, 0)  
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



time_step = 2e-3


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


time = 0
while vis.Run():
    
    sensor_manager.Update()
    
    
    robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
    robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)