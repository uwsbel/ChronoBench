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
lidar = sens.ChLidarSensor()
lidar.SetName("my_lidar")
lidar.SetCollisionEnvelope(0.01)
lidar.SetMaxDistance(10)
lidar.SetHorizontalFOV(math.pi / 2)
lidar.SetVerticalFOV(math.pi / 4)
lidar.SetScanFrequency(10)
lidar.SetFilter(sens.ChLidarFilterGaussian(0.05))
lidar.AttachToBody(robot.GetChassisBody())
sensor_manager.AddSensor(lidar)


for i in range(5):
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 10, True, True)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.1))
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
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)


time = 0
while vis.Run():
    move('straight')  

    
    sensor_manager.Update()

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)