import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
system.GetCollisionSystem().SetDefaultSuggestedEnvelope(0.0025)
system.GetCollisionSystem().SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


sensor_mgr = sens.ChSensorManager(system)
system.AddSensorManager(sensor_mgr)

lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetBody(robot.GetBody())  
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))  
lidar_params = sens.ChLidarParameters()
lidar_params.SetResolutionH(1024)
lidar_params.SetResolutionV(1)
lidar_params.SetFOV(chrono.ChVectorD(-20, 20, 0, 0))  
lidar_params.SetRange(0.1, 10.0)
lidar.SetParameters(lidar_params)


noise_filter = sens.ChFilterNoise()
noise_filter.SetName("Noise Filter")
noise_filter.SetNoiseSigma(0.01)
lidar.AddFilter(noise_filter)
sensor_mgr.AddSensor(lidar)


for _ in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 0.5, 1000, True, True, ground_mat)
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.25  
    box.SetPos(chrono.ChVectorD(x, y, z))
    box.SetCollide(True)
    system.Add(box)


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

time_step = 2e-3
time = 0


while vis.Run():
    
    move(robot, 'straight')
    
    
    sensor_mgr.Update(time)
    
    
    time += time_step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)