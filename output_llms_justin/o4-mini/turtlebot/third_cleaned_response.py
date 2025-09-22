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




ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(
    20, 20, 1,            
    1000,                 
    True, True,           
    ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




for i in range(5):
    box = chrono.ChBodyEasyBox(
        0.5, 0.5, 0.5,      
        1000,               
        True, True)         
    
    rx = np.random.uniform(-5, 5)
    ry = np.random.uniform(-5, 5)
    box.SetPos(chrono.ChVector3d(rx, ry, 0.25))
    
    box.GetMaterialSurfaceNSC().SetFriction(0.6)
    
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.2, 0.2))
    system.Add(box)




init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()




manager = sens.ChSensorManager(system)
manager.SetVerbose(True)


chassis = robot.GetBody()


lidar_offset = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0.3),
    chrono.ChQuaternionD(1, 0, 0, 0))


lidar = sens.ChLidarSensor(
    chassis,
    updateRate=30,                
    offsetPose=lidar_offset,
    horizontalSamples=360,
    verticalSamples=1,
    horizontalFov=chrono.CH_C_2PI,  
    verticalFov=0.0,               
    maxDistance=10.0,
    minDistance=0.1)


lidar.PushFilter(sens.ChFilterPCtoBuffer())
lidar.PushFilter(sens.ChFilterVisualize(1280, 720))

manager.AddSensor(lidar)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot w/ LiDAR and Random Boxes')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 1.5, 0.2),
    chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512)






LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    
    if mode == 'straight':
        v = math.pi
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  v)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, v)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)
    else:
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)




time_step = 2e-3
t = 0.0

while vis.Run():
    
    move('straight')

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    t += time_step