import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr


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
vis.EnableShadows()  


class WheelID:
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, WheelID.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, WheelID.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, WheelID.RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError('Invalid mode. Please use "straight", "left", or "right".')


time = 0
while vis.Run():
    
    if time < 5:
        print('Robot is moving straight.')
        move('straight')
    
    elif time < 10:
        print('Robot is turning left.')
        move('left')
    
    else:
        print('Robot is turning right.')
        move('right')

    
    time += 2e-3  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(2e-3)