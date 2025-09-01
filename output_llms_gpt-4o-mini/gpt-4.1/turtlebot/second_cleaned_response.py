import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
















system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC(system)  

ground = chrono.ChBodyEasyBox(10, 10, 0.5, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  
ground.SetFixed(True)

if ground.GetVisualShape(0):
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chronoirr.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)



LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    
    if mode == 'straight':
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid movement mode: {mode}")


time_step = 2e-3
time = 0
current_mode = None

while vis.Run():
    
    if time < 5.0:
        mode = 'straight'
    elif time < 10.0:
        mode = 'left'
    else:
        mode = 'right'

    
    if mode != current_mode:
        move(mode)
        print(f"Time {time:.2f}s: Moving {mode}")
        current_mode = mode

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    time += time_step