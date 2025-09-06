import os
import math
import numpy as np
import pychrono as chrono
from pychrono.robots.turtlebot import TurtleBot  
import pychrono.irrlicht as chronoirr


LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


def move(robot, mode):
    
    if mode == "straight":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)  
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == "left":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)        
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)  
    elif mode == "right":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)   
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)        
    else:
        raise ValueError("Invalid movement mode")


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravity(chrono.ChVector3d(0, 0, -9.81))  
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
robot = TurtleBot(system, init_pos, init_rot)  
robot.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 1), chrono.ChVector3d(0, 0, 0.2))  
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)


time_step = 2e-3


time = 0
current_mode = None
while vis.Run():
    
    if time < 5:
        mode = "straight"
    elif time < 10:
        mode = "left"
    else:
        mode = "right"
    
    
    if mode != current_mode:
        move(robot, mode)
        print(f"Time {time:.1f}s: Robot now {mode}")
        current_mode = mode

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    time += time_step