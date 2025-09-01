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





class MovementMode:
    STRAIGHT = "straight"
    LEFT = "left"
    RIGHT = "right"

def move(mode):
    
    if mode not in MovementMode.__dict__.values():
        raise ValueError(f"Invalid mode: {mode}")
    
    
    left_speed = 0
    right_speed = 0
    
    if mode == MovementMode.STRAIGHT:
        left_speed = 1
        right_speed = 1
    elif mode == MovementMode.LEFT:
        left_speed = 1
        right_speed = -1
    elif mode == MovementMode.RIGHT:
        left_speed = -1
        right_speed = 1
    
    robot.SetMotorSpeed(left_speed, LEFT_DRIVE_WHEEL)
    robot.SetMotorSpeed(right_speed, RIGHT_DRIVE_WHEEL)
    
    return


time_step = 2e-3


time = 0
while vis.Run():
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    
    move_mode = MovementMode.STRAIGHT
    if time >= 5:
        if time >= 10:
            move_mode = MovementMode.RIGHT
        else:
            move_mode = MovementMode.LEFT
    elif time >= 5:
        move_mode = MovementMode.LEFT
    
    
    print(f"Robot action: {move_mode}")
    
    
    if abs(time - 1.0) < 1e-4:
        move(MovementMode.LEFT)
        
    
    if abs(time - 2.0) < 1e-4:
        move(MovementMode.RIGHT)
    
    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)