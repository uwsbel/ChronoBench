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


time_step = 2e-3


time = 0
def move(mode):
    if mode == "straight":
        robot.SetMotorSpeed(1, 0)
        robot.SetMotorSpeed(1, 1)
    elif mode == "left":
        robot.SetMotorSpeed(1, 0)
        robot.SetMotorSpeed(-1, 1)
    elif mode == "right":
        robot.SetMotorSpeed(-1, 0)
        robot.SetMotorSpeed(1, 1)
    else:
        raise ValueError("Invalid mode")

while vis.Run():
    
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    
    if abs(time - 5.0) < 1e-4:
        move("straight")
        print("Moving straight")

    
    if abs(time - 10.0) < 1e-4:
        move("left")
        print("Turning left")

    
    if abs(time - 15.0) < 1e-4:
        move("right")
        print("Turning right")

    
    time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)