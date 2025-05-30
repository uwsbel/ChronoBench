import os
import math

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



LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


ROBOT_FORWARD_SPEED = math.pi  
TURN_SPEED_MAGNITUDE = math.pi / 2  


def move(mode_command):
    
    if mode_command == "straight":
        robot.SetMotorSpeed(ROBOT_FORWARD_SPEED, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(ROBOT_FORWARD_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode_command == "left":
        
        robot.SetMotorSpeed(-TURN_SPEED_MAGNITUDE, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(TURN_SPEED_MAGNITUDE, RIGHT_DRIVE_WHEEL)
    elif mode_command == "right":
        
        robot.SetMotorSpeed(TURN_SPEED_MAGNITUDE, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-TURN_SPEED_MAGNITUDE, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid move mode: {mode_command}. Must be 'straight', 'left', or 'right'.")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Modified Control')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2)) 
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 2e-3
time = 0.0  



current_action = None 

print("Starting simulation...")

while vis.Run():
    
    new_action_this_step = None
    if time < 5.0:
        new_action_this_step = "straight"
    elif time < 10.0: 
        new_action_this_step = "left"
    else: 
        new_action_this_step = "right"

    
    if new_action_this_step != current_action:
        current_action = new_action_this_step
        print(f"Time: {time:.2f}s - Robot action: Changing to {current_action}")
        move(current_action) 

    
    system.DoStepDynamics(time_step)

    
    time += time_step

    
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) 
    vis.Render()
    vis.EndScene()

print("Simulation finished.")