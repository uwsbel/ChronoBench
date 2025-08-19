import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot







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


myrobot = robot.MyRobot(system)
myrobot.Initialize()


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Turtlebot')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5,0,0.2))
vis.AddTypicalLights()


step_size = 2e-3


time = 0
while vis.Run():
    time += step_size

    if time < 5 :
        print ("Moving straight...\n")
        myrobot.SetMotorSpeeds(3,3)
    elif time < 10 :
        print ("Turning left...\n")
        myrobot.SetMotorSpeeds(3,-3)
    else:
        print ("Turning right...\n")
        myrobot.SetMotorSpeeds(-3,3)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    myrobot.Advance(step_size)
    system.DoStepDynamics(step_size)