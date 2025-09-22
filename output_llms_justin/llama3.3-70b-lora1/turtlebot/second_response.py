import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create myrobot
myrobot = robot.MyRobot(system)
myrobot.Initialize()

# Create Irrlicht visualization
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Turtlebot')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5,0,0.2))
vis.AddTypicalLights()

# Set the simulation step size
step_size = 2e-3

# Simulation loop
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