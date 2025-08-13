import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono environment
chrono.SetChronoDataPath('/path/to/data')  # Update the path to your Chrono data directory
sys = chrono.ChSystemNSC()

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))

# Create materials
material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)

# Create a fixed truss
truss = chrono.ChBody()
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetMass(0)
truss.SetFixed(True)  # Fix the truss in space
truss.EnableCollision(True)
sys.Add(truss)

# Create a rotating bar
bar_length = 4.0
bar = chrono.ChBodyEasyBox(bar_length, 0.1, 0.1, 1000, True, True, material)
bar.SetPos(chrono.ChVector3d(0, 0, 0))
bar.SetRot(chrono.Q_from_AngZ(math.pi / 2))  # Rotate the bar to be vertical
sys.Add(bar)

# Create two gears
gear1_radius = 0.5
gear2_radius = 1.0

# Create first gear
gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.1, 1000, True, True, material)
gear1.SetPos(chrono.ChVector3d(0, 0, 0))
gear1.SetRot(chrono.Q_from_AngZ(0))  # Align with the bar
sys.Add(gear1)

# Create second gear
gear2 = chrono.ChBodyEasyCylinder(gear2_radius, 0.1, 1000, True, True, material)
gear2.SetPos(chrono.ChVector3d(0, 0, gear1_radius + gear2_radius))  # Position above gear1
gear2.SetRot(chrono.Q_from_AngZ(math.pi))  # Align with the bar
sys.Add(gear2)

# Create a motor joint to enforce constant rotation speed on gear1
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngZ(0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Constant speed of 1 rad/s
sys.Add(motor)

# Set up the visualization for the bodies
truss_shape = chrono.ChVisualShapeBox(4, 0.1, 0.1)
truss.AddVisualShape(truss_shape)

bar_shape = chrono.ChVisualShapeBox(bar_length, 0.1, 0.1)
bar.AddVisualShape(bar_shape)

gear1_shape = chrono.ChVisualShapeCylinder(gear1_radius, 0.1)
gear1.AddVisualShape(gear1_shape)

gear2_shape = chrono.ChVisualShapeCylinder(gear2_radius, 0.1)
gear2.AddVisualShape(gear2_shape)

# Simulation loop
while vis.Run():
    sys.DoStepDynamics(0.01)  # Step the simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print("Simulation completed.")