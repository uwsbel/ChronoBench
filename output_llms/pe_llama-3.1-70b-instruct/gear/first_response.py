import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system
sys = chrono.ChSystemSMC()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()

# Set the window dimensions
vis.SetWindowSize(1024, 768)

# Set the window title
vis.SetWindowTitle("Epicyclic Gears Simulation")

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)

# Set the default camera position and look-at point
vis.SetCamera(chrono.ChVector3D(0, 0, 6), chrono.ChVector3D(0, 0, 0))

# Add typical lights to the visualization
vis.AddTypicalLights()

# Create the fixed truss
truss = chrono.ChBodyEasyBox(chrono.ChVector3D(1, 1, 1), 1000, True, True, chrono.ChContactMaterialNSC())
truss.SetPos(chrono.ChVector3D(0, 0, 0))
sys.Add(truss)

# Create the rotating bar
bar = chrono.ChBodyEasyBox(chrono.ChVector3D(1, 1, 1), 1000, True, True, chrono.ChContactMaterialNSC())
bar.SetPos(chrono.ChVector3D(0, 0, 1))
sys.Add(bar)

# Create the motor for the rotating bar
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
sys.Add(motor)

# Create the gears
gear1 = chrono.ChBodyEasyCylinder(chrono.ChVector3D(0.5, 0.5, 0.5), 1000, True, True, chrono.ChContactMaterialNSC())
gear1.SetPos(chrono.ChVector3D(-1, 0, 0))
sys.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(chrono.ChVector3D(0.5, 0.5, 0.5), 1000, True, True, chrono.ChContactMaterialNSC())
gear2.SetPos(chrono.ChVector3D(1, 0, 0))
sys.Add(gear2)

# Create the gear motor
gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
gear_motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
sys.Add(gear_motor)

# Simulation loop
while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()