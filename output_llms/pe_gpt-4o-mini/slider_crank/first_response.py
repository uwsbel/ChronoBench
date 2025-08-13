import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono system
chrono.SetChronoDataPath('/path/to/data')  # Set to your local Chrono data path
sys = chrono.ChSystemNSC()

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 10))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Create the floor (truss)
floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -0.05, 0))
floor.SetFixed(True)
sys.Add(floor)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(0.05, 1, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(crankshaft)

# Create the connecting rod
connecting_rod = chrono.ChBodyEasyBox(0.05, 0.5, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(0, 0.25, 0))
sys.Add(connecting_rod)

# Create the piston
piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(0, 0.5, 0))
sys.Add(piston)

# Create joints
# Revolute joint for the crankshaft
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(crank_joint)

# Prismatic joint for the piston
piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(piston_joint)

# Universal joint for the connecting rod
connecting_joint = chrono.ChLinkUniversal()
connecting_joint.Initialize(connecting_rod, crankshaft, chrono.ChCoordsysD(chrono.ChVector3d(0, 0.25, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(connecting_joint)

# Motor to drive the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Set constant speed
sys.Add(motor)

# Simulation loop
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)

# Clean up
vis.Close()