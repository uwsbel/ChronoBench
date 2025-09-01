import math
import numpy as np
import pychrono as chrono
import pychrono.robot as chronorobot
import pychrono.vehicle as chronovehicle
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath("chrono_data/")

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 3))
vis.AddTypicalLights()

# Floor (truss)
floor = chrono.ChBodyEasyBox(4, 0.2, 4, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -0.1, 0))
floor.SetFixed(True)
sys.Add(floor)

# Crankshaft
crank_length = 1.0
crank_mass = 1.0
crank = chrono.ChBodyEasyCylinder(crank_length / 2, 0.1, crank_mass, True, True)
crank.SetPos(chrono.ChVector3d(0, 0.2, 0))
sys.Add(crank)

# Connecting rod
rod_length = 2.0
rod_mass = 1.0
rod = chrono.ChBodyEasyCylinder(rod_length / 2, 0.05, rod_mass, True, True)
rod.SetPos(chrono.ChVector3d(crank_length, 0.2, 0))
sys.Add(rod)

# Piston
piston_mass = 2.0
piston = chrono.ChBodyEasyBox(0.2, 0.4, 0.2, piston_mass, True, True)
piston.SetPos(chrono.ChVector3d(crank_length + rod_length, 0.2, 0))
sys.Add(piston)

# Joints
# Revolute joint between floor and crank
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(floor, crank, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(rev_joint)

# Connecting rod to crank
spher_joint = chrono.ChLinkLockSpherical()
spher_joint.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVector3d(crank_length, 0, 0), chrono.QUNIT))
sys.Add(spher_joint)

# Piston to rod
pris_joint = chrono.ChLinkLockPrismatic()
pris_joint.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
pris_joint.SetPrismaticAxis(chrono.ChVector3d(0, 1, 0))
sys.Add(pris_joint)

# Motor to drive the crank
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crank, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_2PI * 0.5))  # 0.5 Hz

# Add all components to the system
sys.Add(motor)

# Simulation loop
time = 0.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(0.001)
    time += 0.001