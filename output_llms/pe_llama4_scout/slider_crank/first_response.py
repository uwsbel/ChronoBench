import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robosimian
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the data path
chrono.SetChronoDataPath('/path/to/data')

# Create a new NSC system
system = chrono.ChSystemNSC()

# Set the gravity
system.SetGravity(chrono.ChVector3d(0, -9.81, 0))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.Initialize()

# Create a floor body
floor_body = chrono.ChBody()
floor_body.SetFixed(True)
floor_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 0.1, 10))
floor_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
floor_body.AddVisualShape(floor_shape)
system.Add(floor_body)

# Create a crankshaft body
crankshaft_body = chrono.ChBody()
crankshaft_body.SetMass(1.0)
crankshaft_body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
crankshaft_shape = chrono.ChVisualShapeCylinder(0.1, 1.0)
crankshaft_shape.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
crankshaft_body.AddVisualShape(crankshaft_shape)
crankshaft_body.SetPos(chrono.ChVector3d(0, 0.5, 0))
system.Add(crankshaft_body)

# Create a connecting rod body
connecting_rod_body = chrono.ChBody()
connecting_rod_body.SetMass(0.5)
connecting_rod_body.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
connecting_rod_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1.0, 0.1, 0.1))
connecting_rod_shape.SetColor(chrono.ChColor(0.0, 1.0, 0.0))
connecting_rod_body.AddVisualShape(connecting_rod_shape)
connecting_rod_body.SetPos(chrono.ChVector3d(0.5, 1.0, 0))
system.Add(connecting_rod_body)

# Create a piston body
piston_body = chrono.ChBody()
piston_body.SetMass(0.2)
piston_body.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
piston_shape = chrono.ChVisualShapeCylinder(0.2, 0.5)
piston_shape.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
piston_body.AddVisualShape(piston_shape)
piston_body.SetPos(chrono.ChVector3d(1.0, 1.5, 0))
system.Add(piston_body)

# Create a revolute joint for the crankshaft
crankshaft_joint = chrono.ChLinkLockRevolute()
crankshaft_joint.Initialize(floor_body, crankshaft_body, chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
system.Add(crankshaft_joint)

# Create a spherical joint for the connecting rod
connecting_rod_joint = chrono.ChLinkLockSpherical()
connecting_rod_joint.Initialize(crankshaft_body, connecting_rod_body, chrono.ChCoordsysd(chrono.ChVector3d(0.5, 1.0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
system.Add(connecting_rod_joint)

# Create a prismatic joint for the piston
piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(connecting_rod_body, piston_body, chrono.ChCoordsysd(chrono.ChVector3d(1.0, 1.5, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
system.Add(piston_joint)

# Create a motor for the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor_body, crankshaft_body, chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Constant angular speed
system.Add(motor)

while vis.Run():
    system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Add a logo
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Add a skybox
vis.AddSkyBox()

# Add a camera
vis.AddCamera(chrono.ChVector3d(0, 3, 6))

# Add typical lights
vis.AddTypicalLights()