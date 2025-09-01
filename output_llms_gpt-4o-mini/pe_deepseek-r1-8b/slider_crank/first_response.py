# Initialize the system with Non-Smooth Contact (NSC) for precise contact handling
sys = chrono.ChSystemNSC()

# Set the path to Chrono's data directory
chrono.SetChronoDataPath('/path/to/chrono/data')

# Initialize the visualization system using Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
# Attach the system to the visualization
vis.AttachSystem(sys)
# Set window size
vis.SetWindowSize(1024, 768)
# Set window title
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
# Add skybox
vis.AddSkyBox()
# Position the camera
cam_pos = chrono.ChVector3d(0, 3, 6)
vis.AddCamera(cam_pos)
# Add typical lights
vis.AddTypicalLights()
# Add logo (replace 'logo_pychrono_alpha.png' with your logo path)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Create a rigid body for the floor with a large size
floor_body = chrono.ChBodyEasyBox(5, 0.5, 0.5, 1000, True, True, chrono.ChContactMaterialSMC())
floor_body.SetPos(chrono.ChVector3d(0, 0, 0))
floor_body.SetFixed(True)
sys.Add(floor_body)

# Add visual shape for the floor
floor_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(5, 0.5, 0.5))
floor_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))  # Dark gray color
floor_body.AddVisualShape(floor_shape)

# Create the crankshaft as a simple rod
crankshaft_body = chrono.ChBodyEasyRod(0.5, 0.05, 1000, True, True, chrono.ChContactMaterialSMC())
crankshaft_body.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # No inertia for a thin rod
crankshaft_body.EnableCollision(True)
sys.Add(crankshaft_body)

# Add visual shape for the crankshaft
crankshaft_shape = chrono.ChVisualShapeCylinder(0.025, 0.05, True)
crankshaft_shape.SetColor(chrono.ChColor(200, 200, 200))  # Silver color
crankshaft_body.AddVisualShape(crankshaft_shape)

# Create the connecting rod
connecting_rod_body = chrono.ChBodyEasyRod(0.5, 0.05, 1000, True, True, chrono.ChContactMaterialSMC())
# Position the connecting rod
connect_pos = chrono.ChVector3d(crankshaft_body.GetPosition() + chrono.ChVector3d(0.5, 0, 0), 0, 0)
connecting_rod_body.SetPos(connect_pos)
connecting_rod_body.EnableCollision(True)
sys.Add(connecting_rod_body)

# Add visual shape for the connecting rod
connecting_rod_shape = chrono.ChVisualShapeCylinder(0.025, 0.05, True)
connecting_rod_shape.SetColor(chrono.ChColor(150, 150, 150))  # Light silver color
connecting_rod_body.AddVisualShape(connecting_rod_shape)

# Create the piston as a sphere
piston_body = chrono.ChBodyEasySphere(0.2, 1000, True, True, chrono.ChContactMaterialSMC())
# Position the piston
piston_pos = connect_pos + chrono.ChVector3d(0, 0, 0.2)
piston_body.SetPos(piston_pos)
piston_body.EnableCollision(True)
sys.Add(piston_body)

# Add visual shape for the piston
piston_shape = chrono.ChVisualShapeSphere(0.2)
piston_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color for visibility
piston_body.AddVisualShape(piston_shape)

# Create a revolute joint for the crankshaft
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft_body, floor_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))

# Create a motor joint to drive the crankshaft
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft_body, floor_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
crank_motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Constant angular speed

sys.Add(crank_joint)
sys.Add(crank_motor)

# Create a prismatic joint for the piston
piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(piston_body, floor_body, chrono.ChCoordsysD(piston_pos, chrono.QuatFromAngleX(chrono.CH_PI / 2)))
sys.Add(piston_joint)

# Add visual shapes to the connecting rod and piston
connecting_rod_body.AddVisualShape(connecting_rod_shape)
piston_body.AddVisualShape(piston_shape)

# Add a logo (replace 'logo_pychrono_alpha.png' with your logo path)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Define the simulation time step
time_step = 0.01

# Set up the simulation loop
while True:
    # Advance the simulation
    sys.DoStepDynamics(time_step)
    
    # Check if the visualization should continue running
    if not vis.Run():
        break
    
    # Begin the scene
    vis.BeginScene()
    
    # Render the scene
    vis.Render()
    
    # End the scene
    vis.EndScene()

import os
import math
import numpy as np
import chrono
from chrono import ChSystem, ChBody, ChContactMaterialSMC, ChFunction_Const
from chronoirr import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeCylinder, ChVisualShapeSphere
from pychrono import robot, vehicle

# Initialize the system with Non-Smooth Contact (NSC)
sys = chrono.ChSystemNSC()

# Set the path to Chrono's data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Initialize the visualization system using Irrlicht
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.AddSkyBox()
cam_pos = ChVector3d(0, 3, 6)
vis.AddCamera(cam_pos)
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Create the floor (truss)
floor_body = ChBodyEasyBox(5, 0.5, 0.5, 1000, True, True, ChContactMaterialSMC())
floor_body.SetPos(ChVector3d(0, 0, 0))
floor_body.SetFixed(True)
sys.Add(floor_body)

floor_shape = ChVisualShapeBox(ChVector3d(5, 0.5, 0.5))
floor_shape.SetColor(ChColor(0.3, 0.3, 0.3))  # Dark gray color
floor_body.AddVisualShape(floor_shape)

# Create the crankshaft
crankshaft_body = ChBodyEasyRod(0.5, 0.05, 1000, True, True, ChContactMaterialSMC())
crankshaft_body.SetPos(ChVector3d(0, 0, 0))
crankshaft_body.SetInertiaXX(ChVector3d(0, 0, 0))  # No inertia for a thin rod
crankshaft_body.EnableCollision(True)
sys.Add(crankshaft_body)

crankshaft_shape = ChVisualShapeCylinder(0.025, 0.05, True)
crankshaft_shape.SetColor(ChColor(200, 200, 200))  # Silver color
crankshaft_body.AddVisualShape(crankshaft_shape)

# Create the connecting rod
connecting_rod_body = ChBodyEasyRod(0.5, 0.05, 1000, True, True, ChContactMaterialSMC())
connect_pos = ChVector3d(crankshaft_body.GetPosition() + ChVector3d(0.5, 0, 0), 0, 0)
connecting_rod_body.SetPos(connect_pos)
connecting_rod_body.EnableCollision(True)
sys.Add(connecting_rod_body)

connecting_rod_shape = ChVisualShapeCylinder(0.025, 0.05, True)
connecting_rod_shape.SetColor(ChColor(150, 150, 150))  # Light silver color
connecting_rod_body.AddVisualShape(connecting_rod_shape)

# Create the piston
piston_body = ChBodyEasySphere(0.2, 1000, True, True, ChContactMaterialSMC())
piston_pos = connect_pos + ChVector3d(0, 0, 0.2)
piston_body.SetPos(piston_pos)
piston_body.EnableCollision(True)
sys.Add(piston_body)

piston_shape = ChVisualShapeSphere(0.2)
piston_shape.SetColor(ChColor(1, 0, 0))  # Red color for visibility
piston_body.AddVisualShape(piston_shape)

# Add joints and motor
crank_joint = ChLinkLockRevolute()
crank_joint.Initialize(crankshaft_body, floor_body, ChCoordsysD(ChVector3d(0, 0, 0), ChQuatFromAngleX(Ch.PI / 2)))
sys.Add(crank_joint)

crank_motor = ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft_body, floor_body, ChCoordsysD(ChVector3d(0, 0, 0), ChQuatFromAngleX(Ch.PI / 2)))
crank_motor.SetSpeedFunction(ChFunction_Const(1.0))  # Constant angular speed
sys.Add(crank_motor)

piston_joint = ChLinkLockPrismatic()
piston_joint.Initialize(piston_body, floor_body, ChCoordsysD(piston_pos, ChQuatFromAngleX(Ch.PI / 2)))
sys.Add(piston_joint)

# Add visual elements
vis.AddVisualShape(crankshaft_shape)
vis.AddVisualShape(connecting_rod_shape)
vis.AddVisualShape(piston_shape)

# Add logo
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Run the simulation
while True:
    sys.DoStepDynamics(0.01)
    if not vis.Run():
        break
    vis.BeginScene()
    vis.Render()
    vis.EndScene()