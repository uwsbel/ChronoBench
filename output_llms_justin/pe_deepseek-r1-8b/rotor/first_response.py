import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea, robosimian, veh
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle

# Initialize the system with SMC contact model
sys = chrono.ChSystemSMC()

# Set the data path
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Create a visualization system with Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()

# Define the beam length
beam_L = 4.0  # Length of the Jeffcott rotor

# Create an IGA beam section
beam_section = fea.ChBeamSectionIga()
beam_section.SetYoungModulus(0.01e9)  # Young's modulus
beam_section.SetShearModulus(0.01e9 * 0.3)  # Shear modulus
beam_section.SetRayleighDamping(0.000)  # Damping coefficient

# Define the coordinate system for the beam
beam_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))

# Create nodes for the beam
nodes = []
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
for i in range(1, 11):  # 10 nodes for IGA approximation
    nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))

# Create beam elements
elements = []
for i in range(len(nodes) - 1):
    elem = fea.ChElementBeamIga()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(beam_section)
    elements.append(elem)
    sys.Add(elem)

# Add all nodes and elements to the mesh
mesh = fea.ChMesh()
for node in nodes:
    mesh.AddNode(node)
for elem in elements:
    mesh.AddElement(elem)

# Create the flywheel body
flywheel_body = chrono.ChBody()
# Set the position of the flywheel at the midpoint of the beam
flywheel_pos = chrono.ChVector3d(beam_L / 2, 0, 0)
flywheel_body.SetPos(flywheel_pos)
flywheel_body.SetMass(1)  # Mass of the flywheel
flywheel_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  # Inertia tensor
flywheel_shape = chrono.ChVisualShapeSphere(0.5)  # Visual sphere for the flywheel
flywheel_body.AddVisualShape(flywheel_shape)
sys.Add(flywheel_body)

# Add the flywheel to the visualization
vis.Add(flywheel_body)

# Create a revolute joint for the motor
motor_joint = chrono.ChLinkLockRevolute()
motor_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2))  # 90-degree rotation
motor_joint.Initialize(nodes[1], flywheel_body, motor_coordsys)
sys.Add(motor_joint)

# Set the motor speed function (sine wave)
motor_speed_func = chrono.ChFunctionSine(0.1, 1.0)  # Frequency and amplitude
motor_joint.SetSpeedFunction(motor_speed_func)

# Define visual properties for the beam
beam_visual = chrono.ChVisualShapeBox(chrono.ChVector3d(beam_L, 0.05, 0.05))  # Box for the beam
for node in nodes:
    node.SetVisualShape(chrono.ChVisualShapeSphere(0.01))  # Small spheres for nodes

# Add all elements and nodes to the visualization
for elem in elements:
    elem.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(elem.GetLengthX(), 0.01, 0.01)))  # Visual boxes for beam elements
vis.AddElement(elem)
for node in nodes:
    vis.AddNode(node)

# Define the simulation time step
time_step = 0.001

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(time_step)
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea, robosimian, veh
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle

# Initialize the system with SMC contact model
sys = chrono.ChSystemSMC()

# Set the data path
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Create a visualization system with Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()

# Define the beam length
beam_L = 4.0  # Length of the Jeffcott rotor

# Create an IGA beam section
beam_section = fea.ChBeamSectionIga()
beam_section.SetYoungModulus(0.01e9)  # Young's modulus
beam_section.SetShearModulus(0.01e9 * 0.3)  # Shear modulus
beam_section.SetRayleighDamping(0.000)  # Damping coefficient

# Define the coordinate system for the beam
beam_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))

# Create nodes for the beam
nodes = []
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
for i in range(1, 11):  # 10 nodes for IGA approximation
    nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))
nodes.append(fea.ChNodeFEAxyzrot(beam_coordsys))

# Create beam elements
elements = []
for i in range(len(nodes) - 1):
    elem = fea.ChElementBeamIga()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(beam_section)
    elements.append(elem)
    sys.Add(elem)

# Add all nodes and elements to the mesh
mesh = fea.ChMesh()
for node in nodes:
    mesh.AddNode(node)
for elem in elements:
    mesh.AddElement(elem)

# Create the flywheel body
flywheel_body = chrono.ChBody()
# Set the position of the flywheel at the midpoint of the beam
flywheel_pos = chrono.ChVector3d(beam_L / 2, 0, 0)
flywheel_body.SetPos(flywheel_pos)
flywheel_body.SetMass(1)  # Mass of the flywheel
flywheel_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  # Inertia tensor
flywheel_shape = chrono.ChVisualShapeSphere(0.5)  # Visual sphere for the flywheel
flywheel_body.AddVisualShape(flywheel_shape)
sys.Add(flywheel_body)

# Add the flywheel to the visualization
vis.Add(flywheel_body)

# Create a revolute joint for the motor
motor_joint = chrono.ChLinkLockRevolute()
motor_coordsys = chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2))  # 90-degree rotation
motor_joint.Initialize(nodes[1], flywheel_body, motor_coordsys)
sys.Add(motor_joint)

# Set the motor speed function (sine wave)
motor_speed_func = chrono.ChFunctionSine(0.1, 1.0)  # Frequency and amplitude
motor_joint.SetSpeedFunction(motor_speed_func)

# Define visual properties for the beam
beam_visual = chrono.ChVisualShapeBox(chrono.ChVector3d(beam_L, 0.05, 0.05))  # Box for the beam
for node in nodes:
    node.SetVisualShape(chrono.ChVisualShapeSphere(0.01))  # Small spheres for nodes

# Add all elements and nodes to the visualization
for elem in elements:
    elem.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(elem.GetLengthX(), 0.01, 0.01)))  # Visual boxes for beam elements
vis.AddElement(elem)
for node in nodes:
    vis.AddNode(node)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.001)
    vis.EndScene()