import pychrono as chrono
import numpy as np

# Initialize the Chrono system
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create beam nodes (11 nodes for 10 elements)
nodes = []
for i in range(11):
    pos = chrono.ChVector3d(i * 0.1, 0, 0)
    node = chrono.ChNodeFEAxyzrot(pos, chrono.ChQuaterniond(1, 0, 0, 0))
    system.Add(node)
    nodes.append(node)

# Create beam elements between nodes
elements = []
for i in range(10):
    element = chrono.ChElementBeamTimo()
    element.SetNodes(nodes[i], nodes[i+1])
    # Material and geometric properties
    E = 210e9  # Young's modulus (Pa)
    G = 0.78e9  # Shear modulus (Pa)
    area = 0.01  # Cross-sectional area (m²)
    Iyy = 1e-6  # Second moment of area (m⁴)
    Izz = 1e-6
    element.SetYoungModulus(E)
    element.SetShearModulus(G)
    element.SetSectionArea(area)
    element.SetIyy(Iyy)
    element.SetIzz(Izz)
    element.SetDiameter(0.05)  # For visualization
    system.Add(element)
    elements.append(element)

# Fix the first node
fixed_link = chrono.ChLinkNodeFixed(nodes[0])
system.Add(fixed_link)

# Apply downward displacement motor to last node
motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(nodes[-1], chrono.ChFrameMoving(chrono.ChVector3d(1, 0, 0)), 
                 chrono.ChFramed(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
motor.SetMotion(chrono.ChFunctionRamp(0, -10))  # -10 m/s downward velocity
system.Add(motor)

# Add visualization for nodes and beam
for node in nodes:
    sphere = chrono.ChVisualShapeSphere(0.02)
    sphere.SetColor(chrono.ChColor(0.8, 0, 0))
    node.AddVisualShape(sphere)

# Set up Irrlicht visualization
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Beam Buckling FEA")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -1, 1), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()

# Configure solver and timestepper
system.SetTimestepperType(chrono.ChTimestepper.Type_FEA_IMPLICIT)
solver = system.GetSolver()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-8)
solver.EnableWarmStart(True)

# Simulation loop
time_end = 2.0
step = 0.001
while system.GetChTime() < time_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step)