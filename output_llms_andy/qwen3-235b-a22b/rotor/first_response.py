import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math

# Initialize the Chrono system
sys = chrono.ChSystemNSC()

# Beam parameters
beam_length = 1.0
r = 0.05  # Radius of circular cross-section
E = 210e9  # Young's modulus (Pa)
nu = 0.3  # Poisson's ratio
G = E / (2 * (1 + nu))  # Shear modulus
rho = 7800  # Density (kg/m^3)

# Cross-sectional properties
area = math.pi * r**2
Iyy = math.pi * r**4 / 4
Izz = Iyy
J = math.pi * r**4 / 2  # Polar moment of inertia

# Create material for IGA beam
material = fea.ChMaterialBeamIGA()
material.E = E
material.G = G
material.rho = rho

# Create mesh
mesh = fea.ChMesh()

# Create nodes (with center node for flywheel attachment)
node_start = fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node_center = fea.ChNodeFEAxyz(chrono.ChVectorD(beam_length/2, 0, 0))
node_end = fea.ChNodeFEAxyz(chrono.ChVectorD(beam_length, 0, 0))
mesh.AddNode(node_start)
mesh.AddNode(node_center)
mesh.AddNode(node_end)

# Create IGA beam elements (two elements for simplicity)
def create_iga_element(nodeA, nodeB):
    element = fea.ChIgaElementBeam()
    element.SetNodes(nodeA, nodeB)
    element.SetMaterial(material)
    element.SetArea(area)
    element.SetIyy(Iyy)
    element.SetIzz(Izz)
    element.SetJ(J)
    return element

mesh.AddElement(create_iga_element(node_start, node_center))
mesh.AddElement(create_iga_element(node_center, node_end))

sys.Add(mesh)

# Attach flywheel at center node
flywheel_mass = 5.0
flywheel_radius = 0.15
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,  # Ixx (rotational axis)
    0.25 * flywheel_mass * flywheel_radius**2,  # Iyy
    0.25 * flywheel_mass * flywheel_radius**2   # Izz
))
flywheel.SetPos(node_center.GetPos())
sys.Add(flywheel)

# Link flywheel to center node
link_flywheel = fea.ChLinkNodeBody()
link_flywheel.Initialize(node_center, flywheel)
sys.Add(link_flywheel)

# Create ground and motor system
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# Motor connection at beam start
start_body = chrono.ChBody()
start_body.SetPos(node_start.GetPos())
sys.Add(start_body)

link_start = fea.ChLinkNodeBody()
link_start.Initialize(node_start, start_body)
sys.Add(link_start)

# Rotational motor setup
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(ground, start_body, chrono.ChFrameD(node_start.GetPos()))
sys.Add(motor)

# Set constant angular velocity (20 rad/s)
omega = 20
ramp_function = chrono.ChFunction_Ramp(0, omega)
motor.SetAngleFunction(ramp_function)

# Visualization setup
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))

# FEM visualization for beam
fea_vis = irr.ChVisualShapeFEA(mesh)
fea_vis.SetFEMdataType(irr.ChVisualShapeFEA.DataType_SURFACE)
fea_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(fea_vis)

# Flywheel visualization
flywheel_vis = chrono.ChVisualShapeSphere(flywheel_radius)
flywheel_vis.SetColor(chrono.ChColor(1, 0, 0))
flywheel.AddVisualShape(flywheel_vis)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)