import pychrono as chrono
from pychrono import fea, irrlicht as chronoirr

# Initialize the PyChrono system
sys = chrono.ChSystemSMC()

# Define beam and flywheel parameters
beam_L = 1.0  # Beam length
beam_wy = 0.05  # Beam width (y-axis)
beam_wz = 0.05  # Beam height (z-axis)
flywheel_mass = 10.0  # Flywheel mass
flywheel_radius = 0.1  # Flywheel radius
flywheel_length = 0.2  # Flywheel thickness along beam axis

# Create beam section properties
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(200e9)  # Steel Young's modulus
msection.SetShearModulus(80e9)  # Steel Shear modulus
msection.SetDensity(7800)  # Steel density

# Create FEA nodes
node_start = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
node_center = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L/2, 0, 0)))
node_end = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))

# Create FEA mesh and elements
mesh = fea.ChMesh()
mesh.AddNode(node_start)
mesh.AddNode(node_center)
mesh.AddNode(node_end)

# Add beam elements
element1 = fea.ChElementBeamEuler()
element1.SetNodes(node_start, node_center)
element1.SetSection(msection)
mesh.AddElement(element1)

element2 = fea.ChElementBeamEuler()
element2.SetNodes(node_center, node_end)
element2.SetSection(msection)
mesh.AddElement(element2)

sys.Add(mesh)

# Create flywheel body
flywheel_body = chrono.ChBody()
flywheel_body.SetMass(flywheel_mass)
flywheel_inertia = chrono.ChVector3d(
    0.5 * flywheel_mass * flywheel_radius**2,
    0.5 * flywheel_mass * flywheel_radius**2,
    flywheel_mass * (flywheel_radius**2/4 + flywheel_length**2/12)
)
flywheel_body.SetInertiaXX(flywheel_inertia)
flywheel_body.SetPos(node_center.GetPos())
flywheel_body.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
sys.Add(flywheel_body)

# Link flywheel to beam's center node
link_flywheel = chrono.ChLinkNodeFrame()
link_flywheel.Initialize(flywheel_body, node_center)
sys.Add(link_flywheel)

# Create motor components
motor_base = chrono.ChBody()
motor_base.SetFixed(True)
motor_base.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(motor_base)

motor_rotor = chrono.ChBody()
motor_rotor.SetMass(1.0)
motor_rotor.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
motor_rotor.SetPos(node_start.GetPos())
sys.Add(motor_rotor)

# Revolute joint between motor_base and motor_rotor
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(motor_base, motor_rotor, chrono.ChCoordsysD(node_start.GetPos(), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add(rev_joint)

# Link motor_rotor to beam's start node
link_rotor = chrono.ChLinkNodeFrame()
link_rotor.Initialize(motor_rotor, node_start)
sys.Add(link_rotor)

# Motor to drive the beam
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(motor_rotor, motor_base, chrono.ChCoordsysD(node_start.GetPos(), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add(motor)

# Set constant angular speed (2 rad/s)
speed_func = chrono.ChFunction_Const(2.0)
motor.SetSpeedFunction(speed_func)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 3))
vis.AddTypicalLights()

# FEM visualization for beam
mesh_shape = chrono.ChVisualShapeFEA(mesh)
mesh_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM)
mesh_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
mesh.AddVisualShapeFEA(mesh_shape)

# Flywheel visualization
flywheel_shape = chrono.ChVisualShapeCylinder(flywheel_radius, flywheel_length)
flywheel_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
flywheel_body.AddVisualShape(flywheel_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)