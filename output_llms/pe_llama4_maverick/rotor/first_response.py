import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Create a Chrono system with NSC (Non-Smooth Contact) dynamics
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

# Create a FEM mesh
mesh = fea.ChMesh()

# Define the beam section properties
beam_wy = 0.1  # width in y direction
beam_wz = 0.2  # width in z direction
beam_L = 10   # length of the beam
order = 3     # order of the IGA beam

# Create an IGA beam element
element = fea.ChElementBeamIGA()
element.SetOrder(order)
element.SetNodesCubic()

# Define the beam section
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(2.1e11)
msection.SetGshearModulus(8.0769231e10)
msection.SetBeamRaleyghDamping(0.01)

# Create a set of nodes for the IGA beam
num_nodes = 10
for i in range(num_nodes):
    x = beam_L * i / (num_nodes - 1)
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)

# Create IGA beam elements
for i in range(num_nodes - order):
    element = fea.ChElementBeamIGA()
    element.SetNodes(list(mesh.GetNode(j) for j in range(i, i + order + 1)))
    element.SetSection(msection)
    mesh.AddElement(element)

# Add the mesh to the system
sys.Add(mesh)

# Create a flywheel body
flywheel = chrono.ChBodyEasyCylinder(0.2, 0.5, 7800, True, True, True)
flywheel.SetPos(chrono.ChVector3d(beam_L / 2, 0, 0))
sys.Add(flywheel)

# Constrain the flywheel to the beam
constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
constraint.Initialize(flywheel, mesh.GetNode(int(num_nodes / 2)), False)
sys.Add(constraint)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(mesh.GetNode(0), mesh.GetNode(1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_PI))  # 0.5 rotations per second

# Add the motor to the system
sys.Add(motor)

# Create an Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))

# Enable FEM visualization
fea.ChVisualizationFEAmesh().SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
fea.ChVisualizationFEAmesh().SetColorscaleMinMax(-0.5, 0.5)
fea.ChVisualizationFEAmesh().SetSmoothFaces(True)
mesh.AddAsset(fea.ChVisualizationFEAmesh())

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)