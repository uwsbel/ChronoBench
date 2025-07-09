import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for the beam
mesh = fea.ChMesh()

# Define beam section properties
beam_length = 10.0
beam_wy = 0.1
beam_wz = 0.1
E = 0.01e9  # Young's modulus
nu = 0.3    # Poisson's ratio
G = E / (2 * (1 + nu))  # Shear modulus

# Create beam section
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(E)
msection.SetShearModulus(G)
msection.SetRayleighDamping(0.000)

# Create nodes for the beam
num_elements = 10
node_interval = beam_length / num_elements
nodes = []
for i in range(num_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(i * node_interval, 0, 0)))
    mesh.AddNode(node)
    nodes.append(node)

# Create beam elements
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(msection)
    mesh.AddElement(element)

# Add mesh to the system
sys.Add(mesh)

# Constrain the first node
nodes[0].SetFixed(True)

# Create a motor to apply a compressive force
body_motor = chrono.ChBody()
sys.Add(body_motor)
body_motor.SetPos(nodes[-1].GetPos())

# Create a motor joint
link_motor = chrono.ChLinkMateGeneric()
link_motor.Initialize(nodes[-1].GetBody(), body_motor, False, nodes[-1].GetFrame_COG(), nodes[-1].GetFrame_COG())
sys.AddLink(link_motor)

# Define a custom motor function
class MotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        return -0.1 * x  # Compressive force

motor_function = MotorFunction()
body_motor.SetForce(chrono.ChVector3d(1, 0, 0))  # Initial force direction
body_motor.SetPos_dt(chrono.ChVector3d(motor_function.Get_y(0), 0, 0))  # Initial velocity

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 3, 4))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()