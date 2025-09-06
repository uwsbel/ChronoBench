import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import math

# --- Simulation Setup ---
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Beam Parameters ---
beam_length = 1.0  # meters
beam_height = 0.1  # meters
beam_width = 0.05  # meters
youngs_modulus = 2.0e11  # Pascals (Steel)
shear_modulus = 8.0e10  # Pascals
density = 7850  # kg/m^3
damping = 0.01

# --- Create the Mesh ---
mesh = fea.ChMesh()

# --- Create Nodes ---
num_elements = 10
node_spacing = beam_length / num_elements

for i in range(num_elements + 1):
    x = i * node_spacing
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)

# --- Create Beam Elements ---
for i in range(num_elements):
    node1 = mesh.GetNode(i)
    node2 = mesh.GetNode(i + 1)
    element = fea.ChElementBeamEuler()
    element.SetNodes(node1, node2)
    
    # --- Section Properties ---
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_height, beam_width)
    section.SetYoungModulus(youngs_modulus)
    section.SetShearModulus(shear_modulus)
    section.SetRayleighDamping(damping)

    element.SetSection(section)
    mesh.AddElement(element)

# --- Create a Body to hold the mesh ---
body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(density * beam_height * beam_width * beam_length)
body.AddAsset(mesh)
body.EnableCollision(True)
sys.Add(body)

# --- Constraints ---
# Fixed constraint at one end
fixed_point = chrono.ChVector3d(0, 0, 0)
body.SetFixedPoint(fixed_point)

# --- Apply a force at the other end to cause buckling ---
force_magnitude = 1000  # Newtons
force_point = chrono.ChVector3d(beam_length, 0, 0)
force_direction = chrono.ChVector3d(0, -1, 0)  # Downward force

# Create a force sensor
force_sensor = chrono.ChForceSensor()
force_sensor.SetForceFunction(chrono.ChFunction_Const(chrono.ChVector3d(0, -force_magnitude, 0)))
force_sensor.SetPoint(force_point)
body.AddForce(force_sensor)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 2, -3))
vis.AddTypicalLights()

# --- Simulation Loop ---
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)