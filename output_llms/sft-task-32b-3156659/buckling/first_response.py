import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math

# Initialize the Chrono system with SMC solver for FEA
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # Disable gravity

# Create a finite element mesh and add it to the system
mesh = fea.ChMesh()
system.Add(mesh)

# Define beam geometry and material properties
beam_length = 1.0
num_elements = 10
element_length = beam_length / num_elements
young_modulus = 210e9  # Steel Young's modulus (Pa)
poisson_ratio = 0.3
density = 7800  # kg/m^3
section_width = 0.02  # m
section_height = 0.02  # m

# Create beam nodes
nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)
    nodes.append(node)

# Create beam elements between nodes
elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    
    # Define cross-section and material
    section = fea.ChBeamSectionCosseratSimple()
    section.SetYoungModulus(young_modulus)
    section.SetShearModulus(young_modulus / (2 * (1 + poisson_ratio)))
    section.SetAsRectangularSection(section_width, section_height)
    section.SetDensity(density)
    element.SetSection(section)
    
    mesh.AddElement(element)
    elements.append(element)

# Fix the first node
fixed_constraint = chrono.ChLinkPointFrame()
fixed_constraint.Initialize(nodes[0], None, chrono.ChFramed(nodes[0].Frame()))
system.Add(fixed_constraint)

# Apply a compressive force on the last node using a custom motor function
force = chrono.ChForce()
nodes[-1].AddForce(force)
force.SetMode(chrono.ChForce.FORCE)
force.SetDir(chrono.ChVector3d(-1, 0, 0))  # Negative x-direction

# Custom force function: linear increase over time
class CustomForce(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
        self.force_scale = 1000.0  # N/s

    def Get_y(self, x):
        return self.force_scale * x  # Force increases linearly with time

    def Get_dydx(self, x):
        return self.force_scale

    def Get_d2ydx2(self, x):
        return 0.0

custom_force_func = CustomForce()
force.SetFunct(custom_force_func)

# Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Beam Buckling Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 1), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()

# Configure solver and timestepper
system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# Simulation loop
time_step = 0.001
end_time = 5.0
time = 0.0

while vis.Run() and time < end_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    time += time_step