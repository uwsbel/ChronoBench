import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize system and set gravity
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in -Y direction

# 2. Create ground body and marker for hinge
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetBodyFixed(True)  # Fix the ground body

# 3. Create FEA mesh
mesh = fea.ChMesh()
system.Add(mesh)

# Beam parameters
beam_length = 2.0          # Total beam length (m)
num_elements = 10          # Number of ANCF elements
node_spacing = beam_length / num_elements
section_area = 0.01        # Cross-section area (m²)
density = 1000             # Density (kg/m³)
young_modulus = 2e9        # Young's modulus (Pa)

# 4. Create nodes and elements
nodes = []
for i in range(num_elements + 1):
    # Position along X-axis
    position = chrono.ChVectorD(i * node_spacing, 0, 0)
    
    # Create node with position and direction vector (initial slope)
    node = fea.ChNodeFEAxyzD(position, chrono.ChVectorD(0, 0, 1))
    mesh.AddNode(node)
    nodes.append(node)
    
    # Fix first node to ground using hinge constraint
    if i == 0:
        constraint = fea.ChLinkPointFrame()
        constraint.Initialize(node, ground)
        mesh.AddLink(constraint)

# 5. Create ANCF cable elements between nodes
for i in range(num_elements):
    element = fea.ChElementBeamANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSectionDimensions(section_area, 0.1)  # Height unused in cable
    element.SetMaterial(fea.ChBeamSectionCable(density, young_modulus))
    element.SetAlphaDamp(0.01)  # Damping coefficient
    mesh.AddElement(element)

# 6. Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.5, 2))
vis.AddTypicalLights()

# 7. Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)