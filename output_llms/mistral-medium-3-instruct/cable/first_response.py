import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set this to your Chrono data directory

# Create a physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Create the ANCF cable beam
# Parameters for the beam
num_elements = 10  # Number of elements
length = 2.0       # Total length of the beam (meters)
radius = 0.05      # Radius of the beam (meters)
density = 7800     # Density (kg/m^3)
E = 2e11           # Young's modulus (Pa)

# Create the ANCF cable mesh
my_mesh = chrono.fea.ChMesh()

# Create nodes (positions along x-axis)
nodes = []
for i in range(num_elements + 1):
    node = chrono.fea.ChNodeFEAxyzD(length * i / num_elements, 0, 0)
    node.SetMass(0)  # Mass will be handled by elements
    my_mesh.AddNode(node)
    nodes.append(node)

# Create ANCF cable elements
for i in range(num_elements):
    element = chrono.fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i+1])

    # Set material properties
    section = chrono.fea.ChBeamSectionCable()
    section.SetDiameter(radius * 2)
    section.SetYoungModulus(E)
    section.SetDensity(density)
    element.SetSection(section)

    my_mesh.AddElement(element)

# Add the mesh to the system
system.Add(my_mesh)

# 3. Set boundary conditions (hinged end at first node)
hinge = chrono.ChLinkLockLock()
hinge.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(hinge)

# 4. Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# Customize visualization of nodes and elements
for node in nodes:
    node_vis = chrono.fea.ChVisualizationFEAmesh(my_mesh)
    node_vis.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_NODES)
    node_vis.SetFEMglyphType(chrono.fea.ChVisualizationFEAmesh.E_GLYPH_SPHERE)
    node_vis.SetFEMglyphSize(0.02)
    node_vis.SetSymbolsThickness(0.01)
    vis.AddVisualizationFEAmesh(node_vis)

# Simulation parameters
time_step = 0.001
end_time = 5.0

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Print current time and position of the free end
    current_time = system.GetChTime()
    if current_time > end_time:
        break

    if int(current_time * 10) % 10 == 0:  # Print every 0.1 seconds
        print(f"Time: {current_time:.2f}s")
        print(f"Free end position: {nodes[-1].GetPos().x:.3f}, {nodes[-1].GetPos().y:.3f}, {nodes[-1].GetPos().z:.3f}")