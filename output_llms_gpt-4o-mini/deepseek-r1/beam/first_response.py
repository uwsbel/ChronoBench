import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.fea as fea
import math

# 1. Initialize the environment
chrono.SetChronoDataPath("path/to/chrono/data/")  # Set appropriate data path
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Optional: Disable gravity

# 2. Create a mesh for finite elements
mesh = fea.ChMesh()
system.Add(mesh)

# 3. Create material for beams
density = 7850  # kg/m³ (steel)
E = 2e11        # Pa (Young's modulus)
nu = 0.3        # Poisson's ratio
beam_material = fea.ChContinuumElastic()
beam_material.Set_E(E)
beam_material.Set_G(E/(2*(1+nu)))
beam_material.Set_density(density)

# 4. Create beam section (rectangular cross-section)
width = 0.05   # m
height = 0.1   # m
beam_section = fea.ChBeamSectionEuler()
beam_section.SetAsRectangularSection(width, height)
beam_section.SetMaterial(beam_material)

# 5. Create nodes and elements
num_elements = 10
beam_length = 2.0  # meters
node_spacing = beam_length / num_elements

# Create nodes along x-axis
nodes = []
for i in range(num_elements + 1):
    position = chrono.ChVectorD(node_spacing * i, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(position))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

# Create beam elements between nodes
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(beam_section)
    mesh.AddElement(element)

# 6. Apply boundary conditions (fix first node)
nodes[0].SetFixed(True)

# 7. Apply load to last node
force = chrono.ChVectorD(0, -5000, 0)  # 5000 N downward
nodes[-1].SetForce(force)

# 8. Set up visualization
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Beam FEM Demo')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(3, 2, -1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Visualization of beam elements
beam_vis = fea.ChVisualShapeFEA(mesh)
beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_vis.SetColorscaleMinMax(-1000, 1000)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_vis)

# 9. Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    # Print tip displacement during simulation
    tip_pos = nodes[-1].GetPos()
    print(f"Tip displacement: {tip_pos.y:.4f} m")