import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize the Chrono system
system = chrono.ChSystemSMC()

# 2. Create a mesh and add it to the system
mesh = fea.ChMesh()
system.Add(mesh)

# 3. Define beam section properties
beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetYoungModulus(2.1e7)      # [Pa]
beam_section.SetGwithPoissonRatio(0.3)   # Poisson ratio
beam_section.SetDensity(7800)            # [kg/m^3]
beam_section.SetAsRectangularSection(0.02, 0.01)  # width, height [m]

# 4. Create nodes and beams
n_nodes = 5
beam_length = 1.0
node_spacing = beam_length / (n_nodes - 1)
nodes = []

# Create nodes along the X axis
for i in range(n_nodes):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * node_spacing, 0, 0)))
    node.SetMass(0.1)
    mesh.AddNode(node)
    nodes.append(node)

# Fix the first node to ground
nodes[0].SetFixed(True)

# Create beam elements between consecutive nodes
for i in range(n_nodes - 1):
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])
    beam.SetSection(beam_section)
    mesh.AddElement(beam)

# 5. Add a load at the tip node (last node)
load_container = chrono.ChLoadContainer()
system.Add(load_container)

# Apply a downward force at the last node
tip_force = chrono.ChVectorD(0, -100, 0)  # [N]
tip_load = fea.ChLoadNodeForce(nodes[-1], tip_force)
load_container.Add(tip_load)

# 6. Visualization assets for the mesh
# Beam visualization
beam_visual = fea.ChVisualizationFEAmesh(mesh)
beam_visual.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_visual.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
beam_visual.SetSymbolsThickness(0.006)
beam_visual.SetSymbolsScale(1)
mesh.AddAsset(beam_visual)

# Deformation visualization
beam_deform = fea.ChVisualizationFEAmesh(mesh)
beam_deform.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_deform.SetColorscaleMinMax(-0.4, 0.4)
beam_deform.SetSmoothFaces(True)
mesh.AddAsset(beam_deform)

# 7. Set up the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "PyChrono Beam Finite Element Demo", chronoirr.dimension2du(1024,768))
application.AddTypicalSky()
application.AddTypicalCamera(chrono.ChVectorD(0.5, 0.2, 1.0), chrono.ChVectorD(0.5, 0, 0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# 8. Simulation loop
application.SetTimestep(0.001)
while(application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()