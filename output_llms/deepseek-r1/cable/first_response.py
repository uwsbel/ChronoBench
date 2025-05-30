import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# System setup
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity downward

# Create FEA mesh
mesh = fea.ChMesh()
system.Add(mesh)

# Cable section properties
section = fea.ChBeamSectionCable()
section.SetDiameter(0.05)       # 5 cm diameter
section.SetYoungModulus(1e7)    # Flexible material
section.SetDensity(1000)        # Density of water-like material
section.SetDamping(0.01)        # Add damping

# Geometry parameters
num_elements = 10
beam_length = 2.0               # 2 meters
dx = beam_length / num_elements # Element length

# Create nodes
nodes = []
for i in range(num_elements + 1):
    x_pos = i * dx
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x_pos, 0, 0), 
                            chrono.ChVectorD(1, 0, 0))  # Initial direction along X
    mesh.AddNode(node)
    nodes.append(node)

# Fix first node to ground using a point frame constraint
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

link = fea.ChLinkPointFrame()
link.Initialize(nodes[0], ground)
system.Add(link)

# Create cable elements between nodes
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)

# Visualization settings
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MX)
vis_mesh.SetColorscaleMinMax(-0.4, 0.4)
vis_mesh.SetSmoothFaces(True)
mesh.AddVisualizationFEA(vis_mesh)

# Irrlicht setup
application = chronoirr.ChIrrApp(system, "ANCF Cable Beam", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.5, 3.0))  # Adjust camera position
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop settings
application.SetTimestep(0.001)
application.SetTryRealtime(True)

# Simulation execution
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()