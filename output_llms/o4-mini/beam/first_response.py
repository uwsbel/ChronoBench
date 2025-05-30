import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# ------------------------------------------------------------------------------
# 1) Initialize the Chrono system
# ------------------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))   # gravity

# ------------------------------------------------------------------------------
# 2) Create an FEA mesh and define beam section & material properties
# ------------------------------------------------------------------------------
mesh = fea.ChMesh()

# Define a circular beam section (Euler beam)
section = chrono.ChBeamSectionEuler()
E = 2e11                # Young's modulus [Pa]
rho = 7800              # density [kg/m^3]
nu = 0.3                # Poisson ratio (for G)
section.SetYoungModulus(E)
section.SetGwithPoissonRatio(E / (2*(1+nu)), nu)
section.SetDensity(rho)
section.SetAsCircularSection(0.02)  # radius = 0.02 m

# ------------------------------------------------------------------------------
# 3) Create nodes along the X axis and add them to the mesh
# ------------------------------------------------------------------------------
num_elems   = 6
elem_length = 0.5
nodes = []

for i in range(num_elems + 1):
    x = i * elem_length
    node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, 0))
    node.SetMass(0)           # lumped mass = 0 (mass comes from section)
    mesh.AddNode(node)
    nodes.append(node)

# Fix the first node (cantilever root)
nodes[0].SetFixed(True)

# ------------------------------------------------------------------------------
# 4) Create beam elements connecting the nodes
# ------------------------------------------------------------------------------
for i in range(num_elems):
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])    # assign start/end nodes
    beam.SetSection(section)               # assign section & material
    mesh.AddElement(beam)

# ------------------------------------------------------------------------------
# 5) Add the mesh to the system
# ------------------------------------------------------------------------------
system.Add(mesh)

# ------------------------------------------------------------------------------
# 6) Create visualization assets for the mesh
# ------------------------------------------------------------------------------
# This will draw each beam element in wireframe with a default color
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetDefaultMeshColor(chrono.ChColor(0.2, 0.5, 0.8))
mesh.AddAsset(vis_mesh)

# ------------------------------------------------------------------------------
# 7) Create the Irrlicht application for real‐time rendering
# ------------------------------------------------------------------------------
application = irr.ChIrrApp(system, "PyChrono Beam FEA Demo", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(3,1.5,3))

# Bind and update all assets (must be done before simulation)
application.AssetBindAll()
application.AssetUpdateAll()

# ------------------------------------------------------------------------------
# 8) Simulation loop
# ------------------------------------------------------------------------------
application.SetTimestep(1e-3)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()