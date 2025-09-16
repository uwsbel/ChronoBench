import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht

# Initialize the PyChrono system with non-smooth contact dynamics
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in Y-direction

# Create FEA mesh for the beam
mesh = fea.ChMesh()

# Beam parameters
L = 1.0  # Length of the beam
num_elements = 10  # Number of elements
radius = 0.01  # Radius of the cable
E = 2e5  # Young's modulus (Pa)
density = 1000  # Density (kg/m^3)
area = chrono.CH_C_PI * radius**2  # Cross-sectional area
I = chrono.CH_C_PI * radius**4 / 4  # Area moment of inertia

# Create nodes along the beam
for i in range(num_elements + 1):
    pos = chrono.ChVectorD(L * i / num_elements, 0, 0)
    node = fea.ChNodeFEAxyz(pos)
    mesh.AddNode(node)

# Create ANCF cable elements
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i+1))
    element.SetDiameter(2 * radius)
    element.SetMaterialYoungModulus(E)
    element.SetArea(area)
    element.SetI(I)
    element.SetDensity(density)
    mesh.AddElement(element)

# Add the mesh to the system
system.Add(mesh)

# Fix the first node to the ground using a generic constraint
node0 = mesh.GetNode(0)
constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
constraint.Initialize(node0, system.GetGroundBody(), False, chrono.ChFrameD(), chrono.ChFrameD())
system.Add(constraint)

# Set up Irrlicht visualization
application = irrlicht.ChIrrApp(system, 'ANCF Beam Simulation', irrlicht.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irrlicht.vector3df(0.5, 0.5, -1.5))  # Adjusted camera position

# Add FEA mesh visualization assets
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetShowNodes(True)
vis_mesh.SetNodeSize(0.01)
mesh.AddAsset(vis_mesh)

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1e-4)  # Smaller timestep for stability