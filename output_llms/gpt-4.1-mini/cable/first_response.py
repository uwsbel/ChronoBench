import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Optional: set data path if needed

system = chrono.ChSystemNSC()

# Set gravity (downwards in Z direction)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Add the required physical systems and objects as specified.

# Create a mesh to hold ANCF cable elements
mesh = fea.ChMesh()

# Beam parameters
beam_length = 2.0  # meters
beam_radius = 0.02  # meters
beam_density = 7850  # steel kg/m^3
E = 2.1e11         # Young's modulus in Pa
nu = 0.3           # Poisson's ratio
G = E / (2 * (1 + nu))

# Discretization
num_elements = 10
num_nodes = num_elements + 1
element_length = beam_length / num_elements

# Create nodes
# ANCF cable nodes have coordinates and gradients: position + slope (2D in direction of beam)
# For simplicity, place beam along the X-axis starting at origin, Z is vertical.

nodes = []
for i in range(num_nodes):
    x = i * element_length
    y = 0
    z = 0
    # For ANCF cable elements, each node has position (X,Y,Z) and unit slope vector components:
    # In PyChrono, ChNodeFEAxyzD has position and a gradient vector (director) that defines slope.
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, y, z), chrono.ChVectorD(0, 0, 1))
    mesh.AddNode(node)
    nodes.append(node)

# Create ANCF cable elements connecting the nodes
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    # Set element dimensions and mechanical properties
    element.SetDimensions(beam_radius, element_length)
    element.SetDensity(beam_density)
    element.SetYoungModulus(E)
    element.SetGshearModulus(G)
    mesh.AddElement(element)

# Add the mesh of elements to the system
system.Add(mesh)

# 2.a Fix one end (hinged to ground)
# We'll fix node 0 in all DOFs: position and slope fixed (to simulate a hinge we fix node position but allow rotation
# For a perfectly hinged support, allow rotation but no translation.
# Since the slope vector controls orientation, we allow slope to vary, but fix position.

# Fix node 0 position in all directions (X,Y,Z)
nodes[0].SetFixed(True)
# The slope (director) is free, so no fixing slope degrees of freedom (default).

# 3. Set necessary default parameters such as positions, forces, and interactions.
# Gravity already set in system.

# Irrlicht application for visualization
application = chronoirr.ChIrrApp(system, "ANCF Cable Beam", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(2, -4, 1.5))

# Visualization assets for mesh nodes (to see nodal positions)
for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.025
    node.AddAsset(sphere)

# Visualization assets for mesh (to see elements deformation)
mesh_shape = fea.ChVisualizationFEAmesh(mesh)
mesh_shape.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_ELEM)
mesh_shape.SetColorscaleMinMax(-1.0, 1.0)
mesh_shape.SetShrinkElements(True, 0.85)
mesh_shape.SetWireframe(False)
mesh.AddAsset(mesh_shape)

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(1e-3)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    # Optionally: extract and print nodal positions (commented out for performance)
    # for i, node in enumerate(nodes):
    #     pos = node.GetPos()
    #     print(f"Node {i}: {pos}")

    application.DoStep()
    application.EndScene()