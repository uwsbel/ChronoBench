importchrono
import math

# Initialize the Chrono system
system = chrono.ChSystem()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity acceleration

# Create Irrlicht application for visualization
application = chrono.ChIrrApp(system, "ANCF Beam Simulation", chrono.CHDimension(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.SetCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))

# Parameters for the beam
num_elements = 10
element_length = 0.5
total_length = num_elements * element_length
mass_per_unit_length = 1.0
cross_section_area = 0.01
young_modulus = 2e5

# Create nodes
nodes = []
for i in range(num_elements + 1):
    node = chrono.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(i * element_length, 0, 0))
    system.Add(node)
    nodes.append(node)

# Create cable elements
for i in range(num_elements):
    cable = chrono.ChElementCableANCF()
    cable.SetNodes(nodes[i], nodes[i+1])
    cable.SetProperties(mass_per_unit_length, cross_section_area, young_modulus)
    system.Add(cable)

# Fix the first node (hinge constraint)
nodes[0].GetBody().SetMass(0)
nodes[0].GetBody().SetBodyFixed(True)

# Create a mesh for visualization
mesh = chrono.ChTriangleMeshShape()
for i in range(num_elements):
    # Create triangles for visualization
    p1 = nodes[i].GetPos()
    p2 = nodes[i+1].GetPos()
    # Add points to the mesh
    mesh.AddPoint(p1)
    mesh.AddPoint(p2)

# Create a body to attach the mesh
body = chrono.ChBody()
body.SetMaterialSurface(chrono.ChMaterialSurface())
body.GetCollisionModel().SetFamily(chrono.COLLIDE_FAMILY_GHOST)
body.AddAsset(mesh)
system.Add(body)

# Add a fixed body for ground reference
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBoxShape()
ground_shape.SetDimensions(chrono.ChVectorD(total_length, 1, 1))
ground.AddAsset(ground_shape)
ground.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
system.Add(ground)

# Compile the mesh
mesh.Compile()

# Simulation loop
application.AssetSetShow(chrono.CHAABB_TYPE, False)
application.AssetSetShow(chrono.CHNODE_TYPE, True)
application.AssetSetShow(chrono.CHELEMENT_TYPE, True)

while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    # Update simulation time
    system.DoStepDynamics(0.001)
    
    # Print time step (optional)
    if system.GetChTime() % 0.1 < 0.001:
        print(f"Time: {system.GetChTime():.3f}s")