import pychrono as chrono
import pychrono_irrlicht as chronoirr

# Create the Chrono::Engine system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Create the Irrlicht application for visualization
application = chronoirr.ChIrrApp(system, "Beam Finite Element Demo", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalLight()
application.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
application.SetCameraRotation(chrono.ChVectorD(-60, 45, 0))

# Create ground plane
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChPlane()
ground.AddShape(ground_shape)
system.Add(ground)

# Beam parameters
num_elements = 10
length = 2.0
element_length = length / num_elements

# Create beam nodes
nodes = []
for i in range(num_elements + 1):
    node = chrono.ChBody()
    node.SetMass(0.1)
    node.SetPos(chrono.ChVectorD(i * element_length, 0, 0))
    node_shape = chrono.ChSphere()
    node_shape.SetRadius(0.05)
    node.AddShape(node_shape)
    system.Add(node)
    nodes.append(node)

# Create beam elements
E = 2.0e11  # Young's modulus
G = 8.0e10  # Shear modulus
A = 0.01    # Cross-sectional area
I = 0.001   # Moment of inertia

for i in range(num_elements):
    node1 = nodes[i]
    node2 = nodes[i+1]
    
    # Create beam element
    beam = chrono.ChBeam()
    beam.SetNodes(node1, node2)
    beam.SetYoungModulus(E)
    beam.SetShearModulus(G)
    beam.SetCrossSectionArea(A)
    beam.SetMomentInertia(I)
    system.Add(beam)
    
    # Add visualization
    capsule = chrono.ChCapsule()
    capsule.SetHeight(element_length)
    capsule.SetRadius(0.02)
    beam.AddVisualShape(capsule)

# Add force to one end
force_node = nodes[-1]
force = chrono.ChForce()
force_node.AddForce(force)
force.SetF(chrono.ChVectorD(-1000, 0, 0))  # Apply horizontal force

# Run simulation
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    # Optional: Sleep to control simulation speed
   chrono.ChSleep(0.001)