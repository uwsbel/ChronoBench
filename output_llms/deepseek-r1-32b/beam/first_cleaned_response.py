import pychrono as chrono
import pychrono_irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chronoirr.ChIrrApp(system, "Beam Finite Element Demo", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalLight()
application.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
application.SetCameraRotation(chrono.ChVectorD(-60, 45, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChPlane()
ground.AddShape(ground_shape)
system.Add(ground)


num_elements = 10
length = 2.0
element_length = length / num_elements


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


E = 2.0e11  
G = 8.0e10  
A = 0.01    
I = 0.001   

for i in range(num_elements):
    node1 = nodes[i]
    node2 = nodes[i+1]
    
    
    beam = chrono.ChBeam()
    beam.SetNodes(node1, node2)
    beam.SetYoungModulus(E)
    beam.SetShearModulus(G)
    beam.SetCrossSectionArea(A)
    beam.SetMomentInertia(I)
    system.Add(beam)
    
    
    capsule = chrono.ChCapsule()
    capsule.SetHeight(element_length)
    capsule.SetRadius(0.02)
    beam.AddVisualShape(capsule)


force_node = nodes[-1]
force = chrono.ChForce()
force_node.AddForce(force)
force.SetF(chrono.ChVectorD(-1000, 0, 0))  


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    
   chrono.ChSleep(0.001)