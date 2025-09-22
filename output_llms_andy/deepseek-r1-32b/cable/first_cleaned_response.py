importchrono
import math
import time


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chrono.ChIrrApp(system, "ANCF Cable Beam Simulation", 1024, 768)
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))


num_elements = 20
num_nodes = num_elements + 1
beam_length = 4.0
young_modulus = 2.1e11
cross_area = 0.001
density = 7800
mass_per_length = density * cross_area


nodes = []
elements = []


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.AddBody(ground)


for i in range(num_nodes):
    node = chrono.ChBody()
    node.SetMass(0.1)  
    node.SetPos(chrono.ChVectorD(beam_length * i / num_elements, 0, 0))
    system.AddBody(node)
    nodes.append(node)


constraint = chrono.ChLinkLock()
constraint.Initialize(nodes[0], ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(constraint)


for i in range(num_elements):
    element = chrono.ChLinkANCF()
    element.Initialize(nodes[i], nodes[i+1], chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
    element.SetProperties(young_modulus, cross_area, 0, 0)
    element.SetStiffness(1e6)  
    element.SetDamping(100)
    system.AddLink(element)
    elements.append(element)

    
    segment_length = beam_length / num_elements
    segment_mass = mass_per_length * segment_length
    nodes[i].SetMass(segment_mass)
    nodes[i+1].SetMass(segment_mass)


for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.SetRadius(0.1)
    node.AddAsset(sphere)


coordsys = chrono.ChCoordsysD()
coordsys.SetPos(chrono.ChVectorD(0, 0, 0))
application.GetScene().AddAsset(coordsys)


application.Begin()
while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    
    
    system.DoStepDynamics(0.001)
    
    
    if chrono.ChKey(chrono.CH_KEY_ESCAPE).IsPressed():
        break
    
    application.EndScene()
application.End()