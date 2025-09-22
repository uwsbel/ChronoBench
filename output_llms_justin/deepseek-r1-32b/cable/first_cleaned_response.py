importchrono
import math
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chrono.ChIrrApp(system, "ANCF Cable Beam Simulation", chrono.CH.dimension(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))


num_elements = 20
num_nodes = num_elements + 1
length = 4.0
young_modulus = 2.1e11
area = 0.001
density = 7800
mass_per_length = density * area


nodes = []
elements = []


for i in range(num_nodes):
    node = chrono.ChBody()
    node.SetMass(1.0)  
    node.SetPos(chrono.ChVectorD(0, length/num_elements * i, 0))
    system.Add(node)
    nodes.append(node)


constraint = chrono.ChLinkLock()
constraint.Initialize(nodes[0], chrono.ChBodyFrame(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))
system.Add(constraint)


for i in range(num_elements):
    element_length = length / num_elements
    stiffness = young_modulus * area / element_length
    damping = 0.01 * stiffness  
    
    cable = chrono.ChLinkANCF()
    cable.Initialize(nodes[i], nodes[i+1], chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,0,0))
    cable.SetProperties(stiffness, damping, mass_per_length * element_length, 0, 0)
    system.Add(cable)
    elements.append(cable)


for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.SetRadius(0.05)
    node.AddAsset(sphere)


coordsys = chrono.ChCoordsysD()
coordsys.SetPos(chrono.ChVectorD(0,0,0))
application.GetScene().AddAsset(coordsys)


application.BeginRun()
step_count = 0
while step_count < 1000:
    application.DoStep()
    if application.GetDevice().keyPress(chrono.CH_KEY_ESCAPE):
        break
    
    
    for i, node in enumerate(nodes):
        pos = node.GetPos()
        print(f"Node {i} position: ({pos.x}, {pos.y}, {pos.z})")
    
    step_count += 1
application.EndRun()