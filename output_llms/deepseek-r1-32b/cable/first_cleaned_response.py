importchrono
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyType(chrono.CHBODY_TYPE_STATIC)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
system.Add(ground)


num_elements = 10
length = 1.0
area = 0.001
density = 8000
young_modulus = 2e11
damping = 0.1


beam = chrono.ChBody()
beam.SetMass(10.0)
beam.SetBodyType(chrono.CHBODY_TYPE_FLEXIBLE)
beam.AddAsset(chrono.ChColorAsset(0.8, 0.2, 0.2))
beam.SetPos(chrono.ChVectorD(0, length/2, 0))
system.Add(beam)


element_mass = (density * area * length) / num_elements
for i in range(num_elements):
    node_mass = element_mass / 3
    element_length = length / num_elements
    
    
    node1 = chrono.ChBody()
    node1.SetMass(node_mass)
    node1.SetBodyType(chrono.CHBODY_TYPE_FLEXIBLE)
    node1.SetPos(chrono.ChVectorD(0, i*element_length, 0))
    system.Add(node1)
    
    node2 = chrono.ChBody()
    node2.SetMass(node_mass)
    node2.SetBodyType(chrono.CHBODY_TYPE_FLEXIBLE)
    node2.SetPos(chrono.ChVectorD(0, (i+1)*element_length, 0))
    system.Add(node2)
    
    
    cable = chrono.ChLinkANCF()
    cable.Initialize(node1, node2, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
    cable.Set_CableK(young_modulus * area / element_length)
    cable.Set_CableD(damping * young_modulus * area / element_length)
    system.Add(cable)


constraint = chrono.ChLinkPointPoint()
constraint.Initialize(ground, beam, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(constraint)


application = chrono.ChIrrApp(system, "Beam Simulation", chrono.CHIRR.dimension(800, 600))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))


while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(0.001)
    application.EndScene()
    
    
    if application.GetDevice().Key按下(chrono.CHKEY_ESCAPE):
        break


application.GetDevice().WaitOnExit()