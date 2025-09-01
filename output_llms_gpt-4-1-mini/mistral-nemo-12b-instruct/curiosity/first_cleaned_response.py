import pychrono as chrono
import numpy as np


sim = chrono.ChSimulation()
sim.SetTimestep(0.01)  
sim.SetMaxSteps(1000)  


sys = chrono.ChSystemSMC()
sim.AddSystem(sys)


mat_ground = chrono.ChMaterialSurfaceNSC()
mat_ground.SetFriction(0.6)  
mat_ground.SetYoungModulus(2e7)  
mat_ground.SetYieldStrength(2e6)  



ground = chrono.ChBodyEasyBox(10, 10, 0.1, mat_ground, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.05))  
sys.Add(ground)


box_mat = chrono.ChMaterialSurfaceNSC()
box_mat.SetFriction(0.4)  
box_mat.SetYoungModulus(1e7)  
box_mat.SetYieldStrength(1e6)  

box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, box_mat, True, True)
box.SetPos(chrono.ChVectorD(1, 0, 0.25))  
box.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 4))  
sys.Add(box)


sphere_mat = chrono.ChMaterialSurfaceNSC()
sphere_mat.SetFriction(0.8)  
sphere_mat.SetYoungModulus(2e7)  
sphere_mat.SetYieldStrength(2e6)  

sphere = chrono.ChBodyEasySphere(0.25, sphere_mat, True, True)
sphere.SetPos(chrono.ChVectorD(0, 1, 0.5))  
sys.Add(sphere)


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetDir(chrono.VECT_X)
force.SetMagnitude(5)
force.SetPos(chrono.ChVectorD(0, 1, 0.5))
sphere.AddForce(force)



contact = chrono.ChLinkMateContact()
contact.Initialize(box, sphere)
sys.Add(contact)


contact = chrono.ChLinkMateContact()
contact.Initialize(ground, sphere)
sys.Add(contact)


contact = chrono.ChLinkMateContact()
contact.Initialize(ground, box)
sys.Add(contact)


sim.Run()


print("Final position of the box:", box.GetPos())
print("Final position of the sphere:", sphere.GetPos())