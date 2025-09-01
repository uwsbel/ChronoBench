import pychrono as chrono


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()



sphere = chrono.ChBodyEasySphere(1.0, 1.0)  
sphere.SetPos(chrono.ChVectorD(0, 0, 1))  
system.Add(sphere)


box = chrono.ChBodyEasyBox(0.5, 0.5, 1.0, 2.0)  
box.SetPos(chrono.ChVectorD(0, 0, 0))  
system.Add(box)



system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


contact_mat = chrono.ChMaterialSurfaceNSC()
contact_mat.SetFriction(0.5)
contact_mat.SetRestitution(0.1)


sphere.GetMaterialSurface().SetMaterial(contact_mat)
box.GetMaterialSurface().SetMaterial(contact_mat)






system.DoStepDynamics(0.01)