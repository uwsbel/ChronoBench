import pychrono as chrono


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()



sphere = chrono.ChBodyEasySphere(1.0, 1.0)  
sphere.SetPos(chrono.ChVectorD(0, 0, 1))  
system.Add(sphere)


box = chrono.ChBodyEasyBox(0.5, 0.5, 1.0, 2.0)  
box.SetPos(chrono.ChVectorD(0, 0, 0))  
system.Add(box)



force = chrono.ChVectorD(0, 0, -10)  
sphere.AddForce(force)


sphere.SetFriction(0.5, 0.5)  






system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  
system.DoStepDynamics(0.01)