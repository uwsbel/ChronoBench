import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  
system = chrono.ChSystemNSC()  
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  



ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)  
system.Add(ground)


rover = chrono.ChBodyEasySphere(0.5, 10)  
rover.SetPos(chrono.ChVectorD(0, 0, 0.5))
system.Add(rover)






ground.SetCollide(True)
rover.SetCollide(True)


vis = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation')


cam = vis.GetActiveCamera()
cam.SetPos(chrono.ChVectorD(5, 5, 2))
cam.SetTarget(chrono.ChVectorD(0, 0, 0))


vis.AddLight(chrono.ChVectorD(10, 10, 10))  
vis.SetShadowMapEnabled(True)





vis.Start()