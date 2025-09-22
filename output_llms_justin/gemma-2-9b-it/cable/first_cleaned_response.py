import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


node1 = chrono.ChBodyEasySphere(0.1, 1000)
node1.SetPos(chrono.ChVectorD(0, 0, 1))
system.Add(node1)

node2 = chrono.ChBodyEasySphere(0.1, 1000)
node2.SetPos(chrono.ChVectorD(1, 0, 1))
system.Add(node2)


cable1 = chrono.ChLinkCable(node1, node2)
cable1.SetMaterial(chrono.ChMaterialCableLinear(1e5, 0.01, 0.01))  
system.Add(cable1)


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vis = chronoirr.ChIrrApp(system, 'Beam Simulation')
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
vis.AssetBindAll()


while vis.Run():
    system.DoStepDynamics(0.01)
    vis.Render()