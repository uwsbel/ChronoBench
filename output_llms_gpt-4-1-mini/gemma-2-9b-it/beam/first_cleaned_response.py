import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 0.1)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
ground.SetBodyFixed(True)
system.Add(ground)


beam_material = chrono.ChMaterialSurfaceNSC()
beam_material.SetFriction(0.5, 0.5)
beam_material.SetRestitution(0.0)
beam_material.SetYoungModulus(2e11)
beam_material.SetDensity(7850)


node1 = chrono.ChNodeBodyEasyPos(system)
node1.SetName("Node1")
node1.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(node1)

node2 = chrono.ChNodeBodyEasyPos(system)
node2.SetName("Node2")
node2.SetPos(chrono.ChVectorD(1, 0, 0))
system.Add(node2)


beam = chrono.ChBeamElement(beam_material, 0.1, 0.01)  
beam.SetNodes(node1, node2)
system.Add(beam)


vis.Init()
vis.Add(ground)
vis.Add(node1)
vis.Add(node2)
vis.Add(beam)
vis.Run()


while True:
    system.DoStepDynamics(0.01)
    vis.Render()