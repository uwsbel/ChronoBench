import pychrono

import pychrono.irrlicht

from pychrono.IrrIterator import IrrIterator




chrono = pychrono.ChLink_Denge()

chrono.Init()




system = pychrono.ChSystemNSC()




visualization = pychrono.irrlicht.ChIrrApp("Beam Finite Elements Simulation", 1280, 720)

visualization.AddTypicalLogo()

visualization.AddTypicalConfiguration()

visualization.AddPlane(pychrono.irrlicht.EbBox(0, 0, 0, 0, 100, 100, 100, 100), 1)

visualization.AddLight(pychrono.irrlicht.EbBox(0, 0, 0, 0, 100, 100, 100, 100), 1)




beam = pychrono.ChBeamNSC()

node1 = pychrono.ChBodyNSC()

node2 = pychrono.ChBodyNSC()




beam.SetMaterial(pychrono.ChMaterialNSC())

beam.SetWireframe(True)

beam.SetColor(pychrono.irrlicht.EColor(255, 0, 0, 255))




node1.SetMaterial(pychrono.ChMaterialNSC())

node1.SetWireframe(True)

node1.SetColor(pychrono.irrlicht.EColor(255, 0, 255, 255))


node2.SetMaterial(pychrono.ChMaterialNSC())

node2.SetWireframe(True)

node2.SetColor(pychrono.irrlicht.EColor(0, 255, 0, 255))




beam.AddBeam(node1.GetPos(), node1.GetPos() + pychrono.ChVectorD(10, 0, 0), 1, 1, 1, 1)

beam.AddBeam(node1.GetPos() + pychrono.ChVectorD(10, 0, 0), node2.GetPos(), 1, 1, 1, 1)




system.Add(beam)

system.Add(node1)

system.Add(node2)




system.SetGravity(pychrono.ChVectorD(0, -9.81, 0))




beam.SetNode1(node1)

beam.SetNode2(node2)




system.AddConstraint(pychrono.ChLinkNSC())




node1.SetPos(pychrono.ChVectorD(0, 0, 0))

node2.SetPos(pychrono.ChVectorD(10, 0, 0))




visualization.AddIterator(IrrIterator(beam, visualization))

visualization.AddIterator(IrrIterator(node1, visualization))

visualization.AddIterator(IrrIterator(node2, visualization))




while(visualization.GetDevice().run()):

    system.DoStepDynamics(0.01)

    visualization.BeginScene()

    visualization.DoCollision(system)





beam.SetName("Beam")

beam.SetPos(pychrono.ChBody(node1)

node1.SetName("Node1")

node2.SetName("Node2")





node1.SetPos(pychrono.ChBody(node1)

node2.SetPos(node2)





node1.SetPos(node1)

node2.SetPos(node2)





node1.SetPos(node1)

node2.SetPos(node2)





while(visualization.GetDevice().SetPos(node1)

node2.SetPos(node2)





visualization.DoFrame()