import pychrono as chrono
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()


mesh = chrono.ChMesh()


node1 = mesh.AddNode(chrono.ChVectorD(0, 0, 0))
node2 = mesh.AddNode(chrono.ChVectorD(1, 0, 0))
node3 = mesh.AddNode(chrono.ChVectorD(0, 1, 0))


beam1 = mesh.AddBeamABC(node1, node2)
beam2 = mesh.AddBeamABC(node2, node3)
beam3 = mesh.AddBeamABC(node3, node1)


beam1.SetBeamRaleyghDamping(0.01)
beam2.SetBeamRaleyghDamping(0.01)
beam3.SetBeamRaleyghDamping(0.01)

beam1.SetBeamSectionEIyy(1e6)
beam2.SetBeamSectionEIyy(1e6)
beam3.SetBeamSectionEIyy(1e6)

beam1.SetBeamSectionGJ(1e5)
beam2.SetBeamSectionGJ(1e5)
beam3.SetBeamSectionGJ(1e5)

beam1.SetBeamSectionArea(0.1)
beam2.SetBeamSectionArea(0.1)
beam3.SetBeamSectionArea(0.1)


beam_element = chrono.ChElementBeamEuler()
beam_element.SetMesh(mesh)


my_system.Add(beam_element)


vis = chronoirr.ChIrrApp(my_system, 'Beam Finite Elements', chronoirr.dimension2du(1024, 768))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -3))
vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStep()
    vis.EndScene()