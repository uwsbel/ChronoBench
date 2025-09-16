import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()


node1 = fea.ChNodeFEAxyz(0, 0, 0)
node2 = fea.ChNodeFEAxyz(1, 0, 0)
node3 = fea.ChNodeFEAxyz(2, 0, 0)
mesh.AddNode(node1)
mesh.AddNode(node2)
mesh.AddNode(node3)


beam1 = fea.ChElementBeamEuler()
beam1.SetNodes(node1, node2)
beam1.SetSection(fea.ChSectionEulerSimple(0.1, 0.1, 1000, 1000))
mesh.AddElement(beam1)

beam2 = fea.ChElementBeamEuler()
beam2.SetNodes(node2, node3)
beam2.SetSection(fea.ChSectionEulerSimple(0.1, 0.1, 1000, 1000))
mesh.AddElement(beam2)


sys.Add(mesh)


sys.Set_G(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


vis.AddCamera(chrono.ChCamera())
vis.AddTypicalLights()
vis.SetWindowSize(1024, 768)
vis.SetCamera(chrono.ChCamera().GetCamera())
vis.SetCameraDistance(5)
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddSystem(sys)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()