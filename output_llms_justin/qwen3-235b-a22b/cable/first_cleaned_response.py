import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


L = 1.0  
N = 10   
A = 0.01**2  
E = 2e5      
rho = 7800   


mesh = fea.ChMesh()


nodes = []
for i in range(N + 1):
    pos = chrono.ChVectorD(i * L / N, 0, 0)
    node = fea.ChNodeFEAxyz(pos)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(N):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSectionProperties(A, E, rho)  
    mesh.AddElement(element)


constraint = fea.ChLinkPointFrame()
constraint.Initialize(nodes[0], chrono.ChFrameD())
sys.Add(constraint)


vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam.SetWireframe(True)
vis_beam.SetDrawInUndeformedState(False)  
mesh.AddVisualShapeFEA(vis_beam)


sys.Add(mesh)


vis = irr.ChIrrApp(sys, 'ANCF Beam Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))
vis.Run()


while vis.GetDevice().run():
    sys.DoStepDynamics(0.001)  
    vis.BeginScene()
    vis.Render()
    vis.EndScene()