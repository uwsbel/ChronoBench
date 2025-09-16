import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = fea.ChMesh()


L = 1.0  
num_elements = 10  
radius = 0.01  
E = 2e5  
density = 1000  
area = chrono.CH_C_PI * radius**2  
I = chrono.CH_C_PI * radius**4 / 4  


for i in range(num_elements + 1):
    pos = chrono.ChVectorD(L * i / num_elements, 0, 0)
    node = fea.ChNodeFEAxyz(pos)
    mesh.AddNode(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i+1))
    element.SetDiameter(2 * radius)
    element.SetMaterialYoungModulus(E)
    element.SetArea(area)
    element.SetI(I)
    element.SetDensity(density)
    mesh.AddElement(element)


system.Add(mesh)


node0 = mesh.GetNode(0)
constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
constraint.Initialize(node0, system.GetGroundBody(), False, chrono.ChFrameD(), chrono.ChFrameD())
system.Add(constraint)


application = irrlicht.ChIrrApp(system, 'ANCF Beam Simulation', irrlicht.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irrlicht.vector3df(0.5, 0.5, -1.5))  


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetShowNodes(True)
vis_mesh.SetNodeSize(0.01)
mesh.AddAsset(vis_mesh)

application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1e-4)