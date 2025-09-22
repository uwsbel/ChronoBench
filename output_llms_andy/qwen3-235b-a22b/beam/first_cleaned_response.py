import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


mesh = chrono.ChMesh()
system.Add(mesh)


material = chrono.ChMaterialBeamEulerSimple()
material.SetYoungModulus(2e5)  
material.SetShearModulus(8e4)   
material.SetArea(0.01)          
material.SetIyy(0.001)          
material.SetIzz(0.001)          
material.SetJ(0.0001)           


for i in range(10):
    pos = chrono.ChVectorD(i * 0.1, 0, 0)  
    rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    node = chrono.ChNodeFEAxyzrot(pos, rot)  
    if i == 0:
        node.SetFixed(True)  
    mesh.AddNode(node)


for i in range(9):
    element = chrono.ChElementBeamEulerSimple()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i + 1))
    element.SetMaterial(material)
    element.SetYDirection(chrono.ChVectorD(0, 1, 0))  
    mesh.AddElement(element)


system.Set_G(chrono.ChVectorD(0, -9.81, 0))


application = chronoirr.ChIrrApp(system, 'Beam Finite Element Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.5, -1), chronoirr.vector3df(0, 0, 0))


mesh_vis = chrono.ChVisualShapeFEA(mesh)
mesh_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mesh.AddVisualShapeFEA(mesh_vis)


application.AssetBindAll()
application.AssetUpdateAll()


step_size = 0.01  
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(step_size)