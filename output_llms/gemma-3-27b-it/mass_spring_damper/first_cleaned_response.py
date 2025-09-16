import chrono
import chrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


body = chrono.ChBodyEasy()
body.SetMass(1.0)
body.SetPos(chrono.ChVectorD(0, 1, 0))
body.SetBodyFixed(False)
system.Add(body)


sphere = chrono.ChSphereShape()
sphere.GetSphereGeometry().rad = 0.5
body.AddAsset(sphere)
sphere.SetMaterial(chrono.ChMaterialSurfaceNSC())
sphere.SetMaterialSurface(chrono.ChMaterialSurfaceNSC().SetRoughness(0.1))


ground = chrono.ChBodyEasy()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().x = 10
ground_shape.GetBoxGeometry().y = 0.1
ground_shape.GetBoxGeometry().z = 10
ground.AddAsset(ground_shape)
ground_shape.SetMaterial(chrono.ChMaterialSurfaceNSC())
ground_shape.SetMaterialSurface(chrono.ChMaterialSurfaceNSC().SetRoughness(0.1))



link = chrono.ChLinkTSDA()
link.Initialize(body, ground, False, False)
link.Set_SpringRestLength(1.0)  
link.Set_SpringK(100)  
link.Set_SpringR(0.5)  
system.Add(link)


cylinder = chrono.ChCylinderShape()
cylinder.GetCylinderGeometry().rad = 0.05
cylinder.GetCylinderGeometry().height = 1.0
link.AddAsset(cylinder)
cylinder.SetMaterial(chrono.ChMaterialSurfaceNSC())
cylinder.SetMaterialSurface(chrono.ChMaterialSurfaceNSC().SetRoughness(0.1))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(0, 2, -3))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)