import chrono
import chrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






body = chrono.ChBodyEasy()
body.SetBodyFixed(False)
body.SetMass(1.0)
body.SetPos(chrono.ChVectorD(0, 1, 0))
body.SetCollide(True)
system.Add(body)


sphere_shape = chrono.ChSphereShape()
sphere_shape.GetSphereGeometry().rad = 0.5
body.AddAsset(sphere_shape)
sphere_shape.SetMaterial(chrono.ChMaterialSurfaceNSC())


link = chrono.ChLinkTSDA()
link.Initialize(body,
                None,  
                False, 
                chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
link.Set_SpringRestLength(1.0)
link.Set_SpringK(100.0)
link.Set_SpringR(0.5) 
system.Add(link)


cylinder_shape = chrono.ChCylinderShape()
cylinder_shape.GetCylinderGeometry().rad = 0.05
cylinder_shape.GetCylinderGeometry().height = 1.0
link.AddAsset(cylinder_shape)
cylinder_shape.SetMaterial(chrono.ChMaterialSurfaceNSC())





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -3))
vis.AddTypicalLights()





time_step = 0.01
time_end = 10.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)