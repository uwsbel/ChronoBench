import chrono
import chrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


body = chrono.ChBodyEasy()
body.SetMass(1.0)
body.SetPos(chrono.ChVectorD(0, 1, 0))
body.SetBodyFixed(False)
system.Add(body)


sphere_asset = chrono.ChSphereShape()
sphere_asset.GetSphereGeometry().rad = 0.5
body.AddAsset(sphere_asset)
sphere_asset.SetMaterialFlags(chrono.ChMaterialProperties.MSH_VISUAL)


ground_body = chrono.ChBodyEasy()
ground_body.SetBodyFixed(True)
ground_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground_body)

link = chrono.ChLinkTSDA()
link.Initialize(body, ground_body, False, False)
link.Set_SpringRestLength(1.0)  
link.Set_SpringK(400)  
link.Set_DampingC(10)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(0, 2, -3))
vis.AddTypicalLights()


spring_asset = chrono.ChLineShape()
spring_asset.Set_Points(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
link.AddAsset(spring_asset)
spring_asset.SetMaterialFlags(chrono.ChMaterialProperties.MSH_VISUAL)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)