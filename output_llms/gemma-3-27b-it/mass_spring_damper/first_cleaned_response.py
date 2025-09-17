import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as chronoveh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


body = chrono.ChBodyEasy()
body.SetMass(1.0)
body.SetPos(chrono.ChVectorD(0, 1, 0))
body.SetBodyFixed(False)
system.Add(body)


sphere_shape = chrono.ChSphereShape()
sphere_shape.GetSphereGeometry().rad = 0.5
body.AddAsset(sphere_shape)
sphere_shape.SetMaterialFlags(chrono.ChMaterialProperties.MSH_VisualOnly)


ground = chrono.ChBodyEasy()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().x = 10
ground_shape.GetBoxGeometry().y = 0.1
ground_shape.GetBoxGeometry().z = 10
ground.AddAsset(ground_shape)
ground_shape.SetMaterialFlags(chrono.ChMaterialProperties.MSH_VisualOnly)


link = chrono.ChLinkTSDA()
link.Initialize(body, ground, False, False)
link.Set_SpringRestLength(1.0)  
link.Set_SpringK(400)  
link.Set_SpringR(10)  
link.Set_LinkDataOffset(chrono.ChVectorD(0, 0, 0))
link.Set_LinkDataRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(1,0,0)))
system.Add(link)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddSkyBox()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)