import pychrono as chrono
import pychrono.irrlicht as irr


physics = chrono.ChPhysicsEngine()
physics.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
my_system = chrono.ChSystemNSC(physics)
collision_system = chrono.ChCollisionSystem()



truss = chrono.ChBodyEasyBox(0.5, 1, 0.5, 1000, True, True, material=chrono.ChMaterialSurfaceNSC())
truss.SetPos(chrono.ChVectorD(0, 0.5, 0))
truss.SetBodyFixed(True)
my_system.Add(truss)


bar = chrono.ChBodyEasyBox(0.1, 1, 0.1, 10, True, True, material=chrono.ChMaterialSurfaceNSC())
bar.SetPos(chrono.ChVectorD(0, 1, 0))
bar.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
bar.SetBodyFixed(False)
my_system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(0.1, 0.5, 10, True, True, material=chrono.ChMaterialSurfaceNSC())
gear1.SetPos(chrono.ChVectorD(-0.3, 1, 0))
gear1.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
gear1.SetBodyFixed(False)
my_system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.1, 0.5, 10, True, True, material=chrono.ChMaterialSurfaceNSC())
gear2.SetPos(chrono.ChVectorD(0.3, 1, 0))
gear2.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
gear2.SetBodyFixed(False)
my_system.Add(gear2)


motor = chrono.ChLinkMotor AngularMotor(gear1, gear2, chrono.ChVectorD(0, 1, 0), 1000, True)
my_system.Add(motor)


application = irr.ChIrrApp(my_system, physics, collision_system, irr.CH_HAS_GUI)
application.AddBody(irr.ChIrrNodeBody(application.GetSystem(), truss))
application.AddBody(irr.ChIrrNodeBody(application.GetSystem(), bar))
application.AddBody(irr.ChIrrNodeBody(application.GetSystem(), gear1))
application.AddBody(irr.ChIrrNodeBody(application.GetSystem(), gear2))


light = irr.ChIrrLightApplication(application.GetVideoDriver(), application.GetSceneManager())
light.AddDirectionalLight(irr.SColorf(1, 1, 1, 1), chrono.ChVectorD(1, 1, -1))
application.SetLightSettings(light)

material = irr.ChIrrMaterial()
material.SetTexture(irr.ChStr("metal.jpg"))
application.GetSystem().SetMaterialAsset(material)


application.SetTimestep(0.01)
application.Run()