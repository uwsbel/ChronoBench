import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.6)
material.SetRestitution(0.2)


truss = chrono.ChBodyEasyBox(1, 1, 0.1, 1000, True, True, material)
truss.SetPos(chrono.ChVectorD(0, 0, 0.5))
truss.SetBodyFixed(True)
system.Add(truss)


bar = chrono.ChBodyEasyBox(0.1, 0.1, 2, 1000, True, True, material)
bar.SetPos(chrono.ChVectorD(0, 0, 1))
system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True, material)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 1))
system.Add(gear1)


gear2 = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True, material)
gear2.SetPos(chrono.ChVectorD(-0.5, 0, 1))
system.Add(gear2)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.ChVectorD(0, 0, 10)))  
system.Add(motor)


application = chronoirr.ChIrrApp(system, 'Epicyclic Gear System', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
application.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 10, 40, 50)


application.AssetBind(truss, chronoirr.ChIrrNodePointCloud(chrono.ChVectorD(0, 0, 0)))
application.AssetBind(bar, chronoirr.ChIrrNodePointCloud(chrono.ChVectorD(0, 0, 0)))
application.AssetBind(gear1, chronoirr.ChIrrNodePointCloud(chrono.ChVectorD(0, 0, 0)))
application.AssetBind(gear2, chronoirr.ChIrrNodePointCloud(chrono.ChVectorD(0, 0, 0)))


application.SetTimestep(0.01)
application.SetTryRealTime(True)
application.SetSymbolBcgColor(chrono.ChColor(0.8, 0.8, 0.8))

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

application.Close()