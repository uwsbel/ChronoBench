import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
system.Add(ground)


sphere = chrono.ChBodyEasySphere(1, 1000, True, True)
sphere.SetPos(chrono.ChVectorD(0, 10, 0))
system.Add(sphere)


application = chronoirr.ChIrrApp(system, 'Falling Sphere', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 15, -20))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()