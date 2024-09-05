import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
system.Add(ground)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True)
box.SetPos(chrono.ChVectorD(0, 5, 0))
system.Add(box)


ball = chrono.ChBodyEasySphere(0.5, 1000, True)
ball.SetPos(chrono.ChVectorD(2, 5, 0))
system.Add(ball)


application = chronoirr.ChIrrApp(system, "PyChrono Example", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
application.AddTypicalCamera(chronoirr.vector3df(0, 14, -20))
application.AddTypicalLights()


application.AssetBindAll()
application.AssetUpdateAll()

time_step = 0.01
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep(time_step)
    application.EndScene()