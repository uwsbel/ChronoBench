import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("../../../data/")
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


mass = chrono.ChBodyEasySphere(0.1, 1000)  
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))  
my_system.Add(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0.5, 0))
spring_damper.SetSpringCoefficient(50)  
spring_damper.SetDampingCoefficient(1)  
my_system.Add(spring_damper)


col_g = chrono.ChColorAsset()
col_g.SetColor(chrono.ChColor(0.3, 0.3, 0.3))  
ground.AddAsset(col_g)

col_m = chrono.ChColorAsset()
col_m.SetColor(chrono.ChColor(0.6, 0.2, 0.2))  
mass.AddAsset(col_m)


spring_asset = chrono.ChPointPointSpring(0.01, 80, 15)
spring_asset.SetColor(chrono.ChColor(0, 0.8, 0.8))
spring_damper.AddAsset(spring_asset)


application = chronoirr.ChIrrApp(my_system, "Mass-Spring-Damper Demo", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddTypicalCamera(chronoirr.vector3df(1, 0.5, -1))
application.AddTypicalLights()


application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.005)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()