import pychrono as chrono
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath('.')     
system = chrono.ChSystemNSC()




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))

cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-1, 0, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD( 1, 0, 0)
cyl.GetCylinderGeometry().rad = 0.05
ground.AddAsset(cyl)

ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))
system.Add(ground)




bar = chrono.ChBody()
bar.SetMass(2.0)

bar.SetInertiaXX(chrono.ChVectorD(0.02, 0.02, 0.02))
bar.SetPos(chrono.ChVectorD(0, 0, 0))

bar_cyl = chrono.ChCylinderShape()
bar_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.5, 0, 0)
bar_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD( 0.5, 0, 0)
bar_cyl.GetCylinderGeometry().rad = 0.03
bar.AddAsset(bar_cyl)
bar.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.4, 0.8)))
system.Add(bar)


rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(bar, ground,
                   chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                                   chrono.Q_from_AngAxis(0, 0, 1, 0)))
system.AddLink(rev_bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, ground,
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                                 chrono.Q_from_AngAxis(0, 0, 1, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
system.AddLink(motor)




sun_rad = 0.2
sun = chrono.ChBody()
sun.SetMass(1.0)
sun.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))

sun.SetPos(chrono.ChVectorD(0.6, 0, 0))

sun_cyl = chrono.ChCylinderShape()
sun_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0.6, 0, -0.025)
sun_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0.6, 0, +0.025)
sun_cyl.GetCylinderGeometry().rad = sun_rad
sun.AddAsset(sun_cyl)
sun.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.2, 0.2)))
system.Add(sun)


lock_sun = chrono.ChLinkMateGeneric()
lock_sun.Initialize(sun, bar, chrono.ChFrameD())

lock_sun.SetConstrainedCoords(True, True, True,  True, True, True)
system.AddLink(lock_sun)




planet_rad = 0.1
planet = chrono.ChBody()
planet.SetMass(0.5)
planet.SetInertiaXX(chrono.ChVectorD(0.005, 0.005, 0.005))

planet.SetPos(chrono.ChVectorD(1.0, 0, 0))

planet_cyl = chrono.ChCylinderShape()
planet_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(1.0, 0, -0.025)
planet_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(1.0, 0, +0.025)
planet_cyl.GetCylinderGeometry().rad = planet_rad
planet.AddAsset(planet_cyl)
planet.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.8, 0.2)))
system.Add(planet)


rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(planet, ground,
                      chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0),
                                      chrono.Q_from_AngAxis(0, 0, 1, 0)))
system.AddLink(rev_planet)


gear = chrono.ChLinkGear()
gear.Initialize(sun, planet,
                chrono.ChFrameD(chrono.ChVectorD(0,0,0),
                                chrono.Q_from_AngAxis(0,0,1,0)))
gear.SetGearRatio(-sun_rad/planet_rad)
system.AddLink(gear)




app = chronoirr.ChIrrApp(system, "Epicyclic Gear Demo", chronoirr.dimension2du(1024,768))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddCamera(chronoirr.vector3df(1.5,1.0,1.5), chronoirr.vector3df(0.6,0,0))


app.AssetBindAll()
app.AssetUpdateAll()




timestep = 1e-3
while app.Run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(timestep)
    app.EndScene()