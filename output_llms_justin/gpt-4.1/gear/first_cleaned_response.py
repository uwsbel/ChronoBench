import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()


truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)


bar_length = 0.6
bar = chrono.ChBodyEasyBox(bar_length, 0.05, 0.05, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(bar)


sun_radius = 0.1
sun_gear = chrono.ChBodyEasyCylinder(sun_radius, 0.05, 1000, True, True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_gear.SetBodyFixed(True)
system.Add(sun_gear)


planet_radius = 0.05
planet_gear = chrono.ChBodyEasyCylinder(planet_radius, 0.05, 1000, True, True)
planet_gear.SetPos(chrono.ChVectorD(bar_length/2, 0, 0))
system.Add(planet_gear)


rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(bar, truss, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
system.AddLink(rev_bar)


rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(planet_gear, bar, chrono.ChCoordsysD(chrono.ChVectorD(bar_length/2,0,0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
system.AddLink(rev_planet)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(planet_gear, sun_gear, False, 
    chrono.ChCoordsysD(chrono.ChVectorD(bar_length/2,0,0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))),
    chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
gear_constraint.Set_ratio(-sun_radius/planet_radius)
system.AddLink(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
speed_fun = chrono.ChFunction_Const(chrono.CH_C_PI/2)  
motor.SetSpeedFunction(speed_fun)
system.AddLink(motor)


application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024,768)
application.SetWindowTitle('Epicyclic Gear System')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0.5, 0.5, 0.5), chrono.ChVectorD(0,0,0))
application.AddTypicalLights()


mat_sun = chrono.ChVisualMaterial()
mat_sun.SetDiffuseColor(chrono.ChColor(0.8, 0.6, 0.1))
sun_gear.GetVisualShape(0).SetMaterial(0, mat_sun)

mat_planet = chrono.ChVisualMaterial()
mat_planet.SetDiffuseColor(chrono.ChColor(0.2, 0.6, 0.8))
planet_gear.GetVisualShape(0).SetMaterial(0, mat_planet)

mat_bar = chrono.ChVisualMaterial()
mat_bar.SetDiffuseColor(chrono.ChColor(0.6, 0.8, 0.2))
bar.GetVisualShape(0).SetMaterial(0, mat_bar)


while application.Run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(0.01)
    application.EndScene()