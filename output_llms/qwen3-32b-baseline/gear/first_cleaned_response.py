import pychrono as chrono
from pychrono import chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


ground = chrono.ChBody()
system.Add(ground)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChVisualShapeBox(0.2, 0.2, 0.2)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


sun_gear = chrono.ChBody()
system.Add(sun_gear)
sun_gear.SetBodyFixed(True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_shape = chrono.ChVisualShapeCylinder(0.1, 0.2)  
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun_gear.AddVisualShape(sun_shape)


carrier = chrono.ChBody()
system.Add(carrier)
carrier.SetMass(1.0)
carrier.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
carrier.SetPos(chrono.ChVectorD(0.3, 0, 0))  


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, carrier, chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.Q_from_AngZ(0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_C_PI / 2))  
system.AddLink(motor)


planet_gear = chrono.ChBody()
system.Add(planet_gear)
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0))


rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(carrier, planet_gear, chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0, 0), chrono.Q_from_AngZ(0)))
system.AddLink(rev_planet)


planet_shape = chrono.ChVisualShapeCylinder(0.2, 0.2)  
planet_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
planet_gear.AddVisualShape(planet_shape)


gear_link = chrono.ChLinkGear()
gear_link.Initialize(sun_gear, planet_gear,
                     chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)),
                     chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0, 0), chrono.Q_from_AngZ(0)))
gear_link.SetGearRatio(-0.1 / 0.2)  
gear_link.SetMask(chrono.GEAR_SPUR)
system.AddLink(gear_link)


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chronoirr.vector3df(2, 2, 2))
application.SetLight(chronoirr.vector3df(0, 0, 0), True)
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.01)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(0.01)
    application.EndScene()