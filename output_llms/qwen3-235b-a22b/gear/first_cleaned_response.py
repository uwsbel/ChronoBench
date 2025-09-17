import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


truss = chrono.ChBody()
system.Add(truss)
truss.SetBodyFixed(True)


truss_shape = chrono.ChCylinderShape()
truss_shape.GetCylinderGeometry().rad = 0.05
truss_shape.GetCylinderGeometry().p1 = chrono.ChVector(0, 0, 0)
truss_shape.GetCylinderGeometry().p2 = chrono.ChVector(0, 0, 0.1)
truss_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  
truss.AddAsset(truss_shape)


sun = chrono.ChBody()
sun.SetMass(1)
sun.SetInertiaXX(chrono.ChVector(0.1, 0.1, 0.1))
sun.SetPos(chrono.ChVector(0, 0, 0))
system.Add(sun)


sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().rad = 0.2
sun_shape.GetCylinderGeometry().p1 = chrono.ChVector(0, 0, -0.1)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVector(0, 0, 0.1)
sun_shape.SetColor(chrono.ChColor(1, 0.8, 0))  
sun.AddAsset(sun_shape)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun, truss, chrono.ChFrame(chrono.ChVector(0, 0, 0), chrono.QUNIT))
speed_func = chrono.ChFunction_Const(10)  
motor.SetSpeedFunction(speed_func)
system.Add(motor)


carrier = chrono.ChBody()
carrier.SetMass(2)
carrier.SetInertiaXX(chrono.ChVector(1, 1, 1))
carrier.SetPos(chrono.ChVector(0, 0, 0))
system.Add(carrier)


carrier_shape = chrono.ChBoxShape()
carrier_shape.GetBoxGeometry().Size = chrono.ChVector(0.4, 0.05, 0.05)
carrier_shape.SetColor(chrono.ChColor(0, 0.8, 0.8))  
carrier.AddAsset(carrier_shape)


carrier_revolute = chrono.ChLinkRevolute()
carrier_revolute.Initialize(truss, carrier, chrono.ChCoordsys(chrono.ChVector(0, 0, 0), chrono.QUNIT))
system.Add(carrier_revolute)


planet = chrono.ChBody()
planet.SetMass(1)
planet.SetInertiaXX(chrono.ChVector(0.1, 0.1, 0.1))
planet.SetPos(chrono.ChVector(0.4, 0, 0))  
system.Add(planet)


planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().rad = 0.2
planet_shape.GetCylinderGeometry().p1 = chrono.ChVector(0, 0, -0.1)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVector(0, 0, 0.1)
planet_shape.SetColor(chrono.ChColor(0.8, 0, 0.8))  
planet.AddAsset(planet_shape)


planet_revolute = chrono.ChLinkRevolute()
planet_revolute.Initialize(carrier, planet, chrono.ChCoordsys(chrono.ChVector(0.4, 0, 0), chrono.QUNIT))
system.Add(planet_revolute)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun, planet, False, chrono.ChFrame(), chrono.ChFrame())
gear_constraint.SetTransmissionRatio(-1.0)  
system.Add(gear_constraint)


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1), chronoirr.vector3df(0, 0, 0))


application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()