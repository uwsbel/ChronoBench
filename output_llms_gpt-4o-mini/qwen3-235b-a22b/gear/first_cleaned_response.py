import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss)


truss_box = chrono.ChBoxShape()
truss_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.GetVisualShape().AddShape(truss_box)


sun = chrono.ChBody()
sun.SetMass(1.0)
sun.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
sun.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(sun)


sun_cyl = chrono.ChCylinderShape()
sun_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
sun_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
sun_cyl.GetCylinderGeometry().rad = 0.666  
sun.GetVisualShape().AddShape(sun_cyl)


mat_sun = chrono.ChMaterialSurfaceNSC()
mat_sun.SetDiffuseColor(chrono.ChColor(1, 0, 0))
sun.GetVisualShape().SetMaterial(0, mat_sun)


sun_revolute = chrono.ChLinkRevolute()
sun_revolute.Initialize(truss, sun, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(sun_revolute)


omega_sun = 10.0  
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(omega_sun))
system.Add(motor)


carrier = chrono.ChBody()
carrier.SetMass(2.0)
carrier.SetInertiaXX(chrono.ChVectorD(0.5, 0.5, 0.5))
carrier.SetPos(chrono.ChVectorD(0.5, 0, 0))  
system.Add(carrier)


carrier_box = chrono.ChBoxShape()
carrier_box.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.05, 0.05)
carrier.GetVisualShape().AddShape(carrier_box)


mat_carrier = chrono.ChMaterialSurfaceNSC()
mat_carrier.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
carrier.GetVisualShape().SetMaterial(0, mat_carrier)


carrier_revolute = chrono.ChLinkRevolute()
carrier_revolute.Initialize(truss, carrier, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(carrier_revolute)


planet = chrono.ChBody()
planet.SetMass(0.5)
planet.SetInertiaXX(chrono.ChVectorD(0.02, 0.02, 0.02))
planet.SetPos(chrono.ChVectorD(1, 0, 0))  
system.Add(planet)


planet_cyl = chrono.ChCylinderShape()
planet_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
planet_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
planet_cyl.GetCylinderGeometry().rad = 0.333  
planet.GetVisualShape().AddShape(planet_cyl)


mat_planet = chrono.ChMaterialSurfaceNSC()
mat_planet.SetDiffuseColor(chrono.ChColor(0, 0, 1))
planet.GetVisualShape().SetMaterial(0, mat_planet)


planet_revolute = chrono.ChLinkRevolute()
planet_revolute.Initialize(carrier, planet, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0), chrono.QUNIT))
system.Add(planet_revolute)


gear_ratio = -2.0  
sun_planet_gear = chrono.ChLinkGear()
sun_planet_gear.Initialize(sun, planet, True, chrono.ChFrameD(), chrono.ChFrameD())
sun_planet_gear.SetRatio(gear_ratio)
system.Add(sun_planet_gear)


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))


application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1e-3)