import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)


sun_gear = chrono.ChBody()
sun_gear.SetMass(1)
sun_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_gear.GetCollisionModel().ClearModel()
sun_gear.GetCollisionModel().AddCylinder(0.2, 0.2, 0.1)  
sun_gear.SetCollide(False)
system.Add(sun_gear)


revolute_sun = chrono.ChLinkRevolute()
revolute_sun.Initialize(truss, sun_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(revolute_sun)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun_gear, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(2.0))  
system.AddLink(motor)


bar = chrono.ChBody()
bar.SetMass(2)
bar.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.GetCollisionModel().ClearModel()
bar.GetCollisionModel().AddBox(1, 0.1, 0.1)  
bar.SetCollide(False)
system.Add(bar)


revolute_bar = chrono.ChLinkRevolute()
revolute_bar.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(revolute_bar)


planet_gear = chrono.ChBody()
planet_gear.SetMass(1)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
planet_gear.GetCollisionModel().ClearModel()
planet_gear.GetCollisionModel().AddCylinder(0.2, 0.2, 0.1)
planet_gear.SetCollide(False)
system.Add(planet_gear)


revolute_planet = chrono.ChLinkRevolute()
revolute_planet.Initialize(bar, planet_gear, chrono.ChCoordsysD(chrono.ChVectorD(0.4, 0, 0), chrono.QUNIT))
system.AddLink(revolute_planet)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, True, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, 1))
gear_constraint.SetRatio(-1.0)  
system.AddLink(gear_constraint)


application = chronoirr.ChIrrApp(system, "Epicyclic Gears Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 3, 3), chronoirr.vector3df(0, 0, 0))
application.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(0, 0, 0), 10, 0.2, 10, 512, chrono.CH_C_PI / 6)


truss.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.8, 0.8)))  
sun_gear.AddAsset(chrono.ChColorAsset(chrono.ChColor(1, 0, 0)))      
bar.AddAsset(chrono.ChColorAsset(chrono.ChColor(0, 1, 0)))          
planet_gear.AddAsset(chrono.ChColorAsset(chrono.ChColor(0, 0, 1)))  

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())