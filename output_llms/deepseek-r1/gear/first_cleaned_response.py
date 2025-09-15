import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  





truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)





sun_radius = 0.5
planet_radius = 0.3
bar_length = 1.5
gear_height = 0.2


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(bar)


revolute_bar = chrono.ChLinkRevolute()
revolute_bar.Initialize(truss, bar, 
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                         chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.Add(revolute_bar)


sun_gear = chrono.ChBody()
sun_gear.SetBodyFixed(True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_shape = chrono.ChCylinderShape(sun_radius, gear_height)
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.0))
sun_gear.AddAsset(sun_shape)
system.Add(sun_gear)


planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(bar_length, 0, 0))
planet_shape = chrono.ChCylinderShape(planet_radius, gear_height)
planet_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.0))
planet_gear.AddAsset(planet_shape)
system.Add(planet_gear)


revolute_planet = chrono.ChLinkRevolute()
revolute_planet.Initialize(bar, planet_gear,
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0),
                                           chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.Add(revolute_planet)





gear_ratio = -(sun_radius/planet_radius)  
gear_constraint = chrono.ChLinkLockGear()
gear_constraint.Initialize(sun_gear, planet_gear,
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)),
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0)))
gear_constraint.SetRatio(gear_ratio)
system.Add(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, bar, 
                chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 3))  
system.Add(motor)



vis = irr.ChIrrApp(system, "Epicyclic Gear System", irr.dimension2du(1024, 768))
vis.AddTypicalLights(irr.vector3df(30, 100, 30), irr.vector3df(-30, -100, -30), 500, 500)
vis.AddCamera(chrono.ChVectorD(3, 1.5, 3), chrono.ChVectorD(0, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()


vis.Run()

while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStepDynamics(1e-3)
    vis.EndScene()