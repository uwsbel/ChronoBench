import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetName("Truss")
system.Add(truss)


truss_box = chrono.ChBoxShape()
truss_box.GetBoxGeometry().Size = chrono.ChVectorD(1, 1, 0.1)
truss.AddAsset(truss_box)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(bar)


bar_box = chrono.ChBoxShape()
bar_box.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 0.1)
bar.AddAsset(bar_box)


joint_carrier = chrono.ChLinkRevolute()
joint_carrier.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(joint_carrier)


sun_gear = chrono.ChBody()
sun_gear.SetMass(0.5)
sun_gear.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(sun_gear)


sun_cylinder = chrono.ChCylinderShape()
sun_cylinder.GetCylinderGeometry().rad = 0.2
sun_cylinder.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.05)
sun_cylinder.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.05)
sun_gear.AddAsset(sun_cylinder)


motor_sun = chrono.ChLinkMotorRotationSpeed()
motor_sun.Initialize(sun_gear, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor_sun.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
system.AddLink(motor_sun)


planet_gear = chrono.ChBody()
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
planet_gear.SetPos(chrono.ChVectorD(1, 0, 0))  
system.Add(planet_gear)


planet_cylinder = chrono.ChCylinderShape()
planet_cylinder.GetCylinderGeometry().rad = 0.2
planet_cylinder.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.05)
planet_cylinder.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.05)
planet_gear.AddAsset(planet_cylinder)


joint_planet = chrono.ChLinkRevolute()
joint_planet.Initialize(bar, planet_gear, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0), chrono.QUNIT))
system.AddLink(joint_planet)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, chrono.ChFrameD(), chrono.ChFrameD())
gear_constraint.SetTransmissionRatio(-1.0)  
system.AddLink(gear_constraint)


vis = irr.ChIrrApp(system, 'Epicyclic Gears Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(irr.vector3df(0, 3, -6))
vis.AddLightWithShadow(irr.vector3df(2, 5, 2), irr.vector3df(0, 0, 0), 10, 1, 10, 40, 512, irr.SColorf(1, 1, 1))


truss_color = chrono.ChColorAsset()
truss_color.GetColor() = chrono.ChColor(0.8, 0.8, 0.8)  
truss.AddAsset(truss_color)

bar_color = chrono.ChColorAsset()
bar_color.GetColor() = chrono.ChColor(0.6, 0.4, 0.2)  
bar.AddAsset(bar_color)

sun_color = chrono.ChColorAsset()
sun_color.GetColor() = chrono.ChColor(1, 0.5, 0)  
sun_gear.AddAsset(sun_color)

planet_color = chrono.ChColorAsset()
planet_color.GetColor() = chrono.ChColor(0, 0.5, 1)  
planet_gear.AddAsset(planet_color)


vis.AssetBindAll()
vis.AssetUpdateAll()


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
system.SetStep(0.01)

while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)