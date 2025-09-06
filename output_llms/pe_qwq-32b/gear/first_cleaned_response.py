import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 1024)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.SetSymbolScale(0.01)
vis.SetShadingsType(chronoirr.ChrChronoIrrlicht.SHADINGS_GLASS)
vis.SetLightIntensity(0.8)
vis.AddLight(chrono.ChVectorD(3, 3, 5), chrono.ChVectorD(255, 255, 255), 200)
vis.Initialize()
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0)))


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss_shape = chrono.ChBox(0.5, 0.1, 0.5)  
truss.AddVisualShape(chrono.ChVisualShape(truss_shape, chrono.ChColor(0.5, 0.5, 0.5)))
system.Add(truss)


sun_radius = 0.2
sun_mass = 1.0
sun_gear = chrono.ChBody()
sun_gear.SetMass(sun_mass)
sun_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_shape = chrono.ChCylinder(sun_radius, 0.1)  
sun_gear.AddVisualShape(chrono.ChVisualShape(sun_shape, chrono.ChColor(0.8, 0.8, 0.8)))
system.Add(sun_gear)


sun_joint = chrono.ChLinkLockRevolute()
sun_joint.Initialize(truss, sun_gear, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sun_joint.SetAxis(chrono.ChVectorD(0, 0, 1))
system.Add(sun_joint)


motor_sun = chrono.ChLinkMotorRotationSpeed()
motor_sun.Initialize(truss, sun_gear, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
motor_sun.SetSpeedFunction(chrono.ChFunction_Const(2.0))  
system.Add(motor_sun)


bar_length = sun_radius + 0.1  
bar_radius = 0.02
bar_mass = 0.5
bar = chrono.ChBody()
bar.SetMass(bar_mass)
bar.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar_shape = chrono.ChBox(bar_length, bar_radius, bar_radius)  
bar.AddVisualShape(chrono.ChVisualShape(bar_shape, chrono.ChColor(0.2, 0.2, 0.8)))
system.Add(bar)


bar_joint = chrono.ChLinkLockRevolute()
bar_joint.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
bar_joint.SetAxis(chrono.ChVectorD(0, 0, 1))
system.Add(bar_joint)


planet_radius = 0.1
planet_mass = 0.5
planet_gear = chrono.ChBody()
planet_gear.SetMass(planet_mass)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
planet_gear.SetPos(chrono.ChVectorD(bar_length, 0, 0))  
planet_shape = chrono.ChCylinder(planet_radius, 0.1)
planet_gear.AddVisualShape(chrono.ChVisualShape(planet_shape, chrono.ChColor(0.8, 0.2, 0.2)))
system.Add(planet_gear)


planet_joint = chrono.ChLinkLockRevolute()
planet_joint.Initialize(bar, planet_gear, chrono.ChFrameD(chrono.ChVectorD(bar_length,0,0), chrono.QUNIT))
planet_joint.SetAxis(chrono.ChVectorD(0, 0, 1))
system.Add(planet_joint)


gear_ratio = - (sun_radius / planet_radius)  
gear_link = chrono.ChLinkGear()
gear_link.Initialize(sun_gear, planet_gear, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
gear_link.SetGearRatio(gear_ratio)
system.Add(gear_link)


sun_gear.SetCollide(False)
planet_gear.SetCollide(False)


while vis.Run():
    system.DoStepDynamics(0.001)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()