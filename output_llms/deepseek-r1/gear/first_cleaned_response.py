import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


gear_ratio = -2.0  
bar_length = 0.3    
sun_radius = 0.2    
planet_radius = 0.1 
motor_speed = 0.5   


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetCollide(False)


truss_box = chrono.ChBoxShape()
truss_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.AddAsset(truss_box)
truss.AddAsset(chrono.ChColorAsset(0.6, 0.6, 0.6))
system.Add(truss)


sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_gear.SetMass(1)
sun_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
sun_shape.GetCylinderGeometry().rad = sun_radius
sun_gear.AddAsset(sun_shape)
sun_gear.AddAsset(chrono.ChColorAsset(0.8, 0.1, 0.1))
system.Add(sun_gear)


revolute_sun = chrono.ChLinkLockRevolute()
revolute_sun.Initialize(truss, sun_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.Add(revolute_sun)
revolute_sun.SetMotionType(chrono.ChLinkLockRevolute.MOTION_LOCKED)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().Size = chrono.ChVectorD(bar_length/2, 0.02, 0.02)
bar.AddAsset(bar_shape)
bar.AddAsset(chrono.ChColorAsset(0.1, 0.8, 0.1))
system.Add(bar)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetAngleFunction(chrono.ChFunction_Ramp(0, motor_speed))
system.Add(motor)


planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(bar_length, 0, 0))
planet_gear.SetMass(1)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
planet_shape.GetCylinderGeometry().rad = planet_radius
planet_gear.AddAsset(planet_shape)
planet_gear.AddAsset(chrono.ChColorAsset(0.1, 0.1, 0.8))
system.Add(planet_gear)


revolute_planet = chrono.ChLinkLockRevolute()
revolute_planet.Initialize(bar, planet_gear, 
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0)))
system.Add(revolute_planet)


gear_link = chrono.ChLinkGear()
gear_link.Initialize(revolute_sun, revolute_planet)
gear_link.SetTransmissionRatio(gear_ratio)
system.Add(gear_link)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 0.75))
vis.AddTypicalLights()


timestep = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)