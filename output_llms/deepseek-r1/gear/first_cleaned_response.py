import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


r_sun = 0.5      
r_planet = 0.2   
d = r_sun + r_planet  
gear_thickness = 0.2  
bar_width = 0.1       


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")
system.Add(ground)


carrier = chrono.ChBody()
carrier.SetPos(chrono.ChVectorD(d/2, 0, 0))
carrier.SetMass(5)
carrier.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
carrier.SetName("Carrier")
system.Add(carrier)


bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().Size = chrono.ChVectorD(d/2, bar_width/2, bar_width/2)
bar_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
carrier.AddVisualShape(bar_shape)


sun = chrono.ChBody()
sun.SetPos(chrono.ChVectorD(0, 0, 0))
sun.SetBodyFixed(True)
sun.SetMass(3)
sun.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
sun.SetName("Sun")
system.Add(sun)


sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().rad = r_sun
sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun.AddVisualShape(sun_shape)


planet = chrono.ChBody()
planet.SetPos(chrono.ChVectorD(d, 0, 0))
planet.SetMass(2)
planet.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
planet.SetName("Planet")
system.Add(planet)


planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().rad = r_planet
planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
planet_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
planet.AddVisualShape(planet_shape)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, carrier, 
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                 chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
motor.SetSpeedFunction(chrono.ChFunction_Const(0.5))  
system.Add(motor)


rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(carrier, planet, 
                      chrono.ChCoordsysD(chrono.ChVectorD(d, 0, 0),
                      chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
system.Add(rev_planet)


gear = chrono.ChLinkGear()
gear.Initialize(sun, planet, 
                chrono.ChVectorD(0, 0, 0),  
                chrono.ChVectorD(0, 0, 1),  
                chrono.ChVectorD(0, 0, 1))  
gear.SetTau(r_sun / r_planet)  
system.Add(gear)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -2, 0.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1,1,1), chrono.ChVectorD(0,0,0), 10, 1, 10, 40, 512)


timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)