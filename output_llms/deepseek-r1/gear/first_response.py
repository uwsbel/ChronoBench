import pychrono as chrono
import pychrono.irrlicht as irr

# Create the system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Zero gravity

# Parameters
r_sun = 0.5      # Sun gear radius
r_planet = 0.2   # Planet gear radius
d = r_sun + r_planet  # Distance between centers
gear_thickness = 0.2  # Thickness of gears
bar_width = 0.1       # Carrier bar dimensions

# Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")
system.Add(ground)

# Create carrier (rotating bar)
carrier = chrono.ChBody()
carrier.SetPos(chrono.ChVectorD(d/2, 0, 0))
carrier.SetMass(5)
carrier.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
carrier.SetName("Carrier")
system.Add(carrier)

# Carrier visualization (bar)
bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().Size = chrono.ChVectorD(d/2, bar_width/2, bar_width/2)
bar_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
carrier.AddVisualShape(bar_shape)

# Create sun gear (fixed to ground)
sun = chrono.ChBody()
sun.SetPos(chrono.ChVectorD(0, 0, 0))
sun.SetBodyFixed(True)
sun.SetMass(3)
sun.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
sun.SetName("Sun")
system.Add(sun)

# Sun gear visualization
sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().rad = r_sun
sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun.AddVisualShape(sun_shape)

# Create planet gear
planet = chrono.ChBody()
planet.SetPos(chrono.ChVectorD(d, 0, 0))
planet.SetMass(2)
planet.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
planet.SetName("Planet")
system.Add(planet)

# Planet gear visualization
planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().rad = r_planet
planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -gear_thickness/2)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, gear_thickness/2)
planet_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
planet.AddVisualShape(planet_shape)

# Create carrier-ground revolute joint with motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, carrier, 
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                 chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
motor.SetSpeedFunction(chrono.ChFunction_Const(0.5))  # 0.5 rad/sec
system.Add(motor)

# Create carrier-planet revolute joint
rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(carrier, planet, 
                      chrono.ChCoordsysD(chrono.ChVectorD(d, 0, 0),
                      chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
system.Add(rev_planet)

# Create sun-planet gear constraint
gear = chrono.ChLinkGear()
gear.Initialize(sun, planet, 
                chrono.ChVectorD(0, 0, 0),  # Reference position
                chrono.ChVectorD(0, 0, 1),  # Axis on sun
                chrono.ChVectorD(0, 0, 1))  # Axis on planet
gear.SetTau(r_sun / r_planet)  # Gear ratio
system.Add(gear)

# Create Irrlicht visualization
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

# Simulation loop
timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)