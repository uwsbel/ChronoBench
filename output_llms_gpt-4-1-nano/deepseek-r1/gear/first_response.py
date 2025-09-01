import pychrono as chrono
import pychrono.irrlicht as irr

# System setup
system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Parameters
gear_ratio = -2.0  # Negative for external meshing
bar_length = 0.3    # Length of rotating bar
sun_radius = 0.2    # Sun gear radius
planet_radius = 0.1 # Planet gear radius
motor_speed = 0.5   # rad/s

# Create fixed truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetCollide(False)

# Visualization for truss
truss_box = chrono.ChBoxShape()
truss_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.AddAsset(truss_box)
truss.AddAsset(chrono.ChColorAsset(0.6, 0.6, 0.6))
system.Add(truss)

# Create sun gear (fixed to truss)
sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
sun_gear.SetMass(1)
sun_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Sun gear visualization
sun_shape = chrono.ChCylinderShape()
sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
sun_shape.GetCylinderGeometry().rad = sun_radius
sun_gear.AddAsset(sun_shape)
sun_gear.AddAsset(chrono.ChColorAsset(0.8, 0.1, 0.1))
system.Add(sun_gear)

# Fix sun gear to truss with locked revolute
revolute_sun = chrono.ChLinkLockRevolute()
revolute_sun.Initialize(truss, sun_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.Add(revolute_sun)
revolute_sun.SetMotionType(chrono.ChLinkLockRevolute.MOTION_LOCKED)

# Create rotating bar
bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Bar visualization
bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().Size = chrono.ChVectorD(bar_length/2, 0.02, 0.02)
bar.AddAsset(bar_shape)
bar.AddAsset(chrono.ChColorAsset(0.1, 0.8, 0.1))
system.Add(bar)

# Connect bar to truss with motorized revolute
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetAngleFunction(chrono.ChFunction_Ramp(0, motor_speed))
system.Add(motor)

# Create planet gear
planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(bar_length, 0, 0))
planet_gear.SetMass(1)
planet_gear.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Planet gear visualization
planet_shape = chrono.ChCylinderShape()
planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
planet_shape.GetCylinderGeometry().rad = planet_radius
planet_gear.AddAsset(planet_shape)
planet_gear.AddAsset(chrono.ChColorAsset(0.1, 0.1, 0.8))
system.Add(planet_gear)

# Connect planet gear to bar with free revolute
revolute_planet = chrono.ChLinkLockRevolute()
revolute_planet.Initialize(bar, planet_gear, 
                          chrono.ChCoordsysD(chrono.ChVectorD(bar_length, 0, 0)))
system.Add(revolute_planet)

# Create gear constraint between sun and planet
gear_link = chrono.ChLinkGear()
gear_link.Initialize(revolute_sun, revolute_planet)
gear_link.SetTransmissionRatio(gear_ratio)
system.Add(gear_link)

# Irrlicht visualization setup
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 0.75))
vis.AddTypicalLights()

# Simulation loop
timestep = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)