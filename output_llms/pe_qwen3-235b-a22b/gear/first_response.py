import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the system
sys = chrono.ChSystemNSC()

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create fixed truss
truss = chrono.ChBody()
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
truss_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 0.5, 1))
truss_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
truss.AddVisualShape(truss_shape)
sys.Add(truss)

# Create rotating bar (carrier)
rotating_bar = chrono.ChBody()
rotating_bar.SetPos(chrono.ChVector3d(0, 0, 0))  # Center at origin
rotating_bar_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 0.1, 0.1))  # Length 2m along X-axis
rotating_bar_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
rotating_bar.AddVisualShape(rotating_bar_shape)
rotating_bar.SetMass(10)
rotating_bar.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(rotating_bar)

# Revolute joint between truss and rotating bar (around Z-axis)
revolute_truss_bar = chrono.ChLinkLockRevolute()
revolute_truss_bar.Initialize(truss, rotating_bar, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(0, 0, 0, 1)))
sys.Add(revolute_truss_bar)

# Motor for the rotating bar
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, rotating_bar, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(0, 0, 0, 1)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # 1 rad/s
sys.Add(motor)

# Sun gear (fixed)
sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sun_gear.SetFixed(True)
sun_shape = chrono.ChVisualShapeCylinder(0.2, 0.5)
sun_shape.SetColor(chrono.ChColor(1, 0.5, 0))
# Rotate cylinder to align with Z-axis
sun_gear.AddVisualShape(sun_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(chrono.Q_ROTATE_Y_TO_Z)))
sys.Add(sun_gear)

# Planet gear
planet_gear = chrono.ChBody()
planet_radius = 0.2
planet_gear.SetPos(chrono.ChVector3d(1.0, 0, 0))  # At the end of the rotating bar
planet_shape = chrono.ChVisualShapeCylinder(planet_radius, 0.5)
planet_shape.SetColor(chrono.ChColor(0, 0.5, 1))
planet_gear.AddVisualShape(planet_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(chrono.Q_ROTATE_Y_TO_Z)))
sys.Add(planet_gear)

# Revolute joint between rotating bar and planet gear
revolute_bar_planet = chrono.ChLinkLockRevolute()
revolute_bar_planet.Initialize(rotating_bar, planet_gear, chrono.ChCoordsysD(chrono.ChVector3d(1.0, 0, 0), chrono.ChQuaterniond(0, 0, 0, 1)))
sys.Add(revolute_bar_planet)

# Gear constraint between sun and planet gears
gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, True, chrono.ChFrameD(), chrono.ChFrameD())
gear_ratio = -1.0  # Assuming same number of teeth
gear_constraint.SetRatio(gear_ratio)
sys.Add(gear_constraint)

# Simulation loop
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)