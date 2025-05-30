import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity

# Create the fixed truss
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss)

# Add a simple visual shape to the truss (a small box)
truss_box = chrono.ChBoxShape()
truss_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.GetVisualShape().AddShape(truss_box)

# Create the Sun Gear
sun = chrono.ChBody()
sun.SetMass(1.0)
sun.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
sun.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(sun)

# Add a cylinder shape for the sun gear
sun_cyl = chrono.ChCylinderShape()
sun_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
sun_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
sun_cyl.GetCylinderGeometry().rad = 0.666  # Pitch radius of the sun gear
sun.GetVisualShape().AddShape(sun_cyl)

# Apply a red material to the sun gear
mat_sun = chrono.ChMaterialSurfaceNSC()
mat_sun.SetDiffuseColor(chrono.ChColor(1, 0, 0))
sun.GetVisualShape().SetMaterial(0, mat_sun)

# Revolute joint between truss and sun gear
sun_revolute = chrono.ChLinkRevolute()
sun_revolute.Initialize(truss, sun, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(sun_revolute)

# Motor to enforce constant angular speed on the sun gear
omega_sun = 10.0  # rad/s
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(omega_sun))
system.Add(motor)

# Create the Rotating Bar (Carrier)
carrier = chrono.ChBody()
carrier.SetMass(2.0)
carrier.SetInertiaXX(chrono.ChVectorD(0.5, 0.5, 0.5))
carrier.SetPos(chrono.ChVectorD(0.5, 0, 0))  # Center at (0.5, 0, 0)
system.Add(carrier)

# Add a box shape for the carrier
carrier_box = chrono.ChBoxShape()
carrier_box.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.05, 0.05)
carrier.GetVisualShape().AddShape(carrier_box)

# Apply a gray material to the carrier
mat_carrier = chrono.ChMaterialSurfaceNSC()
mat_carrier.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
carrier.GetVisualShape().SetMaterial(0, mat_carrier)

# Revolute joint between truss and carrier
carrier_revolute = chrono.ChLinkRevolute()
carrier_revolute.Initialize(truss, carrier, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(carrier_revolute)

# Create the Planet Gear
planet = chrono.ChBody()
planet.SetMass(0.5)
planet.SetInertiaXX(chrono.ChVectorD(0.02, 0.02, 0.02))
planet.SetPos(chrono.ChVectorD(1, 0, 0))  # At the end of the carrier
system.Add(planet)

# Add a cylinder shape for the planet gear
planet_cyl = chrono.ChCylinderShape()
planet_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.1)
planet_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.1)
planet_cyl.GetCylinderGeometry().rad = 0.333  # Pitch radius of the planet gear
planet.GetVisualShape().AddShape(planet_cyl)

# Apply a blue material to the planet gear
mat_planet = chrono.ChMaterialSurfaceNSC()
mat_planet.SetDiffuseColor(chrono.ChColor(0, 0, 1))
planet.GetVisualShape().SetMaterial(0, mat_planet)

# Revolute joint between carrier and planet gear
planet_revolute = chrono.ChLinkRevolute()
planet_revolute.Initialize(carrier, planet, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0), chrono.QUNIT))
system.Add(planet_revolute)

# Gear Constraint between Sun and Planet
gear_ratio = -2.0  # Assuming sun has 20 teeth, planet has 10 teeth
sun_planet_gear = chrono.ChLinkGear()
sun_planet_gear.Initialize(sun, planet, True, chrono.ChFrameD(), chrono.ChFrameD())
sun_planet_gear.SetRatio(gear_ratio)
system.Add(sun_planet_gear)

# Set up Irrlicht visualization
application = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))

# Bind and update assets
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(1e-3)