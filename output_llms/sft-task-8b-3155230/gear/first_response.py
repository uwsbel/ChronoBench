import pychrono as chrono
import math

# Create the PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create the Irrlicht visualization system
visualize = chrono.ChVisualSystemIrrlicht()
visualize.AttachSystem(system)
visualize.SetWindowSize(1024, 768)
visualize.SetWindowTitle('Epicyclic Gear System')
visualize.Initialize()
visualize.AddSkyBox()
visualize.AddCamera(chrono.ChVector3d(0, 3, 5), chrono.ChVector3d(0, 0, 0))
visualize.AddLightDirectional(chrono.ChVector3d(1, 1, 1), chrono.ChVector3d(0.5, 0.5, 0.5))

# Create the fixed truss structure
truss = chrono.ChBody()
truss.SetName('Fixed Truss')
truss.SetFixed(True)
truss.EnableCollision(False)
truss.SetMass(100)
truss.SetInertiaXX(chrono.ChVector3d(100, 100, 100))
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.Renderable = True
system.AddBody(truss)

# Create the rotating bar (carrier)
carrier = chrono.ChBody()
carrier.SetName('Carrier')
carrier.SetFixed(False)
carrier.EnableCollision(False)
carrier.SetMass(10)
carrier.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
carrier.SetPos(chrono.ChVector3d(0, 0, 0))
carrier.Renderable = True
system.AddBody(carrier)

# Create the sun gear
sun_gear = chrono.ChBody()
sun_gear.SetName('Sun Gear')
sun_gear.SetFixed(False)
sun_gear.EnableCollision(False)
sun_gear.SetMass(1)
sun_gear.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
sun_gear.SetPos(chrono.ChVector3d(0.2, 0, 0))
sun_gear.Renderable = True
system.AddBody(sun_gear)

# Create the planet gear
planet_gear = chrono.ChBody()
planet_gear.SetName('Planet Gear')
planet_gear.SetFixed(False)
planet_gear.EnableCollision(False)
planet_gear.SetMass(1)
planet_gear.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
planet_gear.SetPos(chrono.ChVector3d(0.2, 0, 0))
planet_gear.Renderable = True
system.AddBody(planet_gear)

# Create the gear joints
sun_joint = chrono.ChLinkRevolute()
sun_joint.SetName('Sun Gear Joint')
sun_joint.Initialize(sun_gear, truss, chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0), chrono.QUNIT))
system.AddLink(sun_joint)

planet_joint = chrono.ChLinkRevolute()
planet_joint.SetName('Planet Gear Joint')
planet_joint.Initialize(planet_gear, carrier, chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0), chrono.QUNIT))
system.AddLink(planet_joint)

# Create the gear mesh
sun_mesh = chrono.ChVisualShapeGear()
sun_mesh.SetRadius(0.1)
sun_mesh.SetTeeth(20)
sun_mesh.SetTeethDepth(0.02)
sun_mesh.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun_gear.AddVisualShape(sun_mesh)

planet_mesh = chrono.ChVisualShapeGear()
planet_mesh.SetRadius(0.1)
planet_mesh.SetTeeth(20)
planet_mesh.SetTeethDepth(0.02)
planet_mesh.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
planet_gear.AddVisualShape(planet_mesh)

# Create the gear motor
motor = chrono.ChLinkMotorGearRatio()
motor.SetName('Gear Motor')
motor.Initialize(carrier, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetGearRatio(1.0)
motor.SetTorque(10)
system.AddLink(motor)

# Create the truss visualization
truss_shape = chrono.ChVisualShapeBox(1, 0.1, 0.1)
truss_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
truss.AddVisualShape(truss_shape)

# Create the carrier visualization
carrier_shape = chrono.ChVisualShapeBox(0.4, 0.02, 0.02)
carrier_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
carrier.AddVisualShape(carrier_shape)

# Create the sun gear visualization
sun_shape = chrono.ChVisualShapeSphere(0.1)
sun_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun_gear.AddVisualShape(sun_shape)

# Create the planet gear visualization
planet_shape = chrono.ChVisualShapeSphere(0.1)
planet_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
planet_gear.AddVisualShape(planet_shape)

# Create the ground plane
ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, False)
ground.SetName('Ground')
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -0.5, 0))
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.AddBody(ground)

# Simulation loop
while visualize.Run():
    system.DoStepDynamics(0.01)
    visualize.BeginScene()
    visualize.Render()
    visualize.EndScene()