import pychrono as chrono
import math

# Create the PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create visualization assets
mesh_sun = chrono.ChVisualShapeSphere(0.05)
mesh_sun.SetColor(chrono.ChColor(0.8, 0.1, 0.1))
mesh_planet = chrono.ChVisualShapeSphere(0.05)
mesh_planet.SetColor(chrono.ChColor(0.1, 0.3, 0.8))
mesh_carrier = chrono.ChVisualShapeCylinder(0.03, 0.15)
mesh_carrier.SetColor(chrono.ChColor(0.1, 0.1, 0.6))
mesh_truss = chrono.ChVisualShapeBox(0.2, 0.02, 0.2)
mesh_truss.SetColor(chrono.ChColor(0.2, 0.2, 0.2))

# Create fixed truss support
truss = chrono.ChBodyEasyBox(0.2, 0.02, 0.2, 1000, True, True)
truss.SetName("Epicyclic Gear Truss")
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, -0.5, 0))
truss.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.2))
system.AddBody(truss)

# Create rotating carrier bar
carrier = chrono.ChBodyEasyBox(0.15, 0.02, 0.15, 1000, True, True)
carrier.SetName("Epicyclic Carrier")
carrier.SetFixed(False)
carrier.SetPos(chrono.ChVector3d(0, 0, 0))
carrier.SetRot(chrono.QuatFromAngleZ(0.5 * math.pi))
carrier.GetVisualShape(0).AddVisualShape(mesh_carrier)
system.AddBody(carrier)

# Create sun gear
sun = chrono.ChBodyEasySphere(0.05, 1000, True, True)
sun.SetName("Epicyclic Sun Gear")
sun.SetFixed(True)
sun.SetPos(chrono.ChVector3d(0, 0.1, 0))
sun.GetVisualShape(0).AddVisualShape(mesh_sun)
system.AddBody(sun)

# Create planet gear
planet = chrono.ChBodyEasySphere(0.05, 1000, True, True)
planet.SetName("Epicyclic Planet Gear")
planet.SetFixed(False)
planet.SetPos(chrono.ChVector3d(0, 0.1, 0))
planet.GetVisualShape(0).AddVisualShape(mesh_planet)
system.AddBody(planet)

# Create gear mesh constraint
gear = chrono.ChLinkGear()
gear.SetName("Epicyclic Gear Mesh")
gear.Initialize(sun, planet, True, True)
gear.SetTeethCount(20)
gear.SetTeethCount2(20)
gear.SetCenterDistance(0.1)
system.AddLink(gear)

# Create motor to drive carrier rotation
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Epicyclic Carrier Motor")
motor.Initialize(carrier, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(0.5))
system.AddLink(motor)

# Add visualization lights
light = chrono.ChVisualSystemIrrlicht()
light.AttachSystem(system)
light.SetWindowSize(1024, 720)
light.SetWindowTitle("PyChrono Epicyclic Gear System")
light.Initialize()
light.AddLightPoint(chrono.ChVector3d(0, 0.5, 0.5), 3.0, chrono.ChColor(1.0, 0.9, 0.6))
light.AddLightPoint(chrono.ChVector3d(0, -0.5, -0.5), 3.0, chrono.ChColor(0.6, 0.9, 1.0))

# Simulation loop
while light.Run():
    system.DoStepDynamics(0.001)
    light.BeginScene()
    light.Render()
    light.EndScene()