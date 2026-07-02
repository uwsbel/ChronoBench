import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import *
import numpy as np

# Initialize PyChrono system
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
system.SetIterationNumber(100)
system.SetTolerance(1e-6)

# Create Irrlicht visualization
application = irrlicht.ChIrrApp(system, 'Epicyclic Gears Simulation', irrlicht.dimension2duint(800, 600))
application.AddCamera(chrono.ChVectorD(0, 2, 5))
application.SetTimestep(1e-3)
application.SetUseIrrlichtDevice(True)
application.SetShowConsoleWindow(False)

# Create fixed truss (base)
truss = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetFixed(True)
system.Add(truss)

# Create rotating bar (carrier)
carrier = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0.05, 0.05, 0.5), 0.05, 1000, True, True)
carrier.SetPos(chrono.ChVectorD(0, 0, 0.5))
carrier.SetFixed(False)
system.Add(carrier)

# Revolute joint between truss and carrier (z-axis rotation)
joint_truss_carrier = chrono.ChLinkRevolute()
joint_truss_carrier.SetName("Revolute joint truss-carrier")
joint_truss_carrier.Initialize(truss, carrier, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
system.AddLink(joint_truss_carrier)

# Create sun gear (fixed to truss)
sun_gear = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0.2, 0.2, 0.05), 0.2, 1000, True, True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0.25))
sun_gear.SetFixed(True)
system.Add(sun_gear)

# Create planet gear (attached to carrier)
planet_gear = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0.15, 0.15, 0.05), 0.15, 1000, True, True)
planet_gear.SetPos(chrono.ChVectorD(0, 0, 0.65))
planet_gear.SetFixed(False)
system.Add(planet_gear)

# Revolute joint between carrier and planet gear (z-axis rotation)
joint_carrier_planet = chrono.ChLinkRevolute()
joint_carrier_planet.SetName("Revolute joint carrier-planet")
joint_carrier_planet.Initialize(carrier, planet_gear, chrono.ChVectorD(0, 0, 0.15), chrono.ChVectorD(0, 0, 1))
system.AddLink(joint_carrier_planet)

# Apply motor to carrier for constant rotation
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Motor carrier")
motor.Initialize(carrier, truss, chrono.ChVectorD(0, 0, 1))
motor.SetSpeed(chrono.ChVectorD(0, 0, 10))  # 10 radians per second
system.AddLink(motor)

# Set materials for all bodies
def set_material(body):
    body.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    body.GetMaterialSurface().SetFriction(0.5)
    body.GetMaterialSurface().SetRestitution(0.3)

set_material(truss)
set_material(sun_gear)
set_material(carrier)
set_material(planet_gear)

# Add lighting
application.AddLight(chrono.ChLightSun())
application.AddLight(chrono.ChLightPoint(chrono.ChVectorD(0, 10, 10), 1000000))

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())