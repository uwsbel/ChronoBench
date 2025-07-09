# Import necessary libraries
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize PyChrono environment
chrono.SetChronoDataPath('./data')

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Mass-Spring-Damper Simulation')
vis.Initialize()

# Create the physical system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 1, 10))
ground.AddVisualShape(ground_shape)
sys.Add(ground)

# Create the mass body
mass = chrono.ChBody()
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
mass.SetPos(chrono.ChVector3d(0, 2, 0))
mass_shape = chrono.ChVisualShapeSphere(0.2)
mass.AddVisualShape(mass_shape)
sys.Add(mass)

# Create the spring-damper connection
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
spring.SetSpringConstant(100.0)  # N/m
spring.SetDampingCoefficient(5.0)  # Ns/m
spring.SetRestLength(0.0)  # m
sys.Add(spring)

# Add lighting and camera
vis.AttachSystem(sys)
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)