import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create a Chrono physical system
system = chrono.ChSystemNSC()

# Create a fixed body (ground)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Create a moving body (mass)
mass = chrono.ChBody()
mass.SetMass(1.0)  # Mass in kg
mass.SetInertiaXX(chrono.ChVector3(1.0, 1.0, 1.0))  # Inertia
mass.SetPos(chrono.ChVector3(0, 1, 0))  # Initial position
system.Add(mass)

# Create a spring-damper between the ground and the mass
spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, False, chrono.ChVector3(0, 0, 0), chrono.ChVector3(0, 1, 0))
spring_damper.SetSpringCoefficient(100.0)  # Spring coefficient (N/m)
spring_damper.SetDampingCoefficient(5.0)  # Damping coefficient (N*s/m)
system.Add(spring_damper)

# Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3(0, 2, -4))
vis.AddTypicalLights()

# Visual shapes for the mass and spring
sphere = chrono.ChVisualShapeSphere(0.2)
mass.AddVisualShape(sphere)

spring_damper.AddVisualShape(chrono.ChVisualShapeSpring(0.1, 80, 15))

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)

# No code needs to be returned outside the code block.

# The above Python code is the solution.