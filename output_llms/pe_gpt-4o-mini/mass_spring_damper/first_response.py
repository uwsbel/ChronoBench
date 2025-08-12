import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
chrono.SetChronoDataPath('/path/to/data')  # Set the correct path for your Chrono data
my_system = chrono.ChSystemNSC()

# Create the mass body
mass = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
mass.SetPos(chrono.ChVector3d(0, 1, 0))  # Initial position above the ground
mass.SetMass(1.0)  # Mass of the block
my_system.Add(mass)

# Create the spring-damper connection
spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, my_system.Get_ground(), chrono.ChVector2d(0, 0), chrono.ChVector2d(0, 1))
spring_damper.SetSpringK(1000)  # Spring constant
spring_damper.SetDampingR(10)    # Damping coefficient
my_system.Add(spring_damper)

# Set up the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddSkyBox()

# Add visual shapes for the mass (box) and spring (visual representation)
mass_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.2, 0.2, 0.2))
mass_shape.SetColor(chrono.ChColor(0, 0, 1))  # Blue color for the mass
mass.AddVisualShape(mass_shape)

# Simulation loop
time_step = 0.01
while vis.Run():
    # Perform the simulation step
    my_system.DoStepDynamics(time_step)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Clean up and exit
vis.Close()