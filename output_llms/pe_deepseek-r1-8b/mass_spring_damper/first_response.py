import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, irrlight, vehicle

# Initialize PyChrono system with Non-Smooth Contact (NSC)
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))  # Place ground at z=-1
ground.SetMass(0)  # Ground has no mass
ground.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # Ground has no inertia
ground.SetFixed(True)  # Ground is fixed
sys.Add(ground)

# Create the mass body
mass = chrono.ChBodyEasySphere(0.2, 1000, True, True, chrono.ChContactMaterialSMC())
mass.SetPos(chrono.ChVector3d(0, 0, 5))
sys.Add(mass)

# Create and initialize the TSDA link connecting mass to ground
spring_link = chrono.ChLinkTSDA()
spring_link.Initialize(mass, ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))

# Set spring and damping parameters for the TSDA link
spring_link.SetSpring(100.0)  # Spring constant
spring_link.SetDamping(0.5)  # Damping coefficient

# Set up visualization
vis = irrlight.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mass-Spring-Damper Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  # Position camera

# Add lighting
vis.AddTypicalLights()

# Add visual shapes
mass_shape = chrono.ChVisualShapeSphere(0.3, chrono.ChColor(1, 0, 0))  # Red sphere for mass
spring_shape = chrono.ChVisualShapeCylinder(0.05, 0.2, chrono.ChColor(0, 1, 0))  # Green cylinder for spring
vis.AddVisualShape(mass_shape)
vis.AddVisualShape(spring_shape)

# Attach visualization system to simulation
vis.AttachSystem(sys)

# Run simulation
done = False
while not done:
    # Advance simulation
    sys.DoStepDynamics(0.01)
    
    # Check if visualization should continue
    if not vis.Run():
        done = True
        
    # Begin scene
    vis.BeginScene()
    
    # Render frame
    vis.Render()
    
    # End scene
vis.EndScene()

# Close visualization
vis.Destroy()