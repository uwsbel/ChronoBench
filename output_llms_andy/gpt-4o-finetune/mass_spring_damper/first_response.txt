import os
import math
import numpy as np
import matplotlib.pyplot as plt

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono physical system (non-smooth contact)
sys = chrono.ChSystemNSC()

# Add a single mass:
# - Define a cube shape with specified size and create a visual representation
# - Create a body with the defined visual shape
# - Set the position of the body in the 3D space
# - Add the body to the physical system
# - For this example, we'll keep the body fixed (no motion)
# Define the size of the cube
msize = chrono.ChVector3d(1, 1, 1)
# Create a visualization shape for the cube
mcub = chrono.ChVisualShapeCube(msize.x, msize.y, msize.z)
# Set the color of the visual shape
mcub.SetColor(chrono.ChColor(0.6, 0, 0))
# Create a body and add the visual shape to it
mbody = chrono.ChBody()
mbody.AddVisualShape(mcub)
# Set the position of the body in the 3D space
mbody.SetPos(chrono.ChVector3d(0, 0, 0))
# Add the body to the physical system
sys.Add(mbody)
# Keep the body fixed (commented out to allow motion)
# mbody.SetFixed(True)

# Add a spring-damper:
# - Create a spring-damper link (ChLinkTSDA) to connect the body to the ground
# - Define the end points of the spring-damper in absolute coordinates
# - Initialize the spring-damper with the specified end points
# - Set the spring constant and damping coefficient
# - Add a visual representation for the spring-damper
# - Add the spring-damper link to the physical system
# Create a spring-damper link
spring = chrono.ChLinkTSDA()
# Define the end points of the spring-damper in absolute coordinates
spring.Initialize(mbody,                         # body A to connect
                  sys.GetGroundBody(),           # body B to connect
                  True,                          # use left point relative to body A
                  chrono.ChVector3d(0, 0, 0),     # pt. on body A (abs coords)
                  chrono.ChVector3d(3, 0, 0),     # pt. on body B (abs coords)
                  chrono.CoordinateSystem())
# Set the spring constant (N/m)
spring.SetRestLength(3.0)
spring.SetSpringCoefficient(50)
# Set the damping coefficient (N s/m)
spring.SetDampingCoefficient(3)
# Add a visual shape for the spring-damper
springVis = chrono.ChVisualShapeSpring(0.2, 150, 10, 0.1)
spring.AddVisualShapeSpring(springVis)
# Add the spring-damper link to the physical system
sys.Add(spring)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 6, -6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    # Render the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    # Advance the simulation by a time step
    sys.DoStepDynamics(1e-3)