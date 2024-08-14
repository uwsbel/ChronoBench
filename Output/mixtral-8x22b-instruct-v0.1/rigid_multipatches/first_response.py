import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the Chrono::Engine physical system
physics = chrono.ChSystemNSC()

# Create the Irrlicht visualization system
visualization = chronoirr.ChVisualSystemIrrlicht()

# Attach the visualization system to the physical system
physics.SetVisualSystem(visualization)

# Set the Irrlicht driver (e.g., OPENGL, DIRECT3D9, etc.)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle("HMMWV on Complex Terrain")
visualization.Initialize()