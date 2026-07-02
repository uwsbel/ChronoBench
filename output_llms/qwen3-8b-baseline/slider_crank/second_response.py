import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # Create a box with dimensions 3x1x3 meters and density 1000 kg/m^3
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density
print("error happened with only start ```python")