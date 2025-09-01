import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create a Chrono::Engine physical system
chrono.SetChronoDataPath("/path/to/chrono/data")  # Replace with the actual path
my_system = chrono.ChSystemNSC()

# Set gravity
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))