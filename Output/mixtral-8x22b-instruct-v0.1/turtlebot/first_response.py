import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create the Chrono::Engine physical system
chrono.SetChronoDataPath("/path/to/chrono/data")
my_system = chrono.ChSystemNSC()

# Set gravity
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))