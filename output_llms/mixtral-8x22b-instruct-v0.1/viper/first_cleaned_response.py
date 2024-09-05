import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")  
my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))