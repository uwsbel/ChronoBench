import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


physics_system = chrono.ChSystemNSC()


visualization_window = chronoirr.ChIrrApp(physics_system, "PyChrono Lidar Simulation", chronoirr.dimension2du(800, 600))


visualization_window.Camera.SetPosition(chrono.ChVectorD(0, 0, -5))
visualization_window.Camera.SetFocalPoint(chrono.ChVectorD(0, 0, 0))