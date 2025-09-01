import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np
import sys

sys = chrono.ChSystemNSC()

# ... [other shared data remains the same] ...

# Create crank-rod joint as spherical
mjointA = chrono.ChLinkLockSpherical()
mjointA.Initialize(mrod,
                  mcrank,
                  chrono.ChFramed(crank_center))
sys.Add(mjointA)

# Create rod-piston joint as spherical
mjointB = chrono.ChLinkLockSpherical()
mjointB.Initialize(mpiston,
                  mrod,
                  chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create piston-floor joint as plane-plane
mjointC = chrono.ChLinkLockPlanePlane()
mjointC.Initialize(mpiston,
                  mfloor,
                  chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)),
                  chrono.Q_ROTATE_Z_TO_X)
sys.Add(mjointC)

# ... [rest of the code remains the same] ...