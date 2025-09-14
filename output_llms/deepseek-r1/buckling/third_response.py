```python
import math as m
import pychrono as chrno
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronicls
import os

# Custom function class for motor angle:
class ChFunctionMyFun(chrno.ChFunction):
    def __init__(self):
        chrno.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrno.CH_PI
        else:
            return -chrno.CH_PI * (1.0 - m.cos(chrno.CH_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrno.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrno.ChSystemSMC()

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrno.ChVector3d(0, 0, 0)
vC = chrno.ChVector3d(L, 0, 0)
vB = chrno.ChVector3d(L, -H, 0)
vG = chrno.ChVector3d(L - K, -H, 0)
vd = chrno.ChVector3d(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_trss = chrno.ChBody()
body_trss.SetFixed(True)
sys.Add(body_trss)

# Attach a visualization shape to the truss
boxtruss = chrno.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrno.ChFramed(chrno.ChVector3d(-0.01, 0, 0), chrno.QUNIT))

# Create a crank body:
body_crank = chrno.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

# Attach a visualization shape to the crank
boxcrank = chrno.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrno.ChLinkMotorRotationSpeed()
motor.Initialize(body_trss, body_crank, chrno.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetSpeedFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia