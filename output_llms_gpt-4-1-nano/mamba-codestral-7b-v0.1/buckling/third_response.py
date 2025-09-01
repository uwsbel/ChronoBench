import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronicls  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVectorD(0, 0, 0)
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.GetLastBeamNodes()[32]

# Define vertical beam parameters using Euler beams
section
print("error happened with only start ```python")