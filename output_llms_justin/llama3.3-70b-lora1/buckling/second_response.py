import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

# =============================================================================

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# ---------------------------------------------------------------------

out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

sys = chrono.ChSystemSMC()

# Define key geometrical parameters
L = 1.2  # Length
H = 0.3  # Height
K = 0.07  # Crank length
vA = chrono.ChVector3d(0, 0, 0)  # Point A
vC = chrono.ChVector3d(L, 0, 0)  # Point C
vB = chrono.ChVector3d(L, -H, 0)  # Point B
vG = chrono.ChVector3d(L - K, -H, 0)  # Point G
vd = chrono.ChVector3d(0, 0, 0.0001)  # Small offset vector

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)  # Make the truss immobile
sys.AddBody(body_truss)  # Add the truss to the physical system

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  # Set the position of the crank body
sys.AddBody(body_crank)  # Add the crank to the physical system

# Attach a visualization shape to the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  # Initialize motor between truss and crank
myfun = ChFunctionMyFun()  # Create an instance of the custom function
motor.SetAngleFunction(myfun)  # Set the angle function for the motor
sys.Add(motor)  # Add the motor to the system

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12  # Width in Y direction
beam_wz = 0.012  # Width in Z direction

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # Define the rectangular section with density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  # Set Young's modulus for elasticity
melasticity.SetShearModulusFromPoisson(0.3)  # Set shear modulus using Poisson's ratio
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  # Define the section dimensions

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # Create beam section
msection1.SetDrawThickness(beam_wy, beam_wz)  # Set the drawing thickness

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  # Add IGA beam to mesh

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True
print("error happened with only start ```python")