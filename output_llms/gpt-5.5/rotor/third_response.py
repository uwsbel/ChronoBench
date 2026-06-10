import math as m

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# PardisoMKL is optional and depends on the PyChrono build.
try:
    import pychrono.pardisomkl as mkl
    HAVE_MKL = True
except ImportError:
    HAVE_MKL = False


# -------------------------------------------------------------------------
# Custom motor speed function
# -------------------------------------------------------------------------

class ChFunctionMyFun(chrono.ChFunction):
    """
    Custom angular-speed function for the rotational motor.

    Returned value is the imposed angular speed in rad/s.
    The function uses piecewise behavior:
      1. Smooth ramp from 0 to A1 over [0, T1)
      2. Oscillating speed around A1 over [T1, T2)
      3. Ramp from A1 to A2 with a smaller oscillation over [T2, T3)
      4. Constant speed A2 after T3
    """

    def __init__(self, A1=40.0, A2=80.0, T1=1.0, T2=4.0, T3=7.0, w=2.0 * m.pi):
        chrono.ChFunction.__init__(self)
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < 0.0:
            return 0.0

        if x < self.T1:
            # Smooth half-cosine ramp from 0 to A1
            return 0.5 * self.A1 * (1.0 - m.cos(m.pi * x / self.T1))

        elif x < self.T2:
            # Oscillate around A1
            return self.A1 + 0.15 * self.A1 * m.sin(self.w * (x - self.T1))

        elif x < self.T3:
            # Ramp from A1 to A2, with smaller oscillation
            s = (x - self.T2) / (self.T3 - self.T2)
            return self.A1 + (self.A2 - self.A1) * s + 0.05 * self.A2 * m.sin(self.w * (x - self.T2))

        else:
            # Final constant speed
            return self.A2

    def Clone(self):
        return ChFunctionMyFun(self.A1, self.A2, self.T1, self.T2, self.T3, self.w)


# -------------------------------------------------------------------------
# System setup
# -------------------------------------------------------------------------

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

beam_L = 6.0
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi

# -------------------------------------------------------------------------
# Beam section: inertia and material properties
# -------------------------------------------------------------------------

area = CH_PI * (beam_ro ** 2 - beam_ri ** 2)
Iyy = (CH_PI / 4.0) * (beam_ro ** 4 - beam_ri ** 4)
Izz = Iyy
J = (CH_PI / 2.0) * (beam_ro ** 4 - beam_ri ** 4)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# -------------------------------------------------------------------------
# Build the IGA beam
# -------------------------------------------------------------------------

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    1
)

beam_nodes = builder.GetLastBeamNodes()
node_front = beam_nodes[0]
node_back = beam_nodes[-1]
node_mid = beam_nodes[len(beam_nodes) // 2]

# -------------------------------------------------------------------------
# Flywheel attached to the beam center
# -------------------------------------------------------------------------

mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    0.24,
    0.1,
    7800
)

mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# -------------------------------------------------------------------------
# Fixed truss/support
# -------------------------------------------------------------------------

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# -------------------------------------------------------------------------
# End bearing: free translation along X and rotation about X
# -------------------------------------------------------------------------

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    node_back,
    truss,
    chrono.ChFramed(node_back.GetPos())
)
sys.Add(bearing)

# -------------------------------------------------------------------------
# Rotational speed motor at the front of the beam
# -------------------------------------------------------------------------

rotmotor1 = chrono.ChLinkMotorRotationSpeed()

rotmotor1.Initialize(
    node_front,
    truss,
    chrono.ChFramed(
        node_front.GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)
    )
)
sys.Add(rotmotor1)

motor_fun = ChFunctionMyFun(
    A1=40.0,
    A2=80.0,
    T1=1.0,
    T2=4.0,
    T3=7.0,
    w=2.0 * CH_PI
)

# For ChLinkMotorRotationSpeed, use SetSpeedFunction.
rotmotor1.SetSpeedFunction(motor_fun)

# -------------------------------------------------------------------------
# FEA visualization
# -------------------------------------------------------------------------

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# -------------------------------------------------------------------------
# Solver setup
# -------------------------------------------------------------------------

if HAVE_MKL:
    msolver = mkl.ChSolverPardisoMKL()
    sys.SetSolver(msolver)
else:
    msolver = chrono.ChSolverMINRES()
    msolver.SetMaxIterations(200)
    msolver.SetTolerance(1e-10)
    msolver.EnableDiagonalPreconditioner(True)
    sys.SetSolver(msolver)

# Optional: uncomment this block to use a more precise HHT timestepper.
# timestepper = chrono.ChTimestepperHHT(sys)
# timestepper.SetAlpha(-0.2)
# timestepper.SetMaxiters(20)
# timestepper.SetAbsTolerances(1e-6)
# timestepper.SetMode(chrono.ChTimestepperHHT.POSITION)
# timestepper.SetScaling(True)
# timestepper.SetVerbose(False)
# sys.SetTimestepper(timestepper)

# Static initialization
sys.DoStaticLinear()

# -------------------------------------------------------------------------
# Irrlicht visualization
# -------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Test FEA: Jeffcott rotor with IGA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 1, 4),
    chrono.ChVector3d(beam_L / 2.0, 0, 0)
)
vis.AddTypicalLights()

# -------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------

time_step = 0.002

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)