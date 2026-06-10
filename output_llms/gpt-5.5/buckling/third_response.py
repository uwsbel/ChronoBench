import math as m
import os

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# PardisoMKL is optional depending on the PyChrono installation.
try:
    import pychrono.pardisomkl as pardisomkl
    HAS_PARDISO_MKL = True
except ImportError:
    pardisomkl = None
    HAS_PARDISO_MKL = False


# ---------------------------------------------------------------------
# Custom function class for motor angle
# Smoothly ramps from 0 to pi in the first 0.5 seconds, then stays at pi.
# ---------------------------------------------------------------------
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.5)) / 2.0


# ---------------------------------------------------------------------
# Output directory
# ---------------------------------------------------------------------
out_dir = chrono.GetChronoOutputPath() + "BEAM_FIXED"
os.makedirs(out_dir, exist_ok=True)


# ---------------------------------------------------------------------
# Create the Chrono physical system
# ---------------------------------------------------------------------
sys = chrono.ChSystemSMC()


# ---------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------
L = 1.2
H = 0.4
K = 0.07

vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)

# No artificial offset: endpoints should match exactly for constraints.
vd = chrono.ChVector3d(0, 0, 0)


# ---------------------------------------------------------------------
# Fixed truss body
# ---------------------------------------------------------------------
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_truss.AddVisualShape(
    boxtruss,
    chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT)
)


# ---------------------------------------------------------------------
# Crank body
# ---------------------------------------------------------------------
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)

boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)


# ---------------------------------------------------------------------
# Rotational angle motor
# ---------------------------------------------------------------------
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(
    body_truss,
    body_crank,
    chrono.ChFramed(vG, chrono.QUNIT)
)

myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)


# ---------------------------------------------------------------------
# FEM mesh
# ---------------------------------------------------------------------
mesh = fea.ChMesh()


# ---------------------------------------------------------------------
# Horizontal IGA beam
# ---------------------------------------------------------------------
beam_wy = 0.12
beam_wz = 0.15

# Correct spelling: ChInertiaCosseratSimple
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

# Correct section type for Cosserat/IGA beams
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()

# Important correction:
# The orientation vector must not be parallel to the beam axis.
# The beam is along global X, so use global Y as section orientation.
builder_iga.BuildBeam(
    mesh,
    msection1,
    30,
    vA,
    vC,
    chrono.ChVector3d(0, 1, 0),
    3
)

iga_nodes = builder_iga.GetLastBeamNodes()

# Python lists do not have .front()
iga_nodes[0].SetFixed(True)

# Avoid invalid hard-coded indices such as [65].
node_tip = iga_nodes[-1]
node_mid = iga_nodes[len(iga_nodes) // 2]


# ---------------------------------------------------------------------
# Vertical Euler beam
# ---------------------------------------------------------------------
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05

E2 = 75.0e9
nu2 = 0.25
G2 = E2 / (2.0 * (1.0 + nu2))

section2.SetDensity(2500)
section2.SetYoungModulus(E2)
section2.SetShearModulus(G2)
section2.SetRayleighDamping(0.0)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(
    mesh,
    section2,
    10,
    vC + vd,
    vB + vd,
    chrono.ChVector3d(1, 0, 0)
)

vertical_nodes = builderA.GetLastBeamNodes()

# Correct top node index is 0, not 1.
node_top = vertical_nodes[0]
node_down = vertical_nodes[-1]


# ---------------------------------------------------------------------
# Constraint between horizontal beam tip and vertical beam top
# Use a generic mate for a rigid connection.
# ---------------------------------------------------------------------
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(
    node_top,
    node_tip,
    False,
    node_top.Frame(),
    node_top.Frame()
)
constr_bb.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_bb)

sphereconstr2 = chrono.ChVisualShapeSphere(0.02)
constr_bb.AddVisualShape(sphereconstr2)


# ---------------------------------------------------------------------
# Crank beam
# ---------------------------------------------------------------------
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06

E3 = 75.0e9
nu3 = 0.25
G3 = E3 / (2.0 * (1.0 + nu3))

section3.SetDensity(2800)
section3.SetYoungModulus(E3)
section3.SetShearModulus(G3)
section3.SetRayleighDamping(0.0)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(
    mesh,
    section3,
    4,
    vG + vd,
    vB + vd,
    chrono.ChVector3d(0, 1, 0)
)

crank_nodes = builderB.GetLastBeamNodes()
node_crankG = crank_nodes[0]
node_crankB = crank_nodes[-1]


# ---------------------------------------------------------------------
# Constraint between crank beam and crank body
# Use a generic mate as a fixed/welded connection.
# ---------------------------------------------------------------------
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(
    node_crankG,
    body_crank,
    False,
    node_crankG.Frame(),
    node_crankG.Frame()
)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_cbd)


# ---------------------------------------------------------------------
# Constraint between vertical beam bottom and crank beam end
# Revolute-like planar connection: constrain translations and rotations
# about X and Y, leave rotation about Z free.
# ---------------------------------------------------------------------
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(
    node_down,
    node_crankB,
    False,
    node_crankB.Frame(),
    node_crankB.Frame()
)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)
sys.Add(constr_bc)

sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)


# ---------------------------------------------------------------------
# Finalize FEM mesh
# ---------------------------------------------------------------------
mesh.SetAutomaticGravity(True)
sys.Add(mesh)


# ---------------------------------------------------------------------
# FEM visualization
# ---------------------------------------------------------------------
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_VECTORS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_FULL)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)


# ---------------------------------------------------------------------
# Irrlicht visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Corrected Beam Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.jpg"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, 0.4, -1.6),
    chrono.ChVector3d(0.6, -0.2, 0.0)
)
vis.AddTypicalLights()


# ---------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------
if HAS_PARDISO_MKL:
    custom_solver = pardisomkl.ChSolverPardisoMKL()
else:
    # Fallback if PardisoMKL is unavailable.
    custom_solver = chrono.ChSolverMINRES()
    custom_solver.SetMaxIterations(200)
    custom_solver.SetTolerance(1e-10)
    custom_solver.EnableDiagonalPreconditioner(True)

sys.SetSolver(custom_solver)


# ---------------------------------------------------------------------
# Time stepper
# Use HHT for stiff FEA systems instead of explicit Euler.
# ---------------------------------------------------------------------
ts = chrono.ChTimestepperHHT(sys)
ts.SetAlpha(-0.2)
ts.SetMaxIters(10)
ts.SetAbsTolerances(1e-6)
ts.SetStepControl(False)
sys.SetTimestepper(ts)


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
time_step = 0.002

while vis.Run():
    vis.BeginScene()
    vis.Render()

    chronoirr.drawGrid(
        vis,
        0.1,
        0.01,
        10,
        10,
        chrono.ChCoordsysd(
            chrono.ChVector3d(0, 0, 0),
            chrono.QUNIT
        )
    )

    vis.EndScene()
    sys.DoStepDynamics(time_step)