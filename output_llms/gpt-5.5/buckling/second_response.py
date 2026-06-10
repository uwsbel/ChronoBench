import math as m
import os

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr


# Custom function class for motor angle
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def _angle(self, x):
        # Smoothly rotate from 0 to -pi over 0.4 s, then hold the angle.
        # The original script jumped from -pi to +pi at x=0.4.
        if x > 0.4:
            return -chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0

    # Current PyChrono API
    def GetVal(self, x):
        return self._angle(x)

    # Compatibility with older PyChrono APIs
    def Get_y(self, x):
        return self._angle(x)


# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"
os.makedirs(out_dir, exist_ok=True)

# Create a Chrono physical system
sys = chrono.ChSystemSMC()

# ---------------------------------------------------------------------
# Geometry parameters
# ---------------------------------------------------------------------
L = 1.2
H = 0.3
K = 0.07

vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# ---------------------------------------------------------------------
# Truss body
# ---------------------------------------------------------------------
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(
    boxtruss,
    chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT)
)

# ---------------------------------------------------------------------
# Crank body
# ---------------------------------------------------------------------
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# ---------------------------------------------------------------------
# Rotational motor
# ---------------------------------------------------------------------
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))

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
beam_wz = 0.012

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)

iga_nodes = builder_iga.GetLastBeamNodes()
iga_nodes[0].SetFixed(True)

node_tip = iga_nodes[-1]
node_mid = iga_nodes[17]

# ---------------------------------------------------------------------
# Vertical Euler beam
# ---------------------------------------------------------------------
section2 = fea.ChBeamSectionEulerAdvanced()

hbeam_d = 0.03

section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(
    mesh,
    section2,
    6,
    vC + vd,
    vB + vd,
    chrono.ChVector3d(1, 0, 0)
)

vertical_nodes = builderA.GetLastBeamNodes()
node_top = vertical_nodes[0]
node_down = vertical_nodes[-1]

# Constraint between horizontal and vertical beams
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(
    node_top,
    node_tip,
    False,
    node_top.Frame(),
    node_top.Frame()
)
sys.Add(constr_bb)

constr_bb.SetConstrainedCoords(True, True, True, False, False, False)

sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

# ---------------------------------------------------------------------
# Crank Euler beam
# ---------------------------------------------------------------------
section3 = fea.ChBeamSectionEulerAdvanced()

crankbeam_d = 0.054

section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(
    mesh,
    section3,
    5,
    vG + vd,
    vB + vd,
    chrono.ChVector3d(0, 1, 0)
)

crank_nodes = builderB.GetLastBeamNodes()
node_crankG = crank_nodes[0]
node_crankB = crank_nodes[-1]

# Constraint between crank beam and rigid crank body
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(
    node_crankG,
    body_crank,
    False,
    node_crankG.Frame(),
    node_crankG.Frame()
)
sys.Add(constr_cbd)

constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Constraint between vertical beam and crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(
    node_down,
    node_crankB,
    False,
    node_crankB.Frame(),
    node_crankB.Frame()
)
sys.Add(constr_bc)

constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)

# ---------------------------------------------------------------------
# Final FEM setup
# ---------------------------------------------------------------------
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# ---------------------------------------------------------------------
# FEM visualization
# ---------------------------------------------------------------------
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# ---------------------------------------------------------------------
# Irrlicht visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beams and constraints")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Solver and timestepper
# ---------------------------------------------------------------------
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# ---------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------
while vis.Run():
    vis.BeginScene()
    vis.Render()

    chronoirr.drawGrid(
        vis,
        0.05,
        0.05,
        20,
        20,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
    )

    vis.EndScene()

    sys.DoStepDynamics(0.001)