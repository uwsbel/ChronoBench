"""
Beam buckling simulation with finite element analysis.

Models a crank-driven mechanism that induces buckling in a vertical beam:
- Horizontal IGA beam (cantilever at left end)
- Vertical Euler beam (buckling column)
- Crank beam driven by a custom-angle motor function
- Constraints connect horizontal/vertical/crank beams

System: ChSystemSMC (penalty-based smooth contact method)
Solver: ChSolverPardisoMKL (direct solver for stiff beam matrices)
Timestepper: ChTimestepperHHT (minimal-damping implicit integrator)
"""

import csv
import math as m
import os

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as pardiso

# === Custom motor angle function ===
class ChFunctionMyFun(chrono.ChFunction):
    """Custom angle profile: smooth ramp to pi over 0.4s, then hold."""

    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# Keep references alive for SWIG director (prevents GC-induced RuntimeError)
_keep_alive = []


# === System & solver ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # FEA handles gravity internally

pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Geometry ===
L = 1.0       # horizontal beam length
H = 0.25      # vertical beam height
K = 0.05      # crank length
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)   # small Z offset to avoid degeneracy

# === Truss body (fixed) ===
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)
boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# === Crank body ===
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)
boxcrank = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)

# === Motor between truss and crank ===
my_motor_fun = ChFunctionMyFun()  # must stay alive for SWIG director
_keep_alive.append(my_motor_fun)
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
motor.SetAngleFunction(my_motor_fun)
sys.Add(motor)

# === FEM mesh ===
mesh = fea.ChMesh()

# Horizontal beam: IGA Cosserat beam
beam_wy = 0.10
beam_wz = 0.01
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
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Vertical beam: Euler beam
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.024
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.0)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 3, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Horizontal-vertical beam constraint
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)
sphereconstr2 = chrono.ChVisualShapeSphere(0.01)
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam: Euler beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.048
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.0)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 3, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Crank-to-body constraint
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Vertical-to-crank constraint
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# === FEA visualization ===
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
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beams and constraints")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0), chrono.ChVector3d(L / 2, -H / 2, 0))
vis.AddTypicalLights()

# === Main loop ===
time_step = 0.001
sim_end = 2.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

os.makedirs("frames", exist_ok=True)

# review-only CSV logging


frame = 0
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(
        vis, 0.05, 0.05, 20, 20,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    )
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
