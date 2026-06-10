"""Beam buckling under a rotating crank — PyChrono FEA (SMC) demo.

Models a slender horizontal IGA Cosserat beam (the buckling column) clamped at
one end to a fixed truss and connected at the other end, through a chain of stiff
Euler beams and a crank, to a rotary motor. A custom ChFunction ramps the crank
angle from 0 to PI over 0.4 s and holds it, forcing axial compression that drives
the slender beam into lateral buckling.

System type : ChSystemSMC (stiff FEA beams; Pardiso MKL direct solver + HHT).
Main bodies : fixed truss, rotating crank body, one IGA buckling beam, two stiff
              Euler beams, motor link, and the node-to-node / node-to-body
              constraints that connect them.
World       : Y-up gravity convention (mesh gravity disabled — forced response).
Expected    : the crank rotates the beam end, the slender column buckles laterally,
              and the bending-moment field is rendered over the deformed beam.
"""

import math as m
import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr


# === Custom motor function === ramp crank angle 0 -> PI over 0.4 s, then hold
class ChFunctionMyFun(chrono.ChFunction):
    """Smooth cosine ramp to PI by t=0.4 s, constant PI thereafter."""

    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# === System & gravity === SMC system for stiff FEA beams (no rigid contact here)
sys = chrono.ChSystemSMC()

# Geometry constants (named — no bare position literals downstream).
L = 1.2          # length of the buckling beam (A->C)
H = 0.3          # vertical drop to the crank pivot
K = 0.07         # crank arm length
time_step = 1e-3
sim_end = 1.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

vA = chrono.ChVector3d(0, 0, 0)        # clamped beam root
vC = chrono.ChVector3d(L, 0, 0)        # driven beam tip
vB = chrono.ChVector3d(L, -H, 0)       # lower crank junction
vG = chrono.ChVector3d(L - K, -H, 0)   # crank pivot on the truss
vd = chrono.ChVector3d(0, 0, 0.0001)   # tiny offset to seed lateral buckling

# === Bodies === fixed truss + rotating crank body
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# === Motor === rotary motor (truss -> crank) driven by the custom angle function
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# === FEA mesh & beams ===
mesh = fea.ChMesh()

# Slender buckling beam — IGA Cosserat, rectangular aluminium section.
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
builder_iga.GetLastBeamNodes().front().SetFixed(True)   # clamp the root
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Stiff vertical Euler beam (C -> B), circular section.
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# === Joints / constraints === tie buckling-beam tip to the vertical beam top
# FEA beams: no contact material needed — driven by constraints + motor only.
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

# Stiff crank Euler beam (G -> B), circular section.
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Weld crank-beam root to the rotating crank body (all 6 DOF).
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Hinge the vertical beam bottom to the crank-beam end (free about one axis).
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(False)   # forced buckling response, not gravity sag
sys.Add(mesh)

# === FEA visualization === bending-moment surface + node-coordinate glyphs
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

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))
vis.AddTypicalLights()

# === Solver & timestepper === Pardiso MKL direct solver + HHT for stiff beams
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Main loop === render the deforming beam while stepping the dynamics

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                           chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise
