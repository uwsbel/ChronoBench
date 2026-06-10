"""Beam-buckling mechanism (PyChrono FEA, ChSystemSMC).

Models a planar four-element flexible mechanism:
  * a fixed truss body (ground reference),
  * a rigid crank body driven by a rotational-angle motor whose angle follows a
    custom ChFunction (a smooth cosine ramp that holds at +pi after t=0.4 s),
  * a horizontal IGA (Cosserat) beam clamped at point A and running to point C,
  * a vertical Euler beam from C down to B,
  * a crank Euler beam from G to B,
linked together by ChLinkMateGeneric constraints so that the motor-driven crank
forces the slender vertical beam to buckle. Stiff-beam FEA is solved with the
Pardiso MKL direct solver and the HHT timestepper. Expected behavior: the crank
sweeps through ~pi, compressing the vertical beam which bows out (buckles); the
coloured ChVisualShapeFEA field shows the bending moment, node-csys glyphs show
the deformed configuration.
"""

import math as m
import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr


# === Custom motor angle function ===
# Smooth cosine ramp from 0 to -pi over 0.4 s, then holds at +pi — drives the crank.
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# === Named constants & derived geometry ===
time_step = 0.001          # stiff-beam timestep
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once

L = 1.2                    # mechanism length
H = 0.3                    # mechanism height
K = 0.07                   # crank length

vA = chrono.ChVector3d(0, 0, 0)        # horizontal beam root (clamped)
vC = chrono.ChVector3d(L, 0, 0)        # horizontal beam tip / vertical beam top
vB = chrono.ChVector3d(L, -H, 0)       # vertical beam bottom
vG = chrono.ChVector3d(L - K, -H, 0)   # crank pivot
vd = chrono.ChVector3d(0, 0, 0.0001)   # tiny offset to break the symmetry

# === System & gravity ===
# SMC system: required for the stiff FEA beam stiffness matrices.
sys = chrono.ChSystemSMC()

# === Bodies (truss + crank) ===
body_truss = chrono.ChBody()
body_truss.SetFixed(True)              # ground reference, immobile
sys.AddBody(body_truss)
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# === Motor (truss -> crank) ===
# Rotational-angle motor with the custom cosine-ramp angle function at pivot G.
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# === FEA mesh & beams ===
# FEA beams: no contact material needed — driven by constraints + motor only.
mesh = fea.ChMesh()

# Horizontal beam: IGA (Cosserat) rectangular section, clamped at A.
beam_wy = 0.12             # section width, Y
beam_wz = 0.012            # section width, Z
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
builder_iga.GetLastBeamNodes().front().SetFixed(True)   # clamp the horizontal beam root
node_tip = builder_iga.GetLastBeamNodes()[-1]           # cache: tip node reused for constraint
node_mid = builder_iga.GetLastBeamNodes()[17]           # cache: mid node for logging

# Vertical beam: slender Euler circular section, C -> B (this is the one that buckles).
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03             # circular diameter
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
node_top = builderA.GetLastBeamNodes()[0]               # cache: top node reused below
node_down = builderA.GetLastBeamNodes()[-1]             # cache: bottom node reused below

# Pin the horizontal-beam tip to the vertical-beam top (translational only).
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam: stiffer Euler circular section, G -> B.
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054        # circular diameter
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
node_crankG = builderB.GetLastBeamNodes()[0]            # cache: crank root node
node_crankB = builderB.GetLastBeamNodes()[-1]           # cache: crank end node

# === Joints / constraints ===
# Weld the crank-beam root to the rigid crank body (all 6 DOF).
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Couple the vertical-beam bottom to the crank-beam end (leave one rotation free).
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(False)        # forced buckling response — disable FEA self-gravity
sys.Add(mesh)

# === FEA visualization (surface field + node glyphs) ===
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

# === Solver & timestepper ===
# Pardiso MKL direct solver handles the stiff beam stiffness matrices.
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)
# HHT for less numerical damping / higher accuracy on the stiff dynamics.
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === Main loop ===

frame = 0
try:
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                           chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
