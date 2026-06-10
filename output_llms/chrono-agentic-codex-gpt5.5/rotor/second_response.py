"""Flexible rotor on an IGA beam in a PyChrono SMC system.

The model uses a 10 m hollow Cosserat beam with a sine-driven flywheel welded to
the mid-span FEA node. The beam is constrained by end bearings, runs under
Mars-like gravity, and is viewed from a wide Irrlicht camera angle.
"""

import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants: final requested rotor dimensions and run controls ===
BEAM_L = 10.0
BEAM_RO = 0.060
BEAM_RI = 0.055
BEAM_DENSITY = 7800.0
BEAM_E = 210.0e9
BEAM_NU = 0.3
BEAM_SPANS = 16
BEAM_ORDER = 3

FLYWHEEL_RADIUS = 0.30
FLYWHEEL_THICKNESS = 0.10
FLYWHEEL_DENSITY = 7800.0

TIME_STEP = 0.002
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity: SMC with direct solver for stiff FEA beams ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Flexible beam: hollow Cosserat section rebuilt at the requested size ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

area = math.pi * (BEAM_RO * BEAM_RO - BEAM_RI * BEAM_RI)
iyy = math.pi * (BEAM_RO**4 - BEAM_RI**4) / 4.0
izz = iyy
jpolar = 2.0 * iyy

beam_inertia = fea.ChInertiaCosseratSimple()
beam_inertia.SetDensity(BEAM_DENSITY)
beam_inertia.SetArea(area)
beam_inertia.SetIyy(iyy)
beam_inertia.SetIzz(izz)

beam_elasticity = fea.ChElasticityCosseratSimple()
beam_elasticity.SetYoungModulus(BEAM_E)
beam_elasticity.SetShearModulusFromPoisson(BEAM_NU)
beam_elasticity.SetIyy(iyy)
beam_elasticity.SetIzz(izz)
beam_elasticity.SetJ(jpolar)

beam_section = fea.ChBeamSectionCosserat(beam_inertia, beam_elasticity)
beam_section.SetCircular(True)
beam_section.SetDrawCircularRadius(BEAM_RO)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    beam_section,
    BEAM_SPANS,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(BEAM_L, 0, 0),
    chrono.VECT_Y,
    BEAM_ORDER,
)
beam_nodes_ref = builder.GetLastBeamNodes()  # cache: keep SWIG vector alive
beam_nodes = [beam_nodes_ref[i] for i in range(beam_nodes_ref.size())]  # cache: reused constraints
node_left = beam_nodes[0]  # cache: left bearing node
node_mid = beam_nodes[len(beam_nodes) // 2]  # cache: flywheel attachment node
node_right = beam_nodes[-1]  # cache: right bearing node

# FEA beam: no contact material needed - driven by constraints, gravity, and motor only.
sys.Add(mesh)


# === Rigid flywheel and bearings: a driven disk coupled to the flexible mesh ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

flywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    FLYWHEEL_RADIUS,
    FLYWHEEL_THICKNESS,
    FLYWHEEL_DENSITY,
)
flywheel.SetPos(chrono.ChVector3d(BEAM_L * 0.5, 0, 0))
flywheel.SetRot(chrono.QUNIT)
flywheel.EnableCollision(False)
spin_marker = chrono.ChVisualShapeSphere(0.055)
spin_marker.SetColor(chrono.ChColor(0.9, 0.05, 0.05))
flywheel.AddVisualShape(spin_marker, chrono.ChFramed(chrono.ChVector3d(FLYWHEEL_RADIUS * 0.72, 0, 0)))
sys.Add(flywheel)

weld_flywheel = chrono.ChLinkMateFix()
weld_flywheel.Initialize(node_mid, flywheel)
sys.Add(weld_flywheel)

left_bearing = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
left_bearing.Initialize(node_left, truss, False, node_left.Frame(), node_left.Frame())
sys.Add(left_bearing)

right_bearing = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
right_bearing.Initialize(node_right, truss, False, node_right.Frame(), node_right.Frame())
sys.Add(right_bearing)

f_ramp = chrono.ChFunctionSine(60, 0.1)


# === FEA visualization: surface field plus node glyphs for deformation review ===
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetDefaultMeshColor(chrono.ChColor(0.25, 0.58, 0.85))
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.04)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization: Irrlicht window with requested wide camera position ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Flexible rotor with longer hollow beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(BEAM_L * 0.5, 0, 0))
vis.AddTypicalLights()

sys.DoStaticLinear()


# === Main loop: render once per frame and step the FEA rotor in batches ===


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            now = sys.GetChTime()  # cache: current time used by logging and stop test
            flywheel_pos = flywheel.GetPos()  # cache: sampled once per physics step
            torque_y = f_ramp.GetVal(now)  # cache: sine command sampled once per step
            node_mid.SetTorque(chrono.ChVector3d(0, torque_y, 0))
            flywheel.EmptyAccumulators()
            flywheel.AccumulateTorque(chrono.ChVector3d(0, torque_y, 0), False)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    print(f"Rotor simulation failed: {exc}")
    raise
finally:
    pass
