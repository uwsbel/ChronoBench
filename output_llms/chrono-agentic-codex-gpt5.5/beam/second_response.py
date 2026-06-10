"""Euler-Bernoulli beam demo using PyChrono FEA.

The model uses a Y-up ChSystemSMC with a Pardiso MKL direct solver and HHT
timestepper. A ChBuilderBeamEuler object creates a five-element beam from
(0, 0, -0.1) to (0.2, 0, -0.1); the last node is fixed directly and the first
node receives a downward load. A separate short beam shows node-1 clamping
through ChLinkMateGeneric instead of direct node fixing.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === beam dimensions, material, solver cadence, and review render cadence
TIME_STEP = 1.0e-3
SIM_END = 2.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

BEAM_ELEMENTS = 5
BEAM_START = chrono.ChVector3d(0.0, 0.0, -0.1)
BEAM_END = chrono.ChVector3d(0.2, 0.0, -0.1)
BEAM_UP = chrono.VECT_Y
FORCE_ON_FIRST_NODE = chrono.ChVector3d(0.0, -1.0, 0.0)
CLAMP_BEAM_START = chrono.ChVector3d(0.0, 0.0, 0.04)
CLAMP_BEAM_END = chrono.ChVector3d(0.15, 0.0, 0.04)

SECTION_DIAMETER = 0.006
SECTION_DENSITY = 1200.0
YOUNG_MODULUS = 1.0e9
POISSON_RATIO = 0.30
RAYLEIGH_DAMPING = 0.000


# === System & solver === SMC FEA system with direct solver for stiff beam matrices
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Beam mesh === Euler-Bernoulli beam sections built with the Chrono helper object
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(SECTION_DIAMETER)
section.SetDensity(SECTION_DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON_RATIO)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, BEAM_ELEMENTS, BEAM_START, BEAM_END, BEAM_UP)
beam_node_container = builder.GetLastBeamNodes()  # cache: keep SWIG container alive
loaded_node = beam_node_container.front()  # cache: first node is reused by force and logging
last_node = beam_node_container.back()  # cache: last node is reused by fixing and logging

last_node.SetFixed(True)
loaded_node.SetForce(FORCE_ON_FIRST_NODE)

clamp_builder = fea.ChBuilderBeamEuler()
clamp_builder.BuildBeam(mesh, section, BEAM_ELEMENTS, CLAMP_BEAM_START, CLAMP_BEAM_END, BEAM_UP)
clamp_node_container = clamp_builder.GetLastBeamNodes()  # cache: keep SWIG container alive
hnode1 = clamp_node_container.front()  # cache: node fixed through MateGeneric below
clamp_tip = clamp_node_container.back()  # cache: logged to verify the clamped beam remains stable

# hnode1.SetFixed(True) is intentionally not used; the clamp is a MateGeneric constraint.

# FEA beam: no contact material needed - driven by constraints and applied nodal load only.
sys.Add(mesh)


# === Constraints === clamp node 1 through ChLinkMateGeneric instead of direct fixed flag
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

node1_clamp = chrono.ChLinkMateGeneric()
node1_clamp.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())
node1_clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(node1_clamp)


# === Visualization === FEA bending moment field, node glyphs, and full Irrlicht window
beam_surface = chrono.ChVisualShapeFEA(mesh)
beam_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_surface.SetColorscaleMinMax(-0.4, 0.4)
beam_surface.SetSmoothFaces(True)
beam_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_surface)

beam_glyphs = chrono.ChVisualShapeFEA(mesh)
beam_glyphs.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
beam_glyphs.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
beam_glyphs.SetSymbolsThickness(0.006)
beam_glyphs.SetSymbolsScale(0.01)
beam_glyphs.SetZbufferHide(False)
mesh.AddVisualShapeFEA(beam_glyphs)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli beam with generic node clamp")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.35, 0.18, 0.35), chrono.ChVector3d(0.1, -0.01, -0.1))
vis.AddTypicalLights()
vis.AddGrid(
    0.05,
    0.05,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.1, -0.08, -0.1), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render the beam and advance the FEA system with bounded sim time
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass
