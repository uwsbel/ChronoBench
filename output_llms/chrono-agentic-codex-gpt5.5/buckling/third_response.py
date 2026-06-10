"""FEA buckling simulation using an SMC Chrono system.

The model is a slender Euler beam column clamped at its lower end and compressed
at its upper end. A small lateral force makes the post-critical side deflection
visible while the HHT timestepper and Pardiso MKL solver integrate the stiff
finite-element dynamics.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === physics, mesh, and recording cadence are fixed once
TIME_STEP = 1.0e-3
SIM_END = 3.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

COLUMN_LENGTH = 1.8
COLUMN_DIAMETER = 0.030
NUM_ELEMENTS = 18
DENSITY = 7800.0
YOUNG_MODULUS = 2.1e10
POISSON_RATIO = 0.30
RAYLEIGH_DAMPING = 0.002
AXIAL_FORCE = 900.0
LATERAL_FORCE = 45.0


# === System & solver === stiff beam buckling needs SMC, direct solve, and HHT
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA column === slender beam with base clamp and loaded free tip
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(COLUMN_DIAMETER)
section.SetDensity(DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON_RATIO)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    NUM_ELEMENTS,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0, COLUMN_LENGTH, 0),
    chrono.ChVector3d(1, 0, 0),
)
beam_nodes_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]  # cache: reused in loop
base_node = beam_nodes[0]  # cache: clamped support node
tip_node = beam_nodes[-1]  # cache: loaded free node
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: lateral deflection sample

base_node.SetFixed(True)
tip_node.SetForce(chrono.ChVector3d(LATERAL_FORCE, -AXIAL_FORCE, 0))

# FEA beam: no contact material needed, because buckling is driven by constraints and nodal loads only.
sys.Add(mesh)


# === FEA visualization === colored bending field plus node coordinate glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-4.0, 4.0)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.004)
vis_glyph.SetSymbolsScale(0.02)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization === Irrlicht window, sky, camera, lights, and reference grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Buckling Column")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.8, 1.0, 2.1), chrono.ChVector3d(0, 0.9, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2.0)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render frames while integrating batched FEA steps
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
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
