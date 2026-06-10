"""FEA beam simulation using an SMC system, Euler beam elements, and two connected beam segments.

The model builds a fixed-root flexible beam, then adds a second beam segment with
its A node equal to the last node of the first segment and its B point at
(0.2, 0.1, -0.1). The free structure deflects under Y-up gravity while the
Irrlicht view displays beam bending and node glyphs.
"""

import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === named values keep geometry and integration parameters explicit
TIME_STEP = 0.001
SIM_END = 3.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

BEAM_ELEMENTS_PER_SEGMENT = 8
ROOT_POINT = chrono.ChVector3d(0.0, 0.0, 0.0)
FIRST_TIP_POINT = chrono.ChVector3d(0.12, 0.0, 0.0)
SECOND_TIP_POINT = chrono.ChVector3d(0.2, 0.1, -0.1)
SECTION_Y_DIRECTION = chrono.ChVector3d(0.0, 1.0, 0.0)

BEAM_DIAMETER = 0.012
BEAM_DENSITY = 7850.0
YOUNG_MODULUS = 2.0e11
POISSON_RATIO = 0.3
RAYLEIGH_DAMPING = 0.001


# === System & solver === ChSystemSMC plus direct solver is the FEA beam truth shape
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())
timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === FEA mesh === two Euler beam segments share the first segment's last node
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(BEAM_DIAMETER)
section.SetDensity(BEAM_DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON_RATIO)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    BEAM_ELEMENTS_PER_SEGMENT,
    ROOT_POINT,
    FIRST_TIP_POINT,
    SECTION_Y_DIRECTION,
)
first_segment_nodes_ref = builder.GetLastBeamNodes()  # cache: keep SWIG container alive
first_segment_nodes = [first_segment_nodes_ref[i] for i in range(first_segment_nodes_ref.size())]
root_node = first_segment_nodes[0]  # cache: fixed support node
shared_tip_node = first_segment_nodes[-1]  # cache: A node for the next BuildBeam call
root_node.SetFixed(True)

builder.BuildBeam(
    mesh,
    section,
    BEAM_ELEMENTS_PER_SEGMENT,
    shared_tip_node,
    SECOND_TIP_POINT,
    SECTION_Y_DIRECTION,
)
second_segment_nodes_ref = builder.GetLastBeamNodes()  # cache: keep the second container alive
second_segment_nodes = [second_segment_nodes_ref[i] for i in range(second_segment_nodes_ref.size())]
free_tip_node = second_segment_nodes[-1]  # cache: logged and tracked once per step

# FEA beam: no contact material needed because the beam is driven by gravity and constraints only.


# === FEA visualization assets === scalar beam moment plus node glyphs make deformation visible
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.003)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.Add(mesh)


# === Visualization === Irrlicht setup follows the required Initialize-before-scene-elements order
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Connected Euler Beam Segments")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.32, 0.18, 0.32), chrono.ChVector3d(0.09, 0.02, -0.03))
vis.AddTypicalLights()
vis.AddGrid(
    0.05,
    0.05,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.1, -0.04, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at video cadence and advance stiff FEA with fixed-size steps
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: reused for row and stop check
            tip_pos = free_tip_node.GetPos()  # cache: repeated vector access avoided below
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    print(f"Simulation stopped at t={sys.GetChTime():.3f} s")
