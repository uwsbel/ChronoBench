"""PyChrono FEA beam demonstration.

This script builds a smooth-contact Chrono system containing a finite-element
mesh made from Euler-Bernoulli beam elements and xyz-rotation nodes. The beam is
clamped at one end, loaded at the free end, visualized with Irrlicht using FEA
scalar and glyph shapes, and advanced in a real-time simulation loop.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants ===
# Define beam geometry and solver settings once so the model is easy to inspect.
BEAM_LENGTH = 4.0
BEAM_DIAMETER = 0.10
NUM_ELEMENTS = 16
DENSITY = 1200.0
YOUNG_MODULUS = 2.0e9
POISSON_RATIO = 0.30
RAYLEIGH_DAMPING = 0.020
TIP_FORCE = chrono.ChVector3d(0.0, -500.0, 80.0)
TIME_STEP = 0.001
SIM_END = 1.2
RECORD_FPS = 30.0
RECORD_EVERY = max(1, round(1.0 / (RECORD_FPS * TIME_STEP)))  # precomputed once


# === System & Solver ===
# FEA beam models use ChSystemSMC with a direct solver and HHT integration.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())

timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Beam Mesh ===
# Build a clamped Euler beam and keep strong references to SWIG-owned containers.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(BEAM_DIAMETER)
section.SetDensity(DENSITY)
section.SetYoungModulus(YOUNG_MODULUS)
section.SetShearModulusFromPoisson(POISSON_RATIO)
section.SetRayleighDamping(RAYLEIGH_DAMPING)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh,
    section,
    NUM_ELEMENTS,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(BEAM_LENGTH, 0.0, 0.0),
    chrono.VECT_Z,
)
beam_node_container = builder.GetLastBeamNodes()  # cache: strong SWIG reference
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]
root_node = beam_nodes[0]  # cache: clamped end node
tip_node = beam_nodes[-1]  # cache: loaded end node
root_node.SetFixed(True)
tip_node.SetForce(TIP_FORCE)

# FEA beam: no contact material needed because it is driven by gravity and nodal load only.

sys.Add(mesh)


# === FEA Visualization ===
# Attach scalar and node-glyph visual shapes before initializing Irrlicht.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-800.0, 800.0)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.035)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Irrlicht Visualization ===
# Configure the real-time renderer after mesh setup so FEA shapes are visible.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Beam Elements")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.2, 1.2, 4.0), chrono.ChVector3d(2.0, -0.25, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(2.0, -0.75, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop ===
# Advance the flexible beam while rendering its deformation and nodal frames.

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        tip_pos = tip_node.GetPos()  # cache: reused for review row and validity check
        if not all(abs(v) < 1.0e6 for v in (tip_pos.x, tip_pos.y, tip_pos.z)):
            raise ValueError("beam tip position left the expected finite range")

        tip_node.SetForce(TIP_FORCE)
        sys.DoStepDynamics(TIME_STEP)
        frame += 1
except (RuntimeError, ValueError, OSError) as exc:
    # RuntimeError: solver/render failure; ValueError: invalid state; OSError: capture I/O.
    print(f"simulation failed: {exc}")
    raise


# === Review Outputs ===
# Assemble validation video and plot only when record mode is requested.
