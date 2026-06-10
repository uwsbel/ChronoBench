"""Euler-Bernoulli beam demo using an SMC FEA system.

The model builds a short flexible beam with ChBuilderBeamEuler from point A to
point B, constrains hnode1 to a fixed truss with ChLinkMateGeneric, fixes the
last node directly, and applies a downward load to the first node.
The beam is solved with Pardiso MKL and an HHT timestepper, then rendered with
Irrlicht FEA visual shapes showing bending response and node coordinate glyphs.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === named values keep the beam setup and render cadence explicit
TIME_STEP = 0.001
SIM_END = 2.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

BEAM_ELEMENTS = 5
BEAM_A = chrono.ChVector3d(0.0, 0.0, -0.1)
BEAM_B = chrono.ChVector3d(0.2, 0.0, -0.1)
BEAM_UP = chrono.ChVector3d(0.0, 1.0, 0.0)
FORCE_ON_FIRST_NODE = chrono.ChVector3d(0.0, -1.0, 0.0)

BEAM_DIAMETER = 0.006
BEAM_DENSITY = 1000.0
BEAM_YOUNG_MODULUS = 5.0e4
BEAM_POISSON_RATIO = 0.3
BEAM_DAMPING = 0.02


# === System & Solver === FEA beams use SMC, Pardiso MKL, and HHT integration
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.SetSolver(mkl.ChSolverPardisoMKL())

timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetStepControl(False)
sys.SetTimestepper(timestepper)


# === Beam Mesh === Euler-Bernoulli builder creates the requested five-element span
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(BEAM_DIAMETER)
section.SetDensity(BEAM_DENSITY)
section.SetYoungModulus(BEAM_YOUNG_MODULUS)
section.SetShearModulusFromPoisson(BEAM_POISSON_RATIO)
section.SetRayleighDamping(BEAM_DAMPING)

# Euler-Bernoulli beam setup: create the beam from point A to point B with Y as up.
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, BEAM_ELEMENTS, BEAM_A, BEAM_B, BEAM_UP)
beam_node_container = builder.GetLastBeamNodes()  # cache: keep SWIG container alive
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]  # cache: reused below
first_node = beam_nodes[0]  # cache: loaded once for the applied nodal force
hnode1 = beam_nodes[1]  # cache: constrained support node
last_node = beam_nodes[-1]  # cache: fixed endpoint

last_node.SetFixed(True)
first_node.SetForce(FORCE_ON_FIRST_NODE)

# FEA beam: no contact material needed because it is driven by constraints and nodal load only.
sys.Add(mesh)


# === Constraints === node one is fixed to a truss with ChLinkMateGeneric, not SetFixed
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

# hnode1.SetFixed(True) is intentionally not used; the clamp is modeled as a link.
node1_clamp = chrono.ChLinkMateGeneric()
node1_clamp.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())
node1_clamp.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(node1_clamp)


# === FEA Visualization === surface moment colors and node glyphs match beam-demo rendering
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization === Irrlicht is initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.32, 0.18, 0.32), chrono.ChVector3d(0.10, 0.0, -0.10))
vis.AddTypicalLights()
vis.AddGrid(
    0.05,
    0.05,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.1, -0.08, -0.1), chrono.Q_ROTATE_Z_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop === render the FEA beam and step the solver in batches
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # system or visualization I/O failure
    raise RuntimeError(f"simulation I/O failed: {exc}") from exc
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid FEA state
    raise
finally:
    print(f"final_time={sys.GetChTime():.4f}")
