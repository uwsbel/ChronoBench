"""
PyChrono FEA Beam Simulation — ChSystemSMC
==========================================
Models a cantilever beam finite element using Euler-Bernoulli beam elements.
A mesh of beam elements is built with specific section properties (circular
cross-section, aluminum-like Young's modulus). The root node is clamped to
ground via a ChLinkMateGeneric constraint, and gravity acts on the mesh.
The simulation renders with Irrlicht (real-time), showing bending-moment
colour field and node glyph coordinate systems.

System: ChSystemSMC
Solver: Pardiso MKL (direct, required for stiff beams)
Timestepper: HHT (canonical-minimal)
Expected behaviour: beam deflects under gravity; tip oscillates and settles.

FEA beam: no contact material needed — driven by constraints + gravity only.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Physical / geometry constants ===
BEAM_LENGTH = 1.2          # m — beam span
BEAM_DIAMETER = 0.05       # m — circular cross-section diameter
BEAM_DENSITY = 2700.0      # kg/m³ — aluminium
BEAM_YOUNG = 73.0e9        # Pa
BEAM_POISSON = 0.30
BEAM_DAMPING = 0.0005
N_ELEMENTS = 10            # number of Euler-Bernoulli beam elements
TIME_STEP = 1e-3           # s — stiff beam; 0.001 is canonical
SIM_END = 5.0              # s
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemSMC is required for all FEA demos (beam/buckling/cable/rotor)
sys = chrono.ChSystemSMC()
# Y-up world convention for MBS/FEA demos; gravity pulls downward along -Y
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
# No SetCollisionSystemType here — pure FEA jointed scene, no rigid-body contact

# === Solver: Pardiso MKL (direct, required for stiff Euler beams) ===
mkl_solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(mkl_solver)

# === Timestepper: HHT canonical-minimal ===
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh & beam section ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)  # gravity acts on FEA nodes automatically

sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAMETER)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(BEAM_YOUNG)
sec.SetShearModulusFromPoisson(BEAM_POISSON)
sec.SetRayleighDamping(BEAM_DAMPING)

# Build Euler-Bernoulli beams along the X-axis
# up_vect = lateral reference direction perpendicular to beam axis
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, N_ELEMENTS,
    chrono.ChVector3d(0.0, 0.0, 0.0),            # start (root)
    chrono.ChVector3d(BEAM_LENGTH, 0.0, 0.0),     # end (tip)
    chrono.ChVector3d(0.0, 1.0, 0.0),             # lateral reference (Y)
)

# === SWIG GC safety — keep strong ref to node container ===
beam_nodes_container = builder.GetLastBeamNodes()  # cache: container held to prevent GC
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
root_node = beam_nodes[0]
tip_node = beam_nodes[-1]

sys.Add(mesh)

# === Ground truss (fixed body — anchor for the root constraint) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === Root constraint: clamp root node to ground in all 6 DOF ===
root_constraint = chrono.ChLinkMateGeneric()
root_constraint.Initialize(
    root_node, truss, False,
    root_node.Frame(), root_node.Frame()
)
root_constraint.SetConstrainedCoords(True, True, True,   # tx, ty, tz
                                     True, True, True)   # rx, ry, rz
sys.Add(root_constraint)

# === FEA Visualization shapes ===
# Shape 1: surface coloured by bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node glyph coordinate-system triads
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — Irrlicht window ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA — Euler-Bernoulli Beam")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world
vis.Initialize()                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, -0.8, 0.6),              # eye — side + above
    chrono.ChVector3d(0.6, 0.0, 0.0),               # look-at — beam centre
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 30, 30,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.3, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only recording setup ===


# === Main simulation loop ===
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad beam state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
