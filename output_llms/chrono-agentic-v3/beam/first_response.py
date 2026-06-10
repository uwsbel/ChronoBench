"""
PyChrono FEA Beam Simulation (ChSystemSMC).

Models a mesh composed of Euler-Bernoulli beam finite elements with nodes having
specific material properties. The beam is anchored at one end (clamped root) and
free at the tip. Gravity drives bending deformation, visualized with a colorscale
of the Z bending moment (Mz) and node-coordinate-system glyphs. The Pardiso MKL
direct solver and HHT time-stepper are used for stiff beam elements.

System: ChSystemSMC, Y-up gravity.
FEA element: ChBeamSectionEulerAdvanced, ChBuilderBeamEuler.
Expected behavior: beam bends / oscillates under gravity; Mz colormap shows stress.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Constants ===
BEAM_LENGTH     = 0.4         # m — beam span
BEAM_DIAM       = 0.01        # m — circular cross-section diameter
BEAM_DENSITY    = 1000.0      # kg/m³
BEAM_E          = 0.01e9      # Pa — Young's modulus (flexible beam)
BEAM_POISSON    = 0.3         # Poisson ratio (for shear modulus derivation)
BEAM_DAMPING    = 0.000       # Rayleigh structural damping
N_ELEMENTS      = 16          # number of Euler beam elements
TIME_STEP       = 1e-3        # s — integration timestep for stiff beams
SIM_END         = 10.0        # s — simulation duration
RENDER_FPS      = 50.0        # frames per second for review capture
# precomputed once
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))


# === System & gravity ===
# FEA beam demo uses ChSystemSMC + Pardiso MKL (stiff element requirement)
# Y-up convention: gravity = (0, -9.81, 0)
# No collision system needed — pure FEA, no rigid-body contact
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# FEA beam: no contact material needed — driven by constraints + gravity only
# (no collision surface required)

# === FEA solver & timestepper ===
sys.SetSolver(mkl.ChSolverPardisoMKL())

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA Mesh & Beam ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# Beam cross-section properties
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAM)
sec.SetDensity(BEAM_DENSITY)
sec.SetYoungModulus(BEAM_E)
sec.SetShearModulusFromPoisson(BEAM_POISSON)
sec.SetRayleighDamping(BEAM_DAMPING)

# Build the Euler beam — aligned along X axis, free to deflect under gravity
up_dir = chrono.ChVector3d(0, 1, 0)  # section Y reference direction
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, N_ELEMENTS,
    chrono.ChVector3d(0.0, 0.0, 0.0),          # root (A)
    chrono.ChVector3d(BEAM_LENGTH, 0.0, 0.0),   # tip (B)
    up_dir,
)

# SWIG GC guard: keep strong reference to node container before indexing
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]

# Fix the root node (cantilever clamped end)
nodes[0].SetFixed(True)

# Tip node reference (for CSV logging)
tip_node = nodes[-1]   # cache: fetched once, reused every step

# Register mesh with the system
sys.Add(mesh)

# === FEA visualization shapes ===
# Shape 1: surface/scalar — bending moment Mz colorfield
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node glyphs — coordinate-system triads at each node
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Beam — Euler-Bernoulli cantilever")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up for FEA scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, 0.2, 0.3), chrono.ChVector3d(0.2, 0.0, 0.0))
vis.AddTypicalLights()

# === CSV logging setup ===

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
