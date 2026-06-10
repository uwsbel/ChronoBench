"""
Euler-Bernoulli Beam FEA Simulation — PyChrono 9.0.x / Irrlicht

System type : ChSystemSMC
FEA mesh    : ChMesh with Euler-Bernoulli beam elements built via ChBuilderBeamEuler
              spanning from (0, 0, -0.1) to (0.2, 0, -0.1) with 5 elements.
Constraints : The last node of the builder beam is fixed (SetFixed True); node 1
              of an earlier beam section is constrained via ChLinkMateGeneric
              instead of direct SetFixed.
Force       : (0, -1, 0) N applied to the first node of the builder beam.
Solver      : MKL Pardiso (direct solver for stiff beam matrices).
Timestepper : HHT (canonical-minimal: SetStepControl False).
Expected    : The beam deflects under the applied downward force; the fixed end
              holds; the free end displaces in the -Y direction.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Constants ===
time_step = 1e-3          # s — stiff beam timestep
sim_end   = 5.0           # s
render_fps  = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Beam geometry and material
BEAM_START_A = chrono.ChVector3d(0.0,  0.0, -0.1)
BEAM_END_B   = chrono.ChVector3d(0.2,  0.0, -0.1)
BEAM_UP_Y    = chrono.VECT_Y                         # section Y direction
N_ELEM       = 5

DIAMETER     = 0.012        # m  circular cross-section
DENSITY      = 2700.0       # kg/m³  aluminium
E_YOUNG      = 73e9         # Pa
POISSON_NU   = 0.3
RAYLEIGH_D   = 0.000

APPLIED_FORCE = chrono.ChVector3d(0.0, -1.0, 0.0)  # N

# === System & gravity ===
# FEA truth uses ChSystemSMC + direct solver; Y-up gravity
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# FEA beam: no collision surface needed — driven by constraints + gravity + applied force only

# === Solver & timestepper ===
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# --- Euler-Bernoulli section ---
sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(DIAMETER)
sec.SetDensity(DENSITY)
sec.SetYoungModulus(E_YOUNG)
sec.SetShearModulusFromPoisson(POISSON_NU)
sec.SetRayleighDamping(RAYLEIGH_D)

# === Beam — ChBuilderBeamEuler (spans A → B with 5 elements) ===
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, sec, N_ELEM, BEAM_START_A, BEAM_END_B, BEAM_UP_Y)

# Keep a strong reference to the node container to avoid SWIG GC dangling pointer
beam_nodes = builder.GetLastBeamNodes()  # cache: keep reference before indexing

# Fix the LAST node (the far end) directly
beam_nodes.back().SetFixed(True)

# Apply downward force to the FIRST node (free end)
beam_nodes.front().SetForce(APPLIED_FORCE)

# --- Fixed truss for ChLinkMateGeneric constraint on node 1 ---
# Replace direct SetFixed on hnode1 with a ChLinkMateGeneric that clamps
# node index 1 of the builder beam to ground (all 6 DOF constrained).
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

node1 = beam_nodes[1]  # second node (index 1) — constrained via link
constr = chrono.ChLinkMateGeneric()
constr.Initialize(node1, truss, False, node1.Frame(), node1.Frame())
constr.SetConstrainedCoords(True, True, True,   # tx, ty, tz
                            True, True, True)   # rx, ry, rz
sys.Add(constr)

# Register the mesh with the system
sys.Add(mesh)

# === FEA visualization shapes ===
# Shape 1 — surface/scalar field: bending moment Mz (colour-mapped)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2 — node glyph: coordinate-system triads at each node
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht block (Initialize FIRST, scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam FEA — ChBuilderBeamEuler + ChLinkMateGeneric")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                                          # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.3), chrono.ChVector3d(0.1, 0.0, -0.1))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
frame = 0
tip_node = beam_nodes.front()  # cache: free end — the node with the applied force

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # review-only CSV writers closed in the review-only block below
