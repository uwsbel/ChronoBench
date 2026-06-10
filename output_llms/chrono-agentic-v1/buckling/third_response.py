"""
Euler-Bernoulli beam buckling simulation using PyChrono FEA.

Models a slender column clamped at the base and subjected to an increasing
axial compressive load applied at the tip, producing lateral (Euler) buckling.
A second horizontal stabilizer brace beam, connected via constraints at
mid-height, demonstrates multi-member interaction and lateral load transfer
between distinct parts.

The column diameter is chosen so that the applied peak load (300 N) well
exceeds the Euler critical buckling load (~100 N for d=8 mm, L=1.2 m),
ensuring visible lateral deflection. A small lateral perturbation is applied
at t=0 to trigger the post-buckling mode.

System type: ChSystemSMC
Solver: Pardiso MKL (direct, required for stiff beam matrices)
Timestepper: Euler implicit linearized (robust for stiff FEA beams with MKL)
Main bodies: two FEA Euler-Bernoulli beams (vertical column + horizontal brace),
  a fixed ground truss, FEA visualization with Mz bending moment color map.

Expected behavior: the column deflects laterally under axial compressive load
exceeding Euler buckling load; brace constrains mid-height motion; the Mz
bending moment field is visible as a color gradient on the beam surface.
"""

# === Imports ===
import os
import math
import csv

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Simulation parameters ===
# FEA beam column geometry — slender column (d small to lower Pcr)
# Euler buckling: Pcr = pi^2 * E * I / L^2
# With d=0.008 m, L=1.2 m, E=73e9: Pcr ~ 100 N → load of 300 N is 3x Pcr
BEAM_LENGTH_COL = 1.2        # m — vertical column length
BEAM_DIAM_COL   = 0.008      # m — circular cross-section diameter (slender: Pcr~100 N)
BEAM_DENS       = 2700.0     # kg/m³ — aluminium density
BEAM_E          = 73.0e9     # Pa — aluminium Young's modulus
BEAM_NU         = 0.3        # Poisson's ratio
N_ELEM_COL      = 16         # number of Euler elements in column

# Horizontal stabilizer brace geometry
BEAM_LENGTH_BRACE = 0.5      # m — horizontal brace length
BEAM_DIAM_BRACE   = 0.006    # m — brace diameter (thinner than column)
N_ELEM_BRACE      = 6        # elements in brace

# Loading: linearly increasing axial compressive force at tip (N)
# 300 N >> Pcr (100 N) → large post-buckling deflection
LOAD_MAX         = 300.0     # N — peak compressive load (3x Euler critical load)
LOAD_RAMP_TIME   = 3.0       # s — time to ramp from 0 to LOAD_MAX
PERTURB_FORCE    = 5.0       # N — small lateral perturbation to trigger buckling mode

# Time integration
TIME_STEP = 1e-3             # s — timestep for FEA beams
SIM_END   = 5.0              # s — simulation duration
RENDER_FPS = 50.0            # frames/s for review video

# Precomputed constants  # precomputed once
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame

# === System and solver ===
# ChSystemSMC is required for all FEA ground-truth demos
sys = chrono.ChSystemSMC()
# FEA beam: no contact material needed — driven by constraints + gravity + applied load only
# No collision system needed for pure jointed FEA (no rigid-body contact)

# Pardiso MKL direct solver — required for stiff Euler beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# Euler implicit linearized timestepper — robust for stiff FEA beams with Pardiso MKL
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Y-up world convention for FEA (gravity along -Y)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

# Keep strong references to prevent SWIG GC on shared_ptrs
_refs = {}

# === FEA Mesh ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)  # apply gravity to FEA nodes automatically
sys.Add(mesh)
_refs["mesh"] = mesh

# === Ground truss (fixed body) ===
# Clamped base — a fixed rigid body to which beam root nodes are constrained
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.Add(truss)
_refs["truss"] = truss

# === Euler-Bernoulli column beam section ===
sec_col = fea.ChBeamSectionEulerAdvanced()
sec_col.SetAsCircularSection(BEAM_DIAM_COL)
sec_col.SetDensity(BEAM_DENS)
sec_col.SetYoungModulus(BEAM_E)
sec_col.SetShearModulusFromPoisson(BEAM_NU)
sec_col.SetRayleighDamping(0.005)  # light damping to avoid oscillation
_refs["sec_col"] = sec_col

# Build the vertical column from base (0,0,0) to tip (0,L,0)
# Y-up: column extends along +Y axis; lateral buckling deflection in X direction
builder_col = fea.ChBuilderBeamEuler()
builder_col.BuildBeam(
    mesh,
    sec_col,
    N_ELEM_COL,
    chrono.ChVector3d(0.0, 0.0, 0.0),               # base (root)
    chrono.ChVector3d(0.0, BEAM_LENGTH_COL, 0.0),    # tip
    chrono.ChVector3d(1.0, 0.0, 0.0),                # section Y direction (lateral)
)
_refs["builder_col"] = builder_col

# Store node references before SWIG GC collects the temporary container
col_nodes_container = builder_col.GetLastBeamNodes()
col_nodes = [col_nodes_container[i] for i in range(col_nodes_container.size())]
_refs["col_nodes_container"] = col_nodes_container

# Clamp the root node of the column to the fixed truss (all 6 DOF fixed)
root_node_col = col_nodes[0]
constr_root_col = chrono.ChLinkMateGeneric()
constr_root_col.Initialize(
    root_node_col, truss, False, root_node_col.Frame(), root_node_col.Frame()
)
constr_root_col.SetConstrainedCoords(True, True, True, True, True, True)
constr_root_col.SetName("clamp_col_root")
sys.Add(constr_root_col)
_refs["constr_root_col"] = constr_root_col

# Tip node of the column — receives axial compressive load
tip_node_col = col_nodes[-1]

# Apply a small lateral perturbation at tip to trigger the buckling mode (avoid singularity)
# This small force is kept constant to seed the lateral displacement direction
tip_node_col.SetForce(chrono.ChVector3d(PERTURB_FORCE, 0.0, 0.0))

# === Horizontal stabilizer brace section ===
sec_brace = fea.ChBeamSectionEulerAdvanced()
sec_brace.SetAsCircularSection(BEAM_DIAM_BRACE)
sec_brace.SetDensity(BEAM_DENS)
sec_brace.SetYoungModulus(BEAM_E)
sec_brace.SetShearModulusFromPoisson(BEAM_NU)
sec_brace.SetRayleighDamping(0.005)
_refs["sec_brace"] = sec_brace

# Build brace at mid-height of column, extending horizontally in +X direction
BRACE_Y = BEAM_LENGTH_COL * 0.5  # precomputed once — brace attachment height (0.6 m)
builder_brace = fea.ChBuilderBeamEuler()
builder_brace.BuildBeam(
    mesh,
    sec_brace,
    N_ELEM_BRACE,
    chrono.ChVector3d(0.0, BRACE_Y, 0.0),                   # inner end (at column)
    chrono.ChVector3d(BEAM_LENGTH_BRACE, BRACE_Y, 0.0),     # outer end (at wall)
    chrono.ChVector3d(0.0, 1.0, 0.0),                       # section Y direction
)
_refs["builder_brace"] = builder_brace

# Store brace nodes before GC
brace_nodes_container = builder_brace.GetLastBeamNodes()
brace_nodes = [brace_nodes_container[i] for i in range(brace_nodes_container.size())]
_refs["brace_nodes_container"] = brace_nodes_container

# === Constraints between different parts (column ↔ brace ↔ ground) ===
# Connect brace inner end to the column's mid-height node
# Find the column node nearest to BRACE_Y to connect the two distinct beams
mid_col_node = min(
    col_nodes,
    key=lambda n: abs(n.GetPos().y - BRACE_Y)
)
brace_inner = brace_nodes[0]
constr_brace_col = chrono.ChLinkMateGeneric()
constr_brace_col.Initialize(
    brace_inner, mid_col_node, False, brace_inner.Frame(), brace_inner.Frame()
)
constr_brace_col.SetConstrainedCoords(True, True, True, True, True, True)
constr_brace_col.SetName("joint_brace_col_midheight")
sys.Add(constr_brace_col)
_refs["constr_brace_col"] = constr_brace_col

# Pin outer end of brace to fixed truss wall (pin: TX, TY, TZ fixed; rotation free)
brace_outer = brace_nodes[-1]
constr_brace_wall = chrono.ChLinkMateGeneric()
constr_brace_wall.Initialize(
    brace_outer, truss, False, brace_outer.Frame(), brace_outer.Frame()
)
constr_brace_wall.SetConstrainedCoords(True, True, True, False, False, False)
constr_brace_wall.SetName("pin_brace_wall")
sys.Add(constr_brace_wall)
_refs["constr_brace_wall"] = constr_brace_wall

# === FEA Visualization ===
# Shape 1: beam surface colored by bending moment Mz (load-bearing physics visualization)
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColormapRange(-20.0, 20.0)  # N·m range appropriate for slender column
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)
_refs["vis_beam"] = vis_beam

# Shape 2: node glyph markers (dots at each node showing deformed positions)
vis_glyph = chrono.ChVisualShapeFEA()
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.01)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)
_refs["vis_glyph"] = vis_glyph

# === Visualization window (Irrlicht) ===
# Full Irrlicht block: window + Initialize() FIRST + logo + skybox + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler Buckling — Multi-beam FEA (PyChrono 9.0.0)")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up for FEA world convention
vis.Initialize()                                    # Initialize FIRST (Irrlicht requires this)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(2.5, 0.8, 0.5),   # eye position — side view showing full column
    chrono.ChVector3d(0.0, 0.6, 0.0),   # look-at target — mid-height of column
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

frame = 0  # consecutive frame counter for review video

# === Main simulation loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # Ramp the axial compressive load at column tip (downward, -Y direction)
            ramp = min(1.0, t / LOAD_RAMP_TIME)   # ramp fraction
            current_load = LOAD_MAX * ramp
            # Apply combined axial load + small persistent lateral perturbation to seed buckling
            tip_node_col.SetForce(chrono.ChVector3d(PERTURB_FORCE, -current_load, 0.0))


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state from FEA
    import traceback
    traceback.print_exc()
    raise

finally:
    pass  # scored-core finally block
