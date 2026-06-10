"""
Euler-beam buckling simulation using PyChrono FEA (ChSystemSMC).

Models a slender vertical beam (column) fixed at the base, loaded axially with
a compressive force at the tip, demonstrating Euler buckling instability. The
beam is modelled with ChBeamSectionEulerAdvanced elements, solved with Pardiso
MKL and the HHT timestepper. An initial lateral perturbation triggers the
lateral buckled mode. Beam bending moment (Mz) is visualised via ChVisualShapeFEA
colour-mapped on the deformed mesh.

System: ChSystemSMC, Y-up, gravity disabled on FEA mesh.
Bodies: FEA beam mesh (column) clamped at base; tip node receives compressive load.
Expected behaviour: the slender column buckles laterally under the axial load,
showing classic Euler post-buckling deflection.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl

# === Constants ===
# Beam geometry
BEAM_LENGTH   = 1.0          # m — column height
BEAM_DIAMETER = 0.012        # m — circular cross-section diameter
N_ELEMENTS    = 16           # number of Euler beam elements along the column

# Material — steel-like
DENSITY      = 7800.0        # kg/m³
YOUNG_MOD    = 210.0e9       # Pa
SHEAR_MOD_NU = 0.3           # Poisson ratio (used to derive G)
RAYLEIGH_DAMP = 0.0001       # Rayleigh damping beta

# Axial compressive load — slightly above Euler critical load for clamped-free column
# Pcr = pi² E I / (4 L²)
_radius      = BEAM_DIAMETER / 2.0
_I           = math.pi * _radius**4 / 4.0          # second moment of area  # precomputed once
LOAD_FORCE   = 1.2 * (math.pi**2 * YOUNG_MOD * _I / (4.0 * BEAM_LENGTH**2))

# Lateral perturbation to seed buckling
PERTURB_FORCE = 0.001 * LOAD_FORCE   # small horizontal nudge

# Simulation timing
TIME_STEP    = 0.001          # s — stiff beam requires 1 ms
SIM_END      = 3.0            # s
RENDER_FPS   = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# FEA buckling: ChSystemSMC + Pardiso MKL + HHT timestepper (Y-up)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))   # Y-up
# No collision system — pure jointed FEA, no rigid-body contact

# Pardiso MKL solver — required for stiff beam stiffness matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper — canonical-minimal two-call form
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh — Euler beam column ===
# FEA beam: no contact material needed — driven by base constraint + tip load only
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)   # gravity disabled on FEA elements; load applied at tip

sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(BEAM_DIAMETER)
sec.SetDensity(DENSITY)
sec.SetYoungModulus(YOUNG_MOD)
sec.SetShearModulusFromPoisson(SHEAR_MOD_NU)
sec.SetRayleighDamping(RAYLEIGH_DAMP)

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, sec, N_ELEMENTS,
    chrono.ChVector3d(0.0, 0.0, 0.0),          # base
    chrono.ChVector3d(0.0, BEAM_LENGTH, 0.0),  # tip
    chrono.ChVector3d(1.0, 0.0, 0.0),          # lateral reference direction (X)
)

# Keep strong reference to prevent SWIG GC dangling the node container  # cache:
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

# Clamp the base node (fully fixed)
base_node = beam_nodes[0]
base_node.SetFixed(True)

# Apply axial load + lateral perturbation at tip to trigger buckling
tip_node = beam_nodes[-1]
tip_node.SetForce(chrono.ChVector3d(PERTURB_FORCE, -LOAD_FORCE, 0.0))

sys.Add(mesh)

# === FEA visualization shapes ===
# Surface shape — colour-mapped Mz bending moment field
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Glyph shape — node coordinate-system triads
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler Beam Buckling — PyChrono FEA")
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.8, 0.5, 1.2),   # eye
    chrono.ChVector3d(0.0, 0.5, 0.0),   # look-at mid-column
)
vis.AddTypicalLights()

# === Main loop ===
frame = 0

try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(render_every):
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad beam state
    import traceback
    traceback.print_exc()
    raise
