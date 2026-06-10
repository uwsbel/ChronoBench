"""
Buckling simulation using PyChrono FEA.

Models a vertical column under compressive axial load that undergoes Euler
buckling (lateral deflection into the first mode shape). Uses IGA Cosserat
beam elements for large-rotation accuracy. The beam is fixed at the base;
a compressive load slightly above the Euler critical load and a tiny lateral
perturbation seed the characteristic first-mode buckle shape.

System type: ChSystemSMC
Solver: Pardiso MKL (stiff beam matrices)
Timestepper: HHT (canonical-minimal: SetStepControl(False))
Expected behavior: initially straight vertical beam deflects laterally into
a smooth half-sine buckling mode under the compressive axial force.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Constants ===
# Beam geometry
BEAM_L   = 1.2     # m — column height
BEAM_R   = 0.01    # m — circular cross-section radius

# Steel material
DENSITY  = 7800.0  # kg/m³
E_YOUNG  = 210e9   # Pa
POISSON  = 0.3

# Cross-section quantities (precomputed once)
AREA = math.pi * BEAM_R**2
IYY  = math.pi * BEAM_R**4 / 4.0
IZZ  = math.pi * BEAM_R**4 / 4.0
J    = math.pi * BEAM_R**4 / 2.0

# Euler critical load Pcr = pi^2 * E * I / L^2 (precomputed once)
P_CR = (math.pi**2) * E_YOUNG * IYY / (BEAM_L**2)   # ≈ 449 N

# Load at 1.05 x Pcr — just enough to buckle slowly without dynamic explosion
F_COMPRESS = -1.05 * P_CR      # N (negative = downward in Y-up world)
F_PERTURB  = 0.5               # N lateral seed force (X direction)

# Rayleigh damping coefficient to reduce oscillation amplitude
RAYLEIGH_DAMP = 0.01

# Simulation
TIME_STEP  = 2e-4     # s — fine step for HHT on stiff beam
SIM_END    = 2.0      # s
RENDER_FPS = 50.0     # Hz
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

N_SPANS   = 16
IGA_ORDER = 3

# === System & solver (FEA uses ChSystemSMC + Pardiso MKL) ===
# FEA beam: no contact material needed — driven by constraints + load only
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))   # Y-up

solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# === FEA mesh — IGA Cosserat beam (buckling column) ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(DENSITY)
minertia.SetArea(AREA)
minertia.SetIyy(IYY)
minertia.SetIzz(IZZ)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E_YOUNG)
melasticity.SetShearModulusFromPoisson(POISSON)
melasticity.SetIyy(IYY)
melasticity.SetIzz(IZZ)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(BEAM_R)

# Build vertical IGA beam: base at origin (0,0,0), tip at (0,BEAM_L,0)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    N_SPANS,
    chrono.ChVector3d(0, 0, 0),            # base
    chrono.ChVector3d(0, BEAM_L, 0),       # tip
    chrono.VECT_X,                         # lateral direction reference
    IGA_ORDER,
)

# CRITICAL: keep strong reference to prevent SWIG GC segfault on node access
beam_nodes_cnt = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_cnt[i] for i in range(beam_nodes_cnt.size())]  # cache

node_base = beam_nodes[0]   # cache: fixed base node
node_tip  = beam_nodes[-1]  # cache: loaded tip node

# Fix base all 6 DOF
node_base.SetFixed(True)

# Compressive load + tiny lateral perturbation at tip
node_tip.SetForce(chrono.ChVector3d(F_PERTURB, F_COMPRESS, 0))

sys.Add(mesh)

# === FEA Visualization (attached before vis.Initialize to register with Irrlicht) ===
# Shape 1: colored beam surface showing bending moment Mz
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-200.0, 200.0)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node glyphs showing local coordinate frames
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization — full Irrlicht scene: window + sky + camera + lights ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Buckling Demo")
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.8, 0.6, 1.5),              # eye: angled side view
    chrono.ChVector3d(0.0, 0.6, 0.0),              # target: beam midpoint
)
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t       = sys.GetChTime()
            tip_pos = node_tip.GetPos()    # cache: per-step tip position
            tip_vel = node_tip.GetPosDt()  # cache: per-step tip velocity
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
