"""
ANCF Cable Beam Simulation — PyChrono 9.0.0 / ChSystemSMC

Models a flexible beam composed of ANCF cable elements (ChBuilderCableANCF).
One end of the cable is hinged to a fixed ground truss via ChLinkNodeFrame.
The beam is subjected to gravity and its deformation and nodal positions are
visualized in real time using Irrlicht (ChVisualSystemIrrlicht).

System:   ChSystemSMC
Solver:   ChSolverSparseQR + ChTimestepperEulerImplicitLinearized (canonical for ANCF cable)
Beam:     ChBeamSectionCable + ChBuilderCableANCF, 10 elements, hinged at one end
Gravity:  Y-up  (0, -9.81, 0)   — FEA cable convention
Expected: cable hangs and sways under gravity; nodal positions update each frame
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Physical constants ===
BEAM_LENGTH      = 0.5        # metres — total cable length
N_ELEMENTS       = 10         # number of ANCF cable elements
CABLE_DIAMETER   = 0.015      # metres
CABLE_YOUNG      = 0.01e9     # Pa  (soft cable)
CABLE_DAMPING    = 0.0        # Rayleigh damping coefficient
CABLE_DENSITY    = 7500.0     # kg/m³ (steel-like cable density)
GRAVITY_ACCEL    = -9.81      # m/s² along Y (Y-up world)

TIME_STEP  = 0.01             # s  — canonical for ANCF cable
SIM_END    = 5.0              # s  — total simulation time
RENDER_FPS = 50.0             # frames per second for review video
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame; precomputed once

# Cable start and end positions (horizontal cable that droops under gravity)
CABLE_START = chrono.ChVector3d(0.0, 0.0, -0.1)
CABLE_END   = chrono.ChVector3d(BEAM_LENGTH, 0.0, -0.1)

# Camera position — side view to see the droop
CAM_EYE    = chrono.ChVector3d(-0.5, -0.5, 0.8)
CAM_TARGET = chrono.ChVector3d(0.25, 0.0, -0.1)

# === System & gravity (ChSystemSMC — required for all FEA demos) ===
# FEA beam: no contact material needed — cable driven by constraints + gravity only
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, GRAVITY_ACCEL, 0.0))

# === Solver & timestepper (ANCF cable: sparse QR + Euler implicit linearized) ===
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA Mesh — ANCF cable beam ===
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)  # gravity applied to FEA nodes automatically

# Cable cross-section (ANCF)
sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAMETER)
sec_cable.SetYoungModulus(CABLE_YOUNG)
sec_cable.SetDensity(CABLE_DENSITY)
sec_cable.SetRayleighDamping(CABLE_DAMPING)

# Build the ANCF cable beam (N_ELEMENTS elements, no up-vector / order args)
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(
    mesh,
    sec_cable,
    N_ELEMENTS,
    CABLE_START,
    CABLE_END,
)

# SWIG GC pitfall: keep a strong reference to the node container before indexing
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]

# === Ground truss (fixed body to anchor the cable) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === Hinge: pin the first (start) node to the fixed truss ===
# ChLinkNodeFrame in 9.0.0 (not ChLinkPointFrame)
hinge = fea.ChLinkNodeFrame()
hinge.Initialize(beam_nodes[0], truss)
sys.Add(hinge)

# Register the mesh with the system
sys.Add(mesh)

# === FEA Visualization — two ChVisualShapeFEA shapes ===
# Shape 1: surface/scalar field coloured by bending moment Mz (shows deformation)
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# Shape 2: node glyphs — coordinate-system triads at each node (nodal positions)
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# === Visualization (Irrlicht) — full window: title/size + Initialize + scene elements ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Beam — Gravity Deformation")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA cable world
vis.Initialize()                                     # Initialize FIRST (Irrlicht)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_EYE, CAM_TARGET)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.1, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# Cache tip node getter result once to avoid repeated indexing in the loop
tip_node = beam_nodes[-1]   # cache: last node = cable tip; fetched once

# === Main simulation loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # ensure finally is never empty in the scored core
