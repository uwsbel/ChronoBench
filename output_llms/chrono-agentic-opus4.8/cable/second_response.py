"""ANCF flexible cable simulation (PyChrono 9.0.0, Irrlicht).

Models a single flexible cable built from ANCF cable beam elements suspended in
a Y-up world. One end node is hinged to a fixed truss; a downward point force is
applied to the free front node so the cable swings/sags under gravity and the
applied load. System type: SMC. The cable section uses Rayleigh damping for a
smoothly damped dynamic response. The linear system is solved with an iterative
MINRES solver (diagonal preconditioner + warm start) and advanced with the
Euler-implicit-linearized timestepper suited to ANCF elements.

Expected behavior: the cable bends and settles under gravity plus the applied
front-node load, with stable, smooth dynamics (no blow-up, no NaN).
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing
TIME_STEP = 0.01            # ANCF cable timestep
SIM_END = 5.0              # seconds of simulated dynamics
RENDER_FPS = 50.0
CABLE_DIAMETER = 0.015     # m, circular cable cross-section
CABLE_E = 0.01e9           # Pa, Young's modulus (flexible cable)
CABLE_RAYLEIGH = 0.0001    # Rayleigh (beta) damping for the cable section
N_ELEMENTS = 10            # number of ANCF cable elements
CABLE_A = chrono.ChVector3d(0, 0, -0.1)    # start node position
CABLE_B = chrono.ChVector3d(0.5, 0, -0.1)  # end node position
FRONT_FORCE = chrono.ChVector3d(0, -0.7, 0)  # N, applied to the free front node

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity === SMC system, Y-up world (FEA convention)
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver & timestepper === iterative MINRES + Euler-implicit-linearized (ANCF)
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
print("Using MINRES solver")
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# === FEA mesh & cable === ANCF cable beam with circular section + Rayleigh damping
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

sec_cable = fea.ChBeamSectionCable()
sec_cable.SetDiameter(CABLE_DIAMETER)
sec_cable.SetYoungModulus(CABLE_E)
sec_cable.SetRayleighDamping(CABLE_RAYLEIGH)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, sec_cable, N_ELEMENTS, CABLE_A, CABLE_B)

# Keep strong references to the SWIG node container before indexing (GC guard).
beam_nodes = builder.GetLastBeamNodes()
front_node = beam_nodes.front()   # cache: free front node, force applied here
back_node = beam_nodes.back()     # cache: end node, hinged to the fixed truss

# Apply the downward load on the free front node.
front_node.SetForce(FRONT_FORCE)

sys.Add(mesh)

# === Constraints === pin the back end node to a fixed truss (3 translational DOF)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(back_node, truss)
sys.Add(hinge)

# === FEA visualization === surface field + node glyphs on the cable mesh
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

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Flexible Cable")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.2, -1.0), chrono.ChVector3d(0.25, -0.2, -0.1))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0.25, -0.4, -0.1), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop; physics advanced in batches

os.makedirs("cam", exist_ok=True)   # guard against missing output dir
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
