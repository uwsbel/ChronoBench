"""ANCF cable simulation in a Y-up SMC system.

The model builds a flexible cable from ANCF beam elements, pins the rear node to
a fixed truss, and applies a downward force of 0.7 N to the front node. The cable
section uses Rayleigh damping 0.0001, and the system runs with a configured
MINRES solver so the cable bends smoothly under the applied load.
"""

import traceback

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants ===
# Geometry, force, and integration values are defined once for source-visible reuse.
TIME_STEP = 0.01
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CABLE_ELEMENTS = 10
CABLE_DIAMETER = 0.015
CABLE_YOUNG_MODULUS = 0.01e9
CABLE_DAMPING = 0.0001
CABLE_DENSITY = 1000.0
CABLE_START = chrono.ChVector3d(0.0, 0.0, -0.1)
CABLE_END = chrono.ChVector3d(0.5, 0.0, -0.1)
FRONT_FORCE = chrono.ChVector3d(0.0, -0.7, 0.0)


# === System & solver ===
# SMC is the FEA cable truth family; no collision system is needed for a pinned cable.
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(timestepper)


# === Cable mesh ===
# ANCF cable section and builder create the deformable span from scratch.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionCable()
section.SetDiameter(CABLE_DIAMETER)
section.SetYoungModulus(CABLE_YOUNG_MODULUS)
section.SetRayleighDamping(CABLE_DAMPING)
section.SetDensity(CABLE_DENSITY)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, section, CABLE_ELEMENTS, CABLE_START, CABLE_END)

beam_nodes = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]  # cache: reused in constraints
rear_node = nodes[0]  # cache: fixed end node
front_node = nodes[-1]  # cache: loaded end node
front_node.SetForce(FRONT_FORCE)

sys.Add(mesh)


# === Constraint ===
# Pin the rear cable node to a fixed truss; the cable has no contact material by design.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

rear_hinge = fea.ChLinkNodeFrame()
rear_hinge.Initialize(rear_node, truss)
sys.Add(rear_hinge)


# === FEA visualization ===
# Two FEA shapes show cable deformation and node positions in Irrlicht.
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
vis_surface.SetColorscaleMinMax(0.0, 0.12)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)


# === Visualization ===
# Irrlicht window is initialized before adding camera, sky, and lights.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF cable with MINRES solver")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.8, 0.25, 0.45), chrono.ChVector3d(0.25, -0.05, -0.1))
vis.AddTypicalLights()


# === Main loop ===
# The cable advances in a direct real-time loop with rendering once per frame.


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # device or output errors
    traceback.print_exc()
    raise
finally:
    pass
