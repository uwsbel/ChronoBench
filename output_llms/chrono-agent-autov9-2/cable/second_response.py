"""ANCF flexible cable hanging under gravity, pinned at one end and tip-loaded.

Models a single flexible cable built from ANCF beam elements (fea.ChBuilderCableANCF)
in a ChSystemSMC. The cable's far ("front") node is pulled by a constant downward
force, while its other end is pinned to a fixed truss body through a node-frame link
(fea.ChLinkNodeFrame). The cable swings/sags under gravity plus the applied tip force
and settles into a catenary-like equilibrium.

System type : NSC-free deformable FEA (ChSystemSMC, no contact/collision).
Main bodies : one fixed truss body (anchor) + one ChMesh of ANCF cable elements.
Solver      : iterative MINRES (diagonal preconditioner + warm start).
Expected    : the pinned cable bends and sags smoothly, the loaded front node droops
              the most, and the whole structure reaches a stable hanging shape.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / material / solver constants (no bare literals downstream)
time_step = 1e-3                 # implicit step, small for ANCF stability
sim_end = 5.0                    # seconds of simulated hanging dynamics
render_fps = 50.0                # review render cadence

cable_length = 1.0               # m, total cable span
cable_diameter = 0.015           # m, circular cross-section
n_elements = 10                  # ANCF beam elements along the cable
cable_E = 0.01e9                 # Pa, Young's modulus (compliant cable)
cable_density = 1000.0           # kg/m^3
rayleigh_damping = 0.0001        # cable section Rayleigh damping

# Anchor at +x end, cable runs toward the origin along -x.
anchor_pos = chrono.ChVector3d(0, 0, 0)
cable_far_end = chrono.ChVector3d(-cable_length, 0, 0)

applied_tip_force = chrono.ChVector3d(0, -0.7, 0)   # constant force on the front node

solver_max_iters = 200
solver_tolerance = 1e-10

# === System & gravity === deformable FEA system; no contact -> no collision system
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Solver === iterative MINRES with preconditioner + warm start
solver = chrono.ChSolverMINRES()
print("Using MINRES solver")
solver.SetMaxIterations(solver_max_iters)
solver.SetTolerance(solver_tolerance)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

# === Anchor body === fixed truss the cable end is pinned to (no contact material needed)
anchor = chrono.ChBody()
anchor.SetFixed(True)
anchor.SetPos(anchor_pos)
sys.Add(anchor)

# === Cable mesh === ANCF cable elements built with ChBuilderCableANCF
# FEA cable: no contact material/collision surface needed — driven by gravity,
# the pin constraint, and the applied tip force only.
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionCable()
section.SetDiameter(cable_diameter)
section.SetYoungModulus(cable_E)
section.SetDensity(cable_density)
section.SetRayleighDamping(rayleigh_damping)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, section, n_elements, anchor_pos, cable_far_end)

# KEEPALIVE: hold strong refs so SWIG does not GC the node container / nodes.
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
keepalive = [mesh, section, builder, beam_nodes, nodes]   # cache: prevent premature GC

# Pin the anchor-side node (first) to the fixed body via a node-frame link.
pin = fea.ChLinkNodeFrame()
pin.Initialize(nodes[0], anchor)
sys.Add(pin)

# Apply the constant tip force on the front (far-end) node.
front_node = nodes[-1]            # cache: resolved once, reused every step
front_node.SetForce(applied_tip_force)

sys.Add(mesh)

# === FEA visualization === color by speed + undeformed wireframe overlay
vis_speed = chrono.ChVisualShapeFEA()
vis_speed.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_speed.SetColormapRange(chrono.ChVector2d(0.0, 2.0))
vis_speed.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_speed)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

# === Visualization === full Irrlicht scene (Initialize first, then scene elements)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable - pinned, tip-loaded")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-0.5, -0.4, 1.6), chrono.ChVector3d(-0.5, -0.4, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -1.2, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid below the cable

# === Main loop === render at cadence, advance physics in an inner batch, log CSV
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + plot, then drop frame PNGs
