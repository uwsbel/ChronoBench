"""Two chained Euler-Bernoulli beam segments under gravity (PyChrono FEA, SMC system).

Models a flexible cantilever built from two consecutive Euler beam segments using
`fea.ChBuilderBeamEuler`. The first segment is clamped at its root node; the second
segment is appended so that its 'A' node coincides with the LAST node created by the
first segment, and its 'B' point is (0.2, 0.1, -0.1), using a Y-up reference
direction (0, 1, 0). Both segments share a single circular beam section.

System type: ChSystemSMC (required for FEA). Integration uses the HHT implicit
timestepper for beam stiffness. The expected behavior is a slender two-segment beam
sagging/oscillating under gravity about its fixed root, with the free tip deflecting.

This is a pure deformable-FEA scene driven only by gravity and the clamp constraint:
there is no rigid-body collision/contact anywhere, so no collision system and no
contact material are configured (a free FEA beam needs neither).
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants === geometry / physics / integration parameters
time_step = 5e-4            # FEA-stable implicit step
sim_end = 3.0              # seconds of simulated time
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once

# Beam section (slender wooden-like circular rod)
beam_diameter = 0.02       # m
beam_density = 1000.0      # kg/m^3
beam_youngs = 2.0e8        # Pa
beam_shear = beam_youngs * 0.35
beam_damping = 0.02

# Segment 1: clamped root -> intermediate node (the chain hand-off point)
seg1_n_elements = 6
seg1_start = chrono.ChVector3d(0.0, 0.0, 0.0)
seg1_end = chrono.ChVector3d(0.2, 0.0, 0.0)

# Segment 2 (the appended chained segment): starts at segment-1 last node, ends at B
seg2_n_elements = 6
seg2_end = chrono.ChVector3d(0.2, 0.1, -0.1)   # 'B' point for the second beam
up_dir = chrono.ChVector3d(0, 1, 0)            # 'Y' up reference direction

# === System & gravity === SMC system is required for FEA; Y-up world, gravity -Y
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure deformable FEA beam: no rigid collision/contact in the scene, so the
# collision system and any contact material are intentionally omitted.

# MKL direct solver (iterative solvers diverge on FEA stiffness matrices)
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT implicit timestepper for beam stiffness (numerically dissipative, stable
# for the stiff FEA beam equations where explicit/iterative schemes diverge)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# === FEA mesh & beam section === single circular section shared by both segments
keepalive = []   # strong refs: prevent SWIG GC of mesh/builder/section/nodes

mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
keepalive.append(mesh)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(beam_density)
section.SetYoungModulus(beam_youngs)
section.SetShearModulus(beam_shear)
section.SetRayleighDamping(beam_damping)
keepalive.append(section)

# === Beam segments === two chained Euler-Bernoulli beams
builder = fea.ChBuilderBeamEuler()
keepalive.append(builder)

# Segment 1: clamped root cantilever
builder.BuildBeam(mesh, section, seg1_n_elements, seg1_start, seg1_end, up_dir)
seg1_nodes_container = builder.GetLastBeamNodes()                      # keep ref before indexing
seg1_nodes = [seg1_nodes_container[i] for i in range(seg1_nodes_container.size())]
keepalive.append(seg1_nodes_container)
keepalive.extend(seg1_nodes)

# Clamp the root node of segment 1
root_node = seg1_nodes[0]
root_node.SetFixed(True)

# The last node of segment 1 is the 'A' node (starting point) of segment 2
seg2_start_node = seg1_nodes[-1]

# Segment 2: appended so its 'A' node is segment-1's last node, 'B' point is seg2_end
builder.BuildBeam(mesh, section, seg2_n_elements, seg2_start_node, seg2_end, up_dir)
seg2_nodes_container = builder.GetLastBeamNodes()                      # keep ref before indexing
seg2_nodes = [seg2_nodes_container[i] for i in range(seg2_nodes_container.size())]
keepalive.append(seg2_nodes_container)
keepalive.extend(seg2_nodes)

# Free tip of the whole chained beam (end of segment 2)
tip_node = seg2_nodes[-1]

sys.Add(mesh)

# Snapshot baseline tip coordinates as plain scalars (avoid dangling SWIG refs later)
tip0 = tip_node.GetPos()
tip0_x, tip0_y, tip0_z = tip0.x, tip0.y, tip0.z   # cache: baseline tip, computed once

# === FEA visualization === colored deformed mesh + undeformed wireframe overlay
vis_surface = chrono.ChVisualShapeFEA()
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_surface.SetColormapRange(chrono.ChVector2d(0.0, 0.5))
vis_surface.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_surface)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y, so Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Two chained Euler beam segments")
vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.4, 0.25, 0.6), chrono.ChVector3d(0.15, -0.05, -0.05))
vis.AddTypicalLights()
vis.AddGrid(0.05, 0.05, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.3, 0),
                               chrono.QuatFromAngleX(math.pi / 2)),
            chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid in X-Z plane

# === Output setup ===

# === Main loop === render-cadence outer loop; HHT physics in the inner batch
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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
