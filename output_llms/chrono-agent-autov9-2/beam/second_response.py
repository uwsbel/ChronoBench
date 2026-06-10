"""Euler-Bernoulli FEA beam demo (PyChrono 9.0.1, Irrlicht).

Models flexible Euler-Bernoulli beams with the ChSystemSMC physical system and
the MKL Pardiso direct solver (required for FEA stiffness matrices). Two beam
sections are built:

  * A short reference beam whose first node (hnode1) is held in place by a
    ChLinkMateGeneric constraint to a fixed truss body (rather than by flagging
    the node fixed directly).
  * A beam built with the ChBuilderBeamEuler helper spanning (0, 0, -0.1) to
    (0.2, 0, -0.1) with a 'Y' up reference and 5 elements. Its LAST node is
    clamped fixed and a downward force (0, -1, 0) N is applied to its FIRST node.

Expected behavior: the cantilevered builder beam deflects downward under the
applied tip force and its own gravity, reaching a quasi-static sag; the
constrained reference beam hangs from its fixed node. No rigid-body contact
occurs, so no collision system or contact material is needed.
"""

import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing (no bare literals downstream)
time_step = 5e-4                       # FEA stability step
sim_end = 2.0                          # seconds of simulated time
render_fps = 50.0
gravity = chrono.ChVector3d(0, -9.81, 0)   # gravity along -Y (beam 'Y' up convention)

beam_E = 0.02e10                       # Young's modulus [Pa]
beam_density = 1000.0                  # density [kg/m^3]
beam_diameter = 0.01                   # circular cross-section diameter [m]
beam_damping = 0.000                   # Rayleigh beta damping

builder_start = chrono.ChVector3d(0, 0, -0.1)   # builder beam point A
builder_end = chrono.ChVector3d(0.2, 0, -0.1)   # builder beam point B
builder_up = chrono.ChVector3d(0, 1, 0)         # 'Y' up reference direction
builder_n_elements = 5                          # number of beam elements
tip_force = chrono.ChVector3d(0, -1, 0)         # downward force on first node [N]

# === System & gravity === SMC system required for FEA; no contact in this scene
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(gravity)
# Pure FEA beams driven by gravity + constraints + applied force only:
# no contact material and no collision system are needed (no rigid-body collision).

# keepalive: hold strong Python references so SWIG temporaries are not GC'd
keepalive = {}

# === FEA mesh & section === Euler-Bernoulli beam section properties
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
keepalive["mesh"] = mesh

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetYoungModulus(beam_E)
section.SetShearModulus(beam_E * 0.35)
section.SetDensity(beam_density)
section.SetRayleighDamping(beam_damping)
keepalive["section"] = section

# === Reference beam & constraint === fix node 1 with ChLinkMateGeneric (not SetFixed)
# A fixed "truss" body provides the anchor frame the constraint pins node 1 to.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)
keepalive["truss"] = truss

# Manually create the first reference node; previously this would have been held
# with hnode1.SetFixed(True) — instead we constrain it to the fixed truss.
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
keepalive["hnode1"] = hnode1
keepalive["hnode2"] = hnode2

ref_builder = fea.ChBuilderBeamEuler()
ref_builder.BuildBeam(mesh, section, 1, hnode1, hnode2, builder_up)
keepalive["ref_builder"] = ref_builder

# hnode1.SetFixed(True)  # replaced: node 1 is fixed via the ChLinkMateGeneric below
constraint1 = chrono.ChLinkMateGeneric()
constraint1.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())
constraint1.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF
constraint1.SetName("fix_node1")
sys.Add(constraint1)
keepalive["constraint1"] = constraint1

# === Builder beam === ChBuilderBeamEuler section A->B, 'Y' up, 5 elements
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, builder_n_elements, builder_start, builder_end, builder_up)
keepalive["builder"] = builder

# Strong references to the builder nodes before indexing (SWIG GC guard)
beam_nodes = builder.GetLastBeamNodes()
keepalive["beam_nodes"] = beam_nodes
first_node = beam_nodes.front()        # cache: builder beam first node, reused below
last_node = beam_nodes.back()          # cache: builder beam last node, reused below

# Fix the last node of the builder beam; apply the tip force to the first node.
last_node.SetFixed(True)
first_node.SetForce(tip_force)

sys.Add(mesh)

# === FEA visualization === colored deformation + undeformed wireframe overlay
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.5))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA()
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(vis_nodes)

# === Solver & timestepper === MKL Pardiso direct solver + HHT integrator for beams
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)
keepalive["solver"] = solver

sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)   # implicit integrator for FEA beams

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y here
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli FEA Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.4), chrono.ChVector3d(0.1, -0.05, -0.05))
vis.AddTypicalLights()
vis.AddGrid(0.05, 0.05, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0.1, -0.2, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid

# === Main loop === render-cadence outer loop; physics advanced in inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Snapshot the first node's reference position as scalars (GetPos returns a LIVE
# aliased reference, so copy the components rather than storing the vector).
p0 = first_node.GetPos()
tip_x0, tip_y0, tip_z0 = p0.x, p0.y, p0.z   # cache: baseline reference, scalars only


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
finally:
    pass

# === Post-processing === assemble review video + plot, then clean frames
