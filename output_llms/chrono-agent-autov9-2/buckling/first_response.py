"""Beam buckling simulation with finite-element (FEA) Euler-Bernoulli beam elements.

Models a slender vertical column built from Euler beam finite elements in a
clamped-pinned configuration: the base node is fully clamped, the tip node is pinned
laterally (free to slide axially) to a fixed ground anchor, and a vertical
compressive load is applied at the tip and ramped up over time via a custom motor
function. As the ramped axial load crosses the column's Euler critical buckling load,
the initially straight column loses lateral stability and bows out at mid-span
(buckling).

- System type: ChSystemSMC (required for FEA; deformable beam dynamics).
- Main bodies: a single fea.ChMesh holding the beam elements + their nodes.
- Constraints: base node clamped via SetFixed; ChLinkMateGeneric pins the tip node
  laterally to a fixed ground anchor (a constraint between distinct parts of the
  system: the deformable mesh node and the rigid ground body).
- Actuation: a custom ramp function drives the tip compressive force each step.
- Solver/timestepper: Pardiso MKL direct solver + implicit linearized Euler
  timestepper for stable stiff FEA dynamics through large post-buckling deflection.
- Expected behavior: column stays nearly straight until the ramped load passes the
  Euler critical load, then it buckles and mid-span deflects laterally.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Constants === geometry / material / load schedule (no bare literals downstream)
time_step = 5e-4              # small step for FEA stability
sim_end = 6.0                # seconds
render_fps = 50.0

beam_length = 2.0            # column height (m), built along +Z
n_elements = 16              # Euler beam elements
diameter = 0.02              # circular cross-section diameter (m)
density = 7850.0             # steel (kg/m^3)
youngs_modulus = 2.0e11      # steel Young's modulus (Pa)
shear_modulus = youngs_modulus * 0.35

# Euler critical load for a clamped-pinned column: Pcr = pi^2*E*I/(0.7*L)^2
area_moment = math.pi * (diameter ** 4) / 64.0        # precomputed once: I of circle
euler_critical = (math.pi ** 2) * youngs_modulus * area_moment / (0.7 * beam_length) ** 2

# Ramp the compressive tip load from 0 to ~1.5x the critical load over the run so
# the column is driven gradually (quasi-statically) through the buckling threshold.
max_tip_load = 1.5 * euler_critical
load_ramp_rate = max_tip_load / sim_end               # precomputed once: N per second

# Small constant lateral force at mid-span breaks the perfect symmetry so the column
# has a defined buckling direction. The lateral pins at both ends keep the column
# straight below the critical load; once the axial load passes Pcr the mid-span bows
# out, and this seed sets the bow direction and amplitude.
imperfection_force = 5.0                               # N, precomputed once

base_pos = chrono.ChVector3d(0, 0, 0)                 # clamped base
tip_pos = chrono.ChVector3d(0, 0, beam_length)        # free, axially loaded tip
lateral_dir = chrono.ChVector3d(1, 0, 0)              # beam cross-section reference

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# Strong references to FEA objects so SWIG does not garbage-collect them mid-run.
keepalive = {}

# === System & gravity === SMC system is required for FEA deformable dynamics
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Solver & timestepper === direct MKL solver + implicit linearized Euler
# integrator, stable at a fixed FEA step through the large post-buckling deflection
# regime where an adaptive (HHT) scheme stalls at minimum step size.
sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# === FEA mesh & beam === Euler-Bernoulli column built from a circular steel section
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
keepalive["mesh"] = mesh

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(diameter)
section.SetDensity(density)
section.SetYoungModulus(youngs_modulus)
section.SetShearModulus(shear_modulus)
section.SetRayleighDamping(0.05)   # modest damping -> smooth quasi-static buckling
keepalive["section"] = section

builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(mesh, section, n_elements, base_pos, tip_pos, lateral_dir)
keepalive["builder"] = builder

# Keep a strong reference to the node container before indexing (SWIG GC guard).
beam_nodes_container = builder.GetLastBeamNodes()
beam_nodes = [beam_nodes_container[i] for i in range(beam_nodes_container.size())]
keepalive["nodes"] = beam_nodes

base_node = beam_nodes[0]                  # cache: clamped base node
tip_node = beam_nodes[-1]                  # cache: loaded, laterally-pinned tip node
mid_node = beam_nodes[len(beam_nodes) // 2]   # cache: mid-span node (max bow at buckling)

# Clamp the base node fully (built-in cantilever base boundary condition).
base_node.SetFixed(True)

sys.Add(mesh)

# FEA beam: no contact material needed — driven by constraints + gravity + ramped
# axial load only; the column never collides with a rigid body.

# === Joints / constraints === pin the tip laterally to a fixed ground anchor
# A ChLinkMateGeneric ties the FEA tip node to a fixed rigid body in the two lateral
# directions only (a constraint between two distinct parts: the deformable mesh node
# and the rigid ground), while the axial slide and all rotations stay free. With the
# base clamped this gives a clamped-pinned column that stays straight at both ends and
# bows out at mid-span once the axial load passes the Euler critical load.
anchor = chrono.ChBody()
anchor.SetFixed(True)
anchor.SetPos(tip_pos)
sys.Add(anchor)

tip_link = chrono.ChLinkMateGeneric()
tip_link.Initialize(tip_node, anchor, False, tip_node.Frame(), chrono.ChFramed(tip_pos))
tip_link.SetConstrainedCoords(True, True, False, False, False, False)   # lateral pin only
tip_link.SetName("tip_lateral_pin")
sys.Add(tip_link)
keepalive["tip_link"] = tip_link

# === FEA visualization === colored deformed beam + undeformed wireframe overlay
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.5))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_ref = chrono.ChVisualShapeFEA()
vis_ref.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_ref.SetWireframe(True)
vis_ref.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_ref)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam Buckling (FEA Euler column)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.5, -3.5, 1.4), chrono.ChVector3d(0, 0, 1.0))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === ramp the axial tip load each step and drive the FEA dynamics


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Custom ramp "motor" function for the compressive tip load (a custom
            # ChFunction cannot bind to a node force, so update it explicitly here).
            axial_load = load_ramp_rate * t
            # Axial (-Z) compression at the pinned tip; tiny lateral imperfection at
            # mid-span seeds the buckling mode.
            tip_node.SetForce(chrono.ChVector3d(0.0, 0.0, -axial_load))
            mid_node.SetForce(chrono.ChVector3d(imperfection_force, 0.0, 0.0))
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush data, assemble review video + plot, clean up frames
