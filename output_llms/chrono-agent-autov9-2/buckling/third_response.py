"""Buckling of a slender elastic column (FEA, Euler-Bernoulli beams).

Models a thin vertical column built from Euler-Bernoulli beam finite elements
(`ChMesh` + `ChBuilderBeamEuler`). The base node is clamped (fully fixed). A
compressive axial load is applied at the free tip and ramped up over time, while a
small constant lateral seed force at the column mid-span breaks the perfect
symmetry. Once the compressive load passes the critical (Euler) buckling load, the
column can no longer remain straight and bows sideways into its first buckling
mode, with the lateral deflection growing rapidly thereafter.

System type: ChSystemSMC (FEA requires SMC + a direct solver: Pardiso MKL).
Main objects: one ChMesh with a single clamped Euler beam column; the tip node
carries a ramped downward (compressive) load, a mid-span node carries a small
lateral seed force.
Expected behavior: the column stays nearly straight at low load, then visibly
bows laterally (buckles) once the ramped compressive load exceeds the critical
load; the lateral tip deflection grows while the axial load keeps increasing.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Constants === geometry / material / timing (no bare literals downstream)
beam_length = 1.0            # column height (m), vertical along +Z
beam_diameter = 0.02         # slender circular cross-section (m)
n_elements = 16              # beam finite elements along the column
youngs_modulus = 2.0e8       # Pa: soft enough that buckling load is reachable
density = 1000.0             # kg/m3
shear_modulus = youngs_modulus * 0.35
rayleigh_damping = 0.02

base_z = 0.0
tip_z = base_z + beam_length

load_ramp_rate = 4.0         # N/s: rate at which the axial compressive load grows
max_axial_load = 16.0        # N: final compressive load held at the tip
lateral_seed_force = 0.4     # N: tiny perturbation at mid-span to seed the mode

time_step = 5e-4             # FEA-stable step
load_ramp_time = max_axial_load / load_ramp_rate
sim_end = load_ramp_time + 1.5   # ramp the load up, then hold and let it settle

render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# Euler critical load for a fixed-free column (reference only, for context):
#   I = pi*d^4/64 ; Pcr = pi^2 * E * I / (2*L)^2
area_moment = math.pi * beam_diameter**4 / 64.0                 # precomputed once
euler_pcr = math.pi**2 * youngs_modulus * area_moment / (2.0 * beam_length)**2

# === System & gravity === FEA needs SMC + direct (Pardiso MKL) solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Pure FEA buckling: no rigid-body contact anywhere, so NO collision system is set.
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper: implicit, robust for the stiff beam dynamics of buckling.
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# === FEA mesh & beam === single Euler-Bernoulli column, clamped at the base
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(density)
section.SetYoungModulus(youngs_modulus)
section.SetShearModulus(shear_modulus)
section.SetRayleighDamping(rayleigh_damping)

# FEA beam: no contact material needed — driven by the clamp, gravity, and applied
# node forces only; the column never collides with a rigid body.
lateral_up = chrono.ChVector3d(1, 0, 0)   # cross-section reference direction

builder = fea.ChBuilderBeamEuler()        # this column's OWN builder (never shared)
builder.BuildBeam(
    mesh, section, n_elements,
    chrono.ChVector3d(0, 0, base_z),      # clamped base
    chrono.ChVector3d(0, 0, tip_z),       # driven tip
    lateral_up,
)

# Keep strong refs to the SWIG node container so node shared_ptrs do not dangle.
beam_nodes = builder.GetLastBeamNodes()
column_nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
base_node = column_nodes[0]
tip_node = column_nodes[-1]
mid_node = column_nodes[len(column_nodes) // 2]

base_node.SetFixed(True)                  # clamp: fully fix the base node

# Lateral seed force at mid-span to break symmetry and select a buckling mode.
mid_node.SetForce(chrono.ChVector3d(lateral_seed_force, 0, 0))

sys.Add(mesh)

# === Visualization === FEA color-mapped beam + wireframe reference, full Irrlicht scene
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.5))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Column Buckling (FEA Euler beam)")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.2, -2.2, 1.0), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.2, 0.2, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === compress the column and record tip / mid deflection vs time

# cache: node handles fetched once, reused every step
tip_ref = tip_node
mid_ref = mid_node

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Ramp the compressive axial load up to its cap, then hold it.
            axial = min(max_axial_load, load_ramp_rate * t)
            tip_ref.SetForce(chrono.ChVector3d(0, 0, -axial))
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FEA state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + timeseries plot, clean frames
