"""Cantilever beam built from Euler-Bernoulli finite elements (PyChrono FEA).

Model
-----
A single flexible beam is discretized into a chain of Euler-Bernoulli beam
elements connected by `ChNodeFEAxyzrot` nodes inside one `ChMesh`. The beam has
an explicit circular cross-section with named material properties (Young's
modulus, shear modulus, density, Rayleigh damping). The root node is clamped
(`SetFixed(True)`), so the structure behaves as a cantilever.

System type
-----------
`ChSystemSMC` with a Pardiso/MKL direct solver and the HHT implicit
timestepper, as required for stiff FEA beam matrices (iterative solvers and
explicit steppers diverge here).

Expected behavior
------------------
Under gravity the free tip sags downward, oscillates a few times because of the
small structural damping, and settles toward a static deflection. The tip
vertical position is the quantity of interest. There is no rigid-body contact in
this scene, so no collision system or contact material is needed.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


# === Parameters === geometry, material, and time stepping (named constants)
beam_length = 2.0           # m, total cantilever span
n_elements = 16             # number of beam finite elements along the span
beam_diameter = 0.05        # m, circular cross-section diameter
youngs_modulus = 2.0e8      # Pa, soft so gravity deflection is clearly visible
shear_modulus = youngs_modulus * 0.35   # Pa, ~35% of Young's modulus
density = 1000.0            # kg/m^3
rayleigh_damping = 0.01     # structural (stiffness-proportional) damping

time_step = 5.0e-4          # s, small step required for FEA stability
sim_end = 4.0               # s, long enough to see sag + a few oscillations
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & gravity === SMC system; FEA needs a direct solver + HHT stepper
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Pure FEA beam: no contact/collision in this scene -> no collision system,
# no contact material, no collision surface needed (driven by gravity + the
# clamped root constraint only).
sys.SetSolver(mkl.ChSolverPardisoMKL())     # direct solver required for FEA stiffness
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)   # implicit HHT for stiff beams

# === FEA mesh === beam section properties + the beam/node chain
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)              # let gravity load each element

section = fea.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(beam_diameter)
section.SetDensity(density)
section.SetYoungModulus(youngs_modulus)
section.SetShearModulus(shear_modulus)
section.SetRayleighDamping(rayleigh_damping)

up = chrono.ChVector3d(0, 1, 0)             # lateral reference direction for the section
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, section, n_elements,
    chrono.ChVector3d(0, 0, 0),             # clamped root end
    chrono.ChVector3d(beam_length, 0, 0),   # free tip end
    up,
)

# Keep strong Python references to the SWIG containers so the node shared_ptrs
# are not garbage-collected (indexing a temporary would dangle and segfault).
beam_nodes = builder.GetLastBeamNodes()     # cache: container held alive, reused below
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]
root_node = nodes[0]
tip_node = nodes[-1]                        # cache: tip handle fetched once, logged each step
root_node.SetFixed(True)                    # clamp the root -> cantilever boundary condition

# Prevent premature GC of the FEA objects across the run.
keepalive = [mesh, section, builder, beam_nodes, nodes]

sys.Add(mesh)

# === FEA visualization === colored deformed surface + undeformed wireframe overlay
vis_beam = chrono.ChVisualShapeFEA()
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 0.6))
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_wire = chrono.ChVisualShapeFEA()
vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_wire.SetWireframe(True)
vis_wire.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(vis_wire)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA Cantilever Beam (Euler-Bernoulli)")
vis.Initialize()                            # Initialize FIRST, then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(beam_length * 0.5, -3.0, 1.2),
              chrono.ChVector3d(beam_length * 0.5, 0, -0.3))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(beam_length * 0.5, 0, -1.0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render once per frame, batch physics, log tip deflection
tip_z0 = tip_node.GetPos().z                # precomputed once: reference tip height


try:

    frame = 0
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

# === Post-processing === assemble review video + plot, then clean frame dirs
