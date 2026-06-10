"""ANCF cable beam hinged to ground, swinging under gravity (PyChrono 9.0.1, Irrlicht).

Models a single flexible beam built from ANCF cable finite elements
(fea.ChElementCableANCF via fea.ChBuilderCableANCF). One end of the cable is pinned
to a fixed rigid truss with a node-to-frame constraint (fea.ChLinkNodeFrame); the
free end and interior nodes deform under gravity, so the beam sags and swings like a
flexible chain.

System type: ChSystemSMC (FEA requires SMC + a direct sparse solver). The cable is a
pure FEA body with no rigid-contact interaction, so no contact material or collision
surface is defined. Integration uses the HHT timestepper with the PardisoMKL direct
solver. Expected behavior: starting horizontal, the cable sags downward under gravity
and oscillates about the hinge while its element nodes (rendered as dots) trace the
deformation.
"""

import os
import math

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Parameters === geometry / material / integration constants (no bare literals downstream)
time_step = 5e-4                  # FEA-stable step for ANCF cable + HHT
sim_end = 5.0                     # seconds of simulated swing
render_fps = 50.0                 # review-frame cadence
cable_length = 1.0                # m, total horizontal span at rest
n_elements = 12                   # number of ANCF cable elements
cable_diameter = 0.01             # m, circular cross-section diameter
cable_density = 1000.0            # kg/m^3
cable_E = 1.0e8                   # Pa, Young's modulus (slender, flexible)
cable_damping = 0.02              # Rayleigh damping coefficient
hinge_pos = chrono.ChVector3d(0.0, 0.0, 0.0)          # pinned (hinged) end at origin
free_pos = chrono.ChVector3d(cable_length, 0.0, 0.0)  # free end, horizontal start

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === SMC system (required for FEA) with downward gravity
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Direct sparse solver — iterative solvers diverge on FEA stiffness matrices.
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT implicit timestepper for stable ANCF beam integration.
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# === Truss === fixed rigid body that anchors the hinged cable end
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(hinge_pos)
sys.Add(truss)

# === FEA mesh & cable === ANCF cable elements built between hinge and free end
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)   # apply gravity to FEA nodes

# Cable cross-section properties (slender flexible rod).
section = fea.ChBeamSectionCable()
section.SetDiameter(cable_diameter)
section.SetYoungModulus(cable_E)
section.SetDensity(cable_density)
section.SetRayleighDamping(cable_damping)

# ANCF cable: no contact material / collision surface needed — the beam is driven by
# gravity + the hinge constraint only and never collides with a rigid body.
builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, section, n_elements, hinge_pos, free_pos)

# Keep strong references to the SWIG node container before indexing (GC pitfall).
beam_node_container = builder.GetLastBeamNodes()                       # cache: fetched once, reused
cable_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]
hinge_node = cable_nodes[0]      # node pinned to the truss
tip_node = cable_nodes[-1]       # free end node

sys.Add(mesh)

# === Constraints === pin the first cable node to the fixed truss (the "hinge")
hinge_link = fea.ChLinkNodeFrame()
hinge_link.Initialize(hinge_node, truss)
sys.Add(hinge_link)

# === FEA visualization === colored deformation + nodal-position dot glyphs
vis_cable = chrono.ChVisualShapeFEA()
vis_cable.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_cable.SetColormapRange(chrono.ChVector2d(0.0, 3.0))
vis_cable.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_cable)

vis_nodes = chrono.ChVisualShapeFEA()
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.008)
mesh.AddVisualShapeFEA(vis_nodes)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Beam Hinged to Ground")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, -2.0, 0.3), chrono.ChVector3d(0.5, 0.0, -0.3))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.2), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics advanced in inner batch


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

# === Post-processing === assemble review video + plot the logged tip trajectory
