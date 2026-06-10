"""Multiple ANCF cable chains hanging from a fixed truss (PyChrono 9.0.1, FEA).

Model
-----
A `Model1` object builds `n_chains` flexible cable chains. Each chain is an ANCF
cable beam (Euler--Bernoulli-like, large-deflection) created with
`fea.ChBuilderCableANCF`. The number of cable elements grows with the chain
index so successive chains are progressively more refined. Every chain:
  * has its own fixed `truss` body acting as a static reference frame,
  * is started/ended at offset positions (laid out along Y) so chains do not
    overlap,
  * is hinged at its start node to the truss via `fea.ChLinkNodeFrame`,
  * carries a small downward point load applied to its end node,
  * has its end node connected to a free rigid box body, with a second
    `fea.ChLinkNodeFrame` constraint tying the cable end node to that box.

System type: `ChSystemSMC` (FEA requires SMC + a direct solver). The scene has
NO rigid-body contact/collision — the cables are driven purely by gravity,
internal elasticity, hinge constraints, and the applied end loads — so no
contact material, no collision surface, and no collision system are created.

Expected behavior: each cable sags under gravity and the end load, swinging the
attached box, then settles into a smooth catenary-like static droop. The
`PrintBodyPositions` method reports the end-box position of every chain each
frame to confirm smooth, bounded dynamics.
"""

import os
import math

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl


# === Named constants === geometry / physics / timing (no bare literals downstream)
N_CHAINS_DEFAULT = 6            # number of cable chains to build
BASE_ELEMENTS = 6              # cable elements in the first chain
CABLE_LENGTH = 1.2             # horizontal span of each cable (m), along +X
CABLE_DIAMETER = 0.015         # cable cross-section diameter (m)
CABLE_E = 0.01e9               # Young's modulus (Pa) — soft, visibly flexible cable
CABLE_DENSITY = 1000.0         # cable material density (kg/m^3)
CABLE_RAYLEIGH = 0.0001        # Rayleigh (stiffness-proportional) damping
CHAIN_SPACING = 0.4            # spacing between chains along Y (m), avoids overlap
ANCHOR_Z = 1.5                 # Z height of the fixed anchor / truss (m)
END_LOAD = chrono.ChVector3d(0, 0, -0.7)   # downward point load on each end node (N)
BOX_SIZE = 0.08                # end-box edge length (m)
BOX_MASS = 0.02                # end-box mass (kg)

time_step = 5e-4               # FEA-stable timestep
sim_end = 4.0                  # simulation duration (s)
render_fps = 50.0              # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
GRAVITY = chrono.ChVector3d(0, 0, -9.81)


# === Model === builds and owns all cable chains + their end bodies
class Model1:
    """Builds `n_chains` ANCF cable chains pinned to per-chain fixed trusses."""

    def __init__(self, sys, n_chains=N_CHAINS_DEFAULT):
        self.sys = sys
        self.n_chains = n_chains
        # Strong references kept so SWIG temporaries are not garbage-collected
        # (meshes / builders / sections / node containers must outlive __init__).
        self._keepalive = []
        self.end_bodies = []        # end-box rigid body per chain
        self.end_nodes = []         # cable end FEA node per chain

        for i in range(self.n_chains):
            self._build_chain(i)

    def _build_chain(self, i):
        # --- per-chain fixed truss (static reference frame) ---
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)
        self.sys.Add(mtruss)

        # --- cable section: circular ANCF cable, soft and flexible ---
        section = fea.ChBeamSectionCable()
        section.SetDiameter(CABLE_DIAMETER)
        section.SetYoungModulus(CABLE_E)
        section.SetDensity(CABLE_DENSITY)
        section.SetRayleighDamping(CABLE_RAYLEIGH)

        # --- mesh that holds this chain's elements ---
        mesh = fea.ChMesh()
        mesh.SetAutomaticGravity(True)

        # Element count increases with each chain (progressive refinement).
        n_elements = BASE_ELEMENTS + i

        # Offset each chain along Y so they do not overlap; anchor at +X start.
        y = i * CHAIN_SPACING
        start = chrono.ChVector3d(0.0, y, ANCHOR_Z)
        end = chrono.ChVector3d(CABLE_LENGTH, y, ANCHOR_Z)

        # --- build the ANCF cable beam ---
        builder = fea.ChBuilderCableANCF()
        builder.BuildBeam(mesh, section, n_elements, start, end)

        # Keep strong references to the node containers BEFORE indexing (SWIG GC).
        beam_nodes = builder.GetLastBeamNodes()
        node_start = beam_nodes.front()
        node_end = beam_nodes.back()

        # --- boundary condition: hinge the start node to the fixed truss ---
        hinge = fea.ChLinkNodeFrame()
        hinge.Initialize(node_start, mtruss)
        self.sys.Add(hinge)

        # --- load: small downward point force on the cable end node ---
        node_end.SetForce(END_LOAD)

        # --- connect the cable end to a free rigid box body ---
        box = chrono.ChBody()
        box.SetMass(BOX_MASS)
        box.SetPos(end)
        inertia = (BOX_MASS / 6.0) * BOX_SIZE * BOX_SIZE
        box.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))
        box_shape = chrono.ChVisualShapeBox(BOX_SIZE, BOX_SIZE, BOX_SIZE)
        box_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
        box.AddVisualShape(box_shape)
        self.sys.Add(box)

        # --- constraint: tie the cable end node to the box (further constraint) ---
        end_link = fea.ChLinkNodeFrame()
        end_link.Initialize(node_end, box)
        self.sys.Add(end_link)

        # --- visualization for the cable elements of this chain ---
        vis_beam = chrono.ChVisualShapeFEA()
        vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
        vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 1.0))
        vis_beam.SetSmoothFaces(True)
        vis_beam.SetWireframe(False)
        mesh.AddVisualShapeFEA(vis_beam)

        vis_nodes = chrono.ChVisualShapeFEA()
        vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        vis_nodes.SetSymbolsThickness(0.006)
        mesh.AddVisualShapeFEA(vis_nodes)

        self.sys.Add(mesh)

        # Retain everything that must survive past this method.
        self._keepalive.extend([mtruss, section, mesh, builder, beam_nodes,
                                node_start, node_end, hinge, end_link])
        self.end_bodies.append(box)
        self.end_nodes.append(node_end)

    def PrintBodyPositions(self):
        """Print the end-box position of every chain (smooth-dynamics check)."""
        t = self.sys.GetChTime()
        parts = []
        for i, body in enumerate(self.end_bodies):
            p = body.GetPos()   # cache: fetched once per body per call
            parts.append(f"chain{i}=({p.x:.4f},{p.y:.4f},{p.z:.4f})")
        print(f"t={t:.4f}  " + "  ".join(parts))


# === System & gravity === FEA needs SMC + a direct (MKL) solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(GRAVITY)
# FEA cable chains: NO rigid contact in this scene — driven by constraints +
# gravity + end loads only, so no contact material / collision system is set.
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# HHT timestepper — robust for stiff beam/cable dynamics (implicit, A-stable).
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# === Model === build the cable chains
model = Model1(sys, n_chains=N_CHAINS_DEFAULT)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
cam_eye = chrono.ChVector3d(CABLE_LENGTH * 0.5, -2.6,
                            ANCHOR_Z + 0.2)            # precomputed once
cam_target = chrono.ChVector3d(CABLE_LENGTH * 0.5,
                               (N_CHAINS_DEFAULT - 1) * CHAIN_SPACING * 0.5,
                               ANCHOR_Z - 0.6)         # precomputed once
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF cable chains")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, cam_target)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics in the inner batch


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        model.PrintBodyPositions()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad numeric state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
