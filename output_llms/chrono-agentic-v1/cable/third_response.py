"""
Multiple ANCF Cable Chains with Connected Box Bodies — PyChrono FEA Demo

System: ChSystemSMC (required for FEA / ANCF cable elements)
Solver:  ChSolverSparseQR + ChTimestepperEulerImplicitLinearized (canonical for ANCF cables)
World:   Y-up (standard for FEA demos); gravity = (0, -9.81, 0)

Models 6 chains of ANCF cable elements. Each chain:
  - Has a fixed truss body as anchor.
  - Has increasing element count (chain i → i+3 elements, i=0..5).
  - Is offset in Z so chains do not overlap.
  - Is pinned at the start (hinge constraint between first node and truss).
  - Has a downward point force applied to the free end.
  - Has a box body connected at the endpoint via a ChLinkNodeFrame constraint.

A PrintBodyPositions helper method prints each chain's end-body position per step.
Visualization uses two ChVisualShapeFEA shapes per mesh (surface field + glyph).

Expected behavior: cables sag under gravity and applied load; boxes hang from
the cable endpoints and oscillate before settling.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
TIME_STEP = 0.01          # ANCF cable canonical timestep
SIM_END   = 5.0           # simulation duration [s]
RENDER_FPS = 30.0         # frame rate for review video
N_CHAINS  = 6             # number of cable chains

# Cable section properties
CABLE_DIAMETER    = 0.015   # [m]
CABLE_YOUNG_MOD   = 0.01e9  # [Pa]
CABLE_RAYLEIGH_D  = 0.0
CABLE_LENGTH      = 0.5     # [m] per chain (horizontal extent)

# Box body properties
BOX_MASS          = 0.1     # [kg]
BOX_SIZE          = 0.02    # [m] half-side of box

# Z-spacing between chains (Y-up world: offset along Z axis for visual separation)
CHAIN_Z_SPACING   = 0.12    # [m]

# Downward force applied to free end of cable
CABLE_TIP_FORCE   = chrono.ChVector3d(0, -0.5, 0)  # [N], Y-down

# === System setup — ChSystemSMC for FEA ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up

# ANCF cable uses sparse QR + Euler implicit linearized timestepper
solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# No collision system needed — pure FEA + rigid body constraints, no contact surfaces
# FEA cable: no contact material needed — driven by constraints + gravity + applied forces only

# === Model class: builds N cable chains ===

class Model1:
    """
    Encapsulates all cable chains, their truss anchors, meshes, box bodies,
    hinge constraints, and a helper to print end-body positions.
    """

    def __init__(self, system, n_chains=N_CHAINS):
        self.system   = system
        self.n_chains = n_chains

        # Strong references to prevent SWIG GC of shared_ptrs
        self.meshes       = []
        self.trusses      = []
        self.boxes        = []
        self.hinges_start = []   # ChLinkNodeFrame: first node -> truss
        self.hinges_end   = []   # ChLinkNodeFrame: last node -> box
        self.builders     = []   # keep builders alive for node access

        self._build(system)

    def _build(self, system):
        for i in range(self.n_chains):
            n_elements = i + 3   # chain 0 → 3 elems, chain 5 → 8 elems
            z_offset   = i * CHAIN_Z_SPACING  # separate chains in Z

            # --- Truss (fixed anchor) ---
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            mtruss.SetName(f"truss_{i}")
            system.Add(mtruss)
            self.trusses.append(mtruss)

            # --- ANCF cable section (shared properties per chain) ---
            sec = fea.ChBeamSectionCable()
            sec.SetDiameter(CABLE_DIAMETER)
            sec.SetYoungModulus(CABLE_YOUNG_MOD)
            sec.SetRayleighDamping(CABLE_RAYLEIGH_D)

            # --- FEA mesh ---
            mesh = fea.ChMesh()
            mesh.SetAutomaticGravity(True)

            # Build beam: start at (0, 0, z_offset), end at (CABLE_LENGTH, 0, z_offset)
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                mesh, sec, n_elements,
                chrono.ChVector3d(0.0,  0.0, z_offset),
                chrono.ChVector3d(CABLE_LENGTH, 0.0, z_offset),
            )

            # Retrieve node references — MUST keep container to prevent SWIG GC
            beam_nodes_container = builder.GetLastBeamNodes()
            n_nodes = beam_nodes_container.size()
            node_start = beam_nodes_container[0]
            node_end   = beam_nodes_container[n_nodes - 1]

            # Apply downward force at the free (end) node
            node_end.SetForce(CABLE_TIP_FORCE)

            system.Add(mesh)
            self.meshes.append(mesh)
            self.builders.append((builder, beam_nodes_container))  # cache: keep alive

            # --- Hinge: start node → truss (pin the cable's root) ---
            hinge_start = fea.ChLinkNodeFrame()
            hinge_start.Initialize(node_start, mtruss)
            hinge_start.SetName(f"hinge_start_{i}")
            system.Add(hinge_start)
            self.hinges_start.append(hinge_start)

            # --- Box body connected at the cable endpoint ---
            box = chrono.ChBodyEasyBox(
                BOX_SIZE * 2, BOX_SIZE * 2, BOX_SIZE * 2,
                BOX_MASS / (BOX_SIZE * 2) ** 3,  # density from mass/volume
                True,    # create visual shape
                False,   # no collision shape — pure FEA-coupled, no contact
            )
            box.SetName(f"box_{i}")
            # Position the box at the cable endpoint (will be pulled by constraint)
            box.SetPos(chrono.ChVector3d(CABLE_LENGTH, 0.0, z_offset))
            system.Add(box)
            self.boxes.append(box)

            # --- Hinge: end node → box (attach box to cable tip) ---
            hinge_end = fea.ChLinkNodeFrame()
            hinge_end.Initialize(node_end, box)
            hinge_end.SetName(f"hinge_end_{i}")
            system.Add(hinge_end)
            self.hinges_end.append(hinge_end)

            # --- FEA visualization: two shapes per mesh ---
            # Shape 1 — surface/scalar field (bending moment along beam axis)
            vis_surface = chrono.ChVisualShapeFEA(mesh)
            vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
            vis_surface.SetColorscaleMinMax(-0.4, 0.4)
            vis_surface.SetSmoothFaces(True)
            vis_surface.SetWireframe(False)
            mesh.AddVisualShapeFEA(vis_surface)

            # Shape 2 — glyph: node coordinate-system triads
            vis_glyph = chrono.ChVisualShapeFEA(mesh)
            vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
            vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            vis_glyph.SetSymbolsThickness(0.006)
            vis_glyph.SetSymbolsScale(0.01)
            vis_glyph.SetZbufferHide(False)
            mesh.AddVisualShapeFEA(vis_glyph)

    def PrintBodyPositions(self):
        """Print the world position of each chain's end (box) body."""
        for i, box in enumerate(self.boxes):
            p = box.GetPos()
            print(f"  chain[{i}] box pos: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")


# === Build model ===
model = Model1(sys, N_CHAINS)

# === Visualization — full Irrlicht block ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multiple ANCF Cable Chains — PyChrono FEA")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA demos
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.5, -0.4, 1.5),   # eye: look at the cables from the side
    chrono.ChVector3d(0.25, -0.1, 0.3),  # target: center of cable array
)
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.3, 0.3), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main loop ===
frame = 0
step_count = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):

            sys.DoStepDynamics(TIME_STEP)
            step_count += 1

            # Print body positions every 50 steps (inside physics batch)
            if step_count % 50 == 0:
                print(f"t={sys.GetChTime():.3f}s  End-body positions:")
                model.PrintBodyPositions()

            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad system state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
