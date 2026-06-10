"""Euler-Bernoulli cantilever beam under a tip point load (PyChrono FEA).

Model
-----
A single deformable Euler-Bernoulli beam built with the ``ChBuilderBeamEuler``
helper. The beam spans from point A = (0, 0, -0.1) to point B = (0.2, 0, -0.1)
with a 'Y' up reference direction and is discretised into 5 beam elements.

System type
-----------
``ChSystemSMC`` (smooth contact) with the MKL Pardiso direct solver and the HHT
implicit timestepper — the standard, stable combination for stiff FEA beam
matrices (iterative solvers diverge here).

Constraints & loads
--------------------
* The LAST node of the built beam is clamped to ground with ``SetFixed(True)``.
* The FIRST node is held to ground through a ``ChLinkMateGeneric`` constraint
  (rather than a direct ``SetFixed`` call), locking all six of its coordinates.
* A constant external force of (0, -1, 0) N is applied to the first node.

Expected behavior
------------------
Because the first node is fixed by a full 6-DOF mate while a downward (-Y) force
is applied to it, the constrained node holds its position and the interior of the
beam settles into a small static deflection. The reaction the mate carries grows
to balance the applied load; tip/first-node Y displacement converges to a small
steady value. CSV logs the first-node displacement and the beam tip position.
"""

import os
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants === geometry / physics / discretisation (no bare literals downstream)
GRAVITY = chrono.ChVector3d(0, -9.81, 0)   # gravity along -Y (matches 'Y' up beam frame)
BEAM_START = chrono.ChVector3d(0.0, 0.0, -0.1)   # point A
BEAM_END = chrono.ChVector3d(0.2, 0.0, -0.1)     # point B
BEAM_UP = chrono.ChVector3d(0, 1, 0)             # 'Y' up reference direction
N_ELEMENTS = 5                                    # 5 beam elements
APPLIED_FORCE = chrono.ChVector3d(0, -1, 0)       # force on first node (N)

BEAM_DIAMETER = 0.01      # circular cross-section diameter (m)
BEAM_DENSITY = 1000.0     # kg/m^3
YOUNG_MODULUS = 2.0e8     # Pa (flexible beam so deflection is visible)
SHEAR_MODULUS = YOUNG_MODULUS * 0.35
RAYLEIGH_DAMPING = 0.01

TIME_STEP = 5e-4          # small step for FEA stability
SIM_END = 2.0             # seconds
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

# Camera framing derived once from the beam span (precomputed; not recomputed in loop)
BEAM_MID = (BEAM_START + BEAM_END) * 0.5
CAM_EYE = chrono.ChVector3d(BEAM_MID.x, 0.25, BEAM_MID.z + 0.6)
CAM_TARGET = chrono.ChVector3d(BEAM_MID.x, -0.02, BEAM_MID.z)


def main():
    # === System & gravity === SMC system required for FEA + direct MKL solver
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)

    # MKL Pardiso direct solver — required: iterative solvers diverge on FEA stiffness.
    sys.SetSolver(mkl.ChSolverPardisoMKL())

    # HHT implicit timestepper for the stiff beam matrices. In this PyChrono 9.0.1
    # build only SetTimestepperType is reliable (the shared_ptr SetTimestepper(obj)
    # binding is broken), so select HHT by type and keep its stable default alpha.
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

    # === FEA mesh & beam === Euler-Bernoulli beam via the ChBuilderBeamEuler helper
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)

    # Circular Euler-Bernoulli beam section properties.
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsCircularSection(BEAM_DIAMETER)
    section.SetDensity(BEAM_DENSITY)
    section.SetYoungModulus(YOUNG_MODULUS)
    section.SetShearModulus(SHEAR_MODULUS)
    section.SetRayleighDamping(RAYLEIGH_DAMPING)

    # Build the beam from point A to point B with 5 elements and 'Y' up direction.
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, section, N_ELEMENTS, BEAM_START, BEAM_END, BEAM_UP)

    # FEA beam: no contact material needed — driven by constraints + gravity + the
    # applied nodal force only (the beam never collides with a rigid body).

    # SWIG GC guard: store the node container BEFORE indexing, then keep strong refs.
    beam_nodes = builder.GetLastBeamNodes()                      # cache: fetched once, reused below
    nodes = [beam_nodes[i] for i in range(beam_nodes.size())]    # strong refs (avoid dangling ptr)
    first_node = nodes[0]    # cache: first node handle, reused every step for logging/force
    last_node = nodes[-1]    # cache: last node handle (clamped)
    mid_node = nodes[len(nodes) // 2]   # cache: interior midspan node (free to deflect)

    # Fix the LAST node of the created beam directly.
    last_node.SetFixed(True)

    sys.Add(mesh)

    # === Joints / constraints === fix node 1 with a 6-DOF mate (NOT a direct SetFixed)
    # A ground body provides the anchor frame for the ChLinkMateGeneric constraint.
    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.Add(ground)

    # Replaces a direct 'first_node.SetFixed(True)': constrain the first node to ground
    # in all six coordinates through a generic mate at the node's current frame.
    node1_link = chrono.ChLinkMateGeneric()
    node1_link.Initialize(first_node, ground, False,
                          first_node.Frame(), first_node.Frame())
    node1_link.SetConstrainedCoords(True, True, True, True, True, True)   # lock all 6 DOF
    node1_link.SetName("fix_node1_mate")
    sys.Add(node1_link)

    # Apply the constant external force (0, -1, 0) N to the first node.
    first_node.SetForce(APPLIED_FORCE)

    # === FEA visualization === ChVisualShapeFEA on the mesh (attach BEFORE Initialize)
    vis_beam = chrono.ChVisualShapeFEA()
    vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    vis_beam.SetColormapRange(chrono.ChVector2d(-0.5, 0.5))
    vis_beam.SetSmoothFaces(True)
    vis_beam.SetWireframe(False)
    mesh.AddVisualShapeFEA(vis_beam)

    # Node markers so individual beam nodes are visible in the render.
    vis_nodes = chrono.ChVisualShapeFEA()
    vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    vis_nodes.SetSymbolsThickness(0.004)
    mesh.AddVisualShapeFEA(vis_nodes)

    # === Visualization === full Irrlicht scene (gated for fast headless validation)
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Euler-Bernoulli Beam — tip load (FEA)")
        vis.Initialize()                                     # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                      # standard sky backdrop
        vis.AddCamera(CAM_EYE, CAM_TARGET)                   # AFTER Initialize
        vis.AddTypicalLights()                               # standard lighting
        vis.AddGrid(0.05, 0.05, 20, 20,
                    chrono.ChCoordsysd(chrono.ChVector3d(BEAM_MID.x, -0.1, BEAM_MID.z),
                                       chrono.Q_ROTATE_Y_TO_Z),
                    chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

    # === Main loop === render-cadence outer loop; physics + CSV in the inner batch
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir

    # Reference (undeformed) Y positions for displacement logging — precomputed once.
    # Snapshot the SCALAR (GetPos() returns a live alias in this build, so holding the
    # ChVector3d would track the node and make every displacement read as zero).
    first0_y = first_node.GetPos().y
    last0_y = last_node.GetPos().y
    mid0_y = mid_node.GetPos().y

    csv_file = None
    writer = None
    try:
        try:
            csv_file = open("simulation_data.csv", "w", newline="")   # context-managed below
        except (OSError, IOError) as exc:   # disk full / permission denied
            print(f"Cannot open CSV for writing: {exc}")
            raise
        with csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "time",
                "first_node_x", "first_node_y", "first_node_z",
                "first_node_dy",
                "mid_x", "mid_y", "mid_z",
                "mid_dy",
                "tip_x", "tip_y", "tip_z",
                "tip_dy",
            ])

            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index
                    frame += 1
                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()
                    p_first = first_node.GetPos()
                    p_mid = mid_node.GetPos()
                    p_last = last_node.GetPos()
                    writer.writerow([
                        f"{t:.6f}",
                        f"{p_first.x:.8f}", f"{p_first.y:.8f}", f"{p_first.z:.8f}",
                        f"{p_first.y - first0_y:.8f}",
                        f"{p_mid.x:.8f}", f"{p_mid.y:.8f}", f"{p_mid.z:.8f}",
                        f"{p_mid.y - mid0_y:.8f}",
                        f"{p_last.x:.8f}", f"{p_last.y:.8f}", f"{p_last.z:.8f}",
                        f"{p_last.y - last0_y:.8f}",
                    ])
                    sys.DoStepDynamics(TIME_STEP)
                    if sys.GetChTime() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # CSV is closed by the `with` block; ensure any buffered output is flushed.
        if csv_file is not None and not csv_file.closed:
            csv_file.close()

    # === Post-processing === plot first-node and tip Y-displacement vs time
    times, first_dy, mid_dy, tip_dy = [], [], [], []
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                first_dy.append(float(row["first_node_dy"]))
                mid_dy.append(float(row["mid_dy"]))
                tip_dy.append(float(row["tip_dy"]))
    except (OSError, IOError) as exc:   # CSV missing / unreadable
        print(f"Cannot read CSV for plotting: {exc}")
        return

    if times:
        plt.figure(figsize=(9, 5))
        plt.plot(times, first_dy, label="first-node dY (m)")
        plt.plot(times, mid_dy, label="midspan dY (m)")
        plt.plot(times, tip_dy, label="last-node dY (m)")
        plt.xlabel("time (s)")
        plt.ylabel("Y displacement (m)")
        plt.title("Euler-Bernoulli beam — nodal Y displacement")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("simulation_timeseries.png", dpi=120)
        plt.close()


if __name__ == "__main__":
    main()
