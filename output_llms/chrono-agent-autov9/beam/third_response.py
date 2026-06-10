"""Two-segment flexible Euler-Bernoulli FEA beam (chained ChBuilderBeamEuler).

Model
-----
A deformable cantilever assembled from two beam segments built with
`fea.ChBuilderBeamEuler.BuildBeam`. The first segment runs from the clamped
root toward a free intermediate node. A *second* segment is then appended so
that it begins at the last node produced by the first segment (its 'A' node)
and ends at the point B = (0.2, 0.1, -0.1), oriented with the cross-section
'Y' up direction (0, 1, 0). The two segments share the chaining node, so the
result is one continuous, kinematically connected flexible beam.

System type
-----------
`ChSystemSMC` (smooth contact) — the FEA module requires SMC and a direct
solver (Pardiso MKL) for the stiff beam matrices. Gravity acts along -Y so the
beam droops in the vertical plane that contains the Y-up orientation.

Bodies / elements
-----------------
- One `fea.ChMesh` holding all Euler-Bernoulli beam elements of both segments.
- Root node of segment 1 is clamped (`SetFixed(True)`).
- No rigid bodies are welded to the FEA nodes (avoided per solver stability).

Expected behavior
------------------
Under gravity the unclamped portion of the beam deflects (droops) from its
initial straight configuration and settles toward a static equilibrium. The
free tip (point B end) shows the largest displacement; the shared chaining
node shows an intermediate displacement. Both are logged each step.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants: geometry, material, integration ===
# Segment 1 spans from the clamped root to an intermediate "junction" node.
# Segment 2 is appended FROM that junction node TO point B (per the request).
ROOT_POINT = chrono.ChVector3d(0.0, 0.0, 0.0)       # clamped root of segment 1
JUNCTION_POINT = chrono.ChVector3d(0.1, 0.0, 0.0)   # end of seg 1 = start of seg 2
END_POINT_B = chrono.ChVector3d(0.2, 0.1, -0.1)     # 'B' endpoint of segment 2
Y_UP_DIR = chrono.ChVector3d(0.0, 1.0, 0.0)         # cross-section 'Y' up direction

N_ELEMENTS_SEG1 = 6                                  # elements in first segment
N_ELEMENTS_SEG2 = 6                                  # elements in appended segment

BEAM_DIAMETER = 0.02                                 # circular cross-section [m]
BEAM_DENSITY = 2500.0                                # [kg/m^3] — heavier -> larger sag
YOUNG_MODULUS = 5.0e5                                # [Pa] — very soft, large droop
SHEAR_MODULUS = YOUNG_MODULUS * 0.35                 # ~35% of Young's modulus
RAYLEIGH_DAMPING = 0.02                              # structural damping

GRAVITY = chrono.ChVector3d(0.0, -9.81, 0.0)         # gravity along -Y (Y-up beam)

TIME_STEP = 5.0e-4                                   # FEA-stable step
SIM_END = 3.0                                        # [s] settle to equilibrium
RENDER_FPS = 50.0                                    # review-frame cadence

# Fast, windowless validation run gated on SIMBENCH_VALIDATE.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# Derived constants computed ONCE (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 0.3) if HEADLESS else SIM_END          # short physics check

# Strong references kept alive to defeat the SWIG GC pitfall on FEA objects.
KEEPALIVE = []


def build_beam():
    """Build the two-segment chained Euler beam; return (sys, mesh, nodes)."""
    # === System & gravity === SMC + direct MKL solver are mandatory for FEA.
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)
    sys.SetSolver(mkl.ChSolverPardisoMKL())  # direct solver: iterative diverges on FEA

    # HHT timestepper for beam elements. In this PyChrono 9.0.1 build
    # SetTimestepper(obj) is unreliable and the base GetTimestepper() handle does
    # not expose SetAlpha, so select HHT by type and use its built-in defaults.
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

    # === FEA mesh & section === one mesh holds both beam segments.
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)   # let gravity load every beam element

    sec = fea.ChBeamSectionEulerAdvanced()
    sec.SetAsCircularSection(BEAM_DIAMETER)
    sec.SetDensity(BEAM_DENSITY)
    sec.SetYoungModulus(YOUNG_MODULUS)
    sec.SetShearModulus(SHEAR_MODULUS)
    sec.SetRayleighDamping(RAYLEIGH_DAMPING)

    # FEA beam: no contact material needed — driven by the clamp + gravity only
    # (no rigid-body collision in this scene), so no ChContactSurface is added.

    # === Beam segments === segment 1: root -> junction (point-to-point build).
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, sec, N_ELEMENTS_SEG1, ROOT_POINT, JUNCTION_POINT, Y_UP_DIR)

    # Keep a strong ref to the SWIG container BEFORE indexing (GC pitfall).
    seg1_container = builder.GetLastBeamNodes()
    seg1_nodes = [seg1_container[i] for i in range(seg1_container.size())]
    root_node = seg1_nodes[0]
    junction_node = seg1_nodes[-1]   # 'A' node reused as the start of segment 2

    # Clamp the root so the beam behaves as a cantilever.
    root_node.SetFixed(True)

    # Segment 2: append ANOTHER beam segment that starts at the last node of
    # the previous segment (the 'A' node) and ends at point B, Y-up oriented.
    builder.BuildBeam(mesh, sec, N_ELEMENTS_SEG2, junction_node, END_POINT_B, Y_UP_DIR)

    seg2_container = builder.GetLastBeamNodes()
    seg2_nodes = [seg2_container[i] for i in range(seg2_container.size())]
    tip_node = seg2_nodes[-1]   # free tip at point B — largest deflection

    sys.Add(mesh)

    # Defeat SWIG GC: keep every FEA object alive for the whole run.
    KEEPALIVE.extend([mesh, sec, builder, seg1_container, seg2_container,
                      seg1_nodes, seg2_nodes])

    nodes = {
        "root": root_node,
        "junction": junction_node,
        "tip": tip_node,
    }
    return sys, mesh, nodes


def add_fea_visuals(mesh):
    """Attach ChVisualShapeFEA shapes to the mesh (before vis.Initialize)."""
    # Colored beam surface by node speed so motion is visible in the video.
    vis_surface = chrono.ChVisualShapeFEA()
    vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_surface.SetColormapRange(chrono.ChVector2d(0.0, 1.0))
    vis_surface.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_surface)

    # Wireframe overlay of the undeformed reference for visual comparison.
    vis_wire = chrono.ChVisualShapeFEA()
    vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    vis_wire.SetWireframe(True)
    vis_wire.SetDrawInUndeformedReference(True)
    mesh.AddVisualShapeFEA(vis_wire)


def main():
    sys, mesh, nodes = build_beam()
    add_fea_visuals(mesh)

    # cache: node handles fetched once, reused every step (no per-step lookup)
    root_node = nodes["root"]
    junction_node = nodes["junction"]
    tip_node = nodes["tip"]

    # Initial positions snapshotted as plain float tuples ONCE (copying the
    # components, not holding the live ChVector3d, which mutates each step).
    _jp0 = junction_node.GetPos()
    _tp0 = tip_node.GetPos()
    junction_xyz0 = (_jp0.x, _jp0.y, _jp0.z)   # precomputed once
    tip_xyz0 = (_tp0.x, _tp0.y, _tp0.z)        # precomputed once

    vis = None
    if not HEADLESS:
        # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Two-segment chained FEA Euler beam")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                     # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(0.18, 0.12, 0.45),
                      chrono.ChVector3d(0.12, -0.12, -0.05))  # AFTER Initialize
        vis.AddTypicalLights()                              # standard lighting
        vis.AddGrid(0.05, 0.05, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    os.makedirs("frames", exist_ok=True)   # guard against missing output dir

    csv_file = None
    times, tip_dy, junc_dy = [], [], []
    try:
        # Guard the CSV open specifically against disk / permission errors.
        try:
            csv_file = open("simulation_data.csv", "w", newline="")
        except (OSError, IOError) as exc:   # disk full / permission denied
            print("Could not open CSV for writing:", exc)
            raise

        with csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "time",
                "junction_x", "junction_y", "junction_z", "junction_disp",
                "tip_x", "tip_y", "tip_z", "tip_disp",
            ])

            # === Main loop === render-cadence outer loop; physics in inner batch.
            frame = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive
                    frame += 1
                for _ in range(RENDER_EVERY):
                    t = sys.GetChTime()
                    jp = junction_node.GetPos()
                    tp = tip_node.GetPos()
                    j_disp = math.sqrt((jp.x - junction_xyz0[0]) ** 2 +
                                       (jp.y - junction_xyz0[1]) ** 2 +
                                       (jp.z - junction_xyz0[2]) ** 2)
                    t_disp = math.sqrt((tp.x - tip_xyz0[0]) ** 2 +
                                       (tp.y - tip_xyz0[1]) ** 2 +
                                       (tp.z - tip_xyz0[2]) ** 2)
                    writer.writerow([t,
                                     jp.x, jp.y, jp.z, j_disp,
                                     tp.x, tp.y, tp.z, t_disp])
                    times.append(t)
                    tip_dy.append(t_disp)
                    junc_dy.append(j_disp)

                    sys.DoStepDynamics(TIME_STEP)
                    if sys.GetChTime() >= RUN_END:
                        break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad FEA state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # CSV is closed by the `with` block; report final state for diagnostics.
        if times:
            print(f"steps logged: {len(times)}  final tip_disp: {tip_dy[-1]:.5f} m")

    # === Post-processing === plot displacement magnitudes vs time.
    if times:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(times, tip_dy, label="tip (point B) displacement")
        ax.plot(times, junc_dy, label="junction node displacement")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("displacement magnitude [m]")
        ax.set_title("Chained two-segment FEA beam — node displacement")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
