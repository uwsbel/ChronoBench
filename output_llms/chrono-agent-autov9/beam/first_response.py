"""Finite-element beam cantilever simulation (PyChrono FEA + Irrlicht).

Model
-----
A single horizontal cantilever beam discretised with Euler-Bernoulli beam
finite elements (`ChElementBeamEuler`) built via `ChBuilderBeamEuler`. The beam
mesh is composed of `N_ELEMENTS` beam elements and `N_ELEMENTS + 1` shared
nodes. The root node is clamped (fixed in all 6 DOF); the remaining nodes are
free. Under gravity the free end sags and oscillates about its static
equilibrium — a classic deformable-structure response.

System type
-----------
`ChSystemSMC` (smooth contact) with the Pardiso MKL direct solver and an HHT
implicit timestepper — the combination required for stable FEA beam dynamics
(iterative solvers diverge on the stiff FEA system).

Main entities
-------------
- one `ChMesh` holding the FEA beam (nodes + elements),
- one `ChBeamSectionEulerAdvanced` circular section (geometry + material),
- a `ChVisualShapeFEA` (deformed, speed-colored) plus a wireframe undeformed
  reference, so the beam is visible in the Irrlicht window.

Expected behaviour
------------------
The clamped root stays at its initial position; the free tip deflects downward
under self-weight and oscillates with damped motion (Rayleigh damping), settling
toward a static cantilever deflection. Tip vertical position and tip speed are
logged each step to `simulation_data.csv` and plotted to
`simulation_timeseries.png`.
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

# === Named constants === geometry / material / time discretisation
BEAM_LENGTH = 3.0           # m, total cantilever length along +X
BEAM_DIAMETER = 0.06        # m, circular cross-section diameter
N_ELEMENTS = 20             # number of Euler beam elements
BEAM_DENSITY = 700.0        # kg/m^3, wood-like (compliant, visibly deformable)
YOUNG_MODULUS = 9.0e9       # Pa, wood (low E -> large, clearly visible tip sag)
SHEAR_MODULUS = YOUNG_MODULUS * 0.35   # Pa, ~G for wood
RAYLEIGH_DAMPING = 0.02     # structural damping coefficient

GRAVITY = chrono.ChVector3d(0, 0, -9.81)   # Z-up world

TIME_STEP = 5e-4            # s, small step for FEA stability
SIM_END = 3.0              # s, enough to see tip sag + oscillation
RENDER_FPS = 50.0          # review-video frame rate

# Beam endpoints (root clamped at origin, tip extends along +X)
ROOT_POINT = chrono.ChVector3d(0.0, 0.0, 0.0)
TIP_POINT = chrono.ChVector3d(BEAM_LENGTH, 0.0, 0.0)
LATERAL_DIR = chrono.ChVector3d(0, 1, 0)   # beam cross-section "up" reference

# Headless validation gate: fast windowless physics check (no rendering).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# Precompute derived loop constants ONCE (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short check when validating

# Keep strong references to FEA objects so SWIG temporaries are not GC'd.
_keepalive = {}


def build_system():
    """Create the SMC system with the MKL solver and HHT timestepper for FEA."""
    # === System & gravity === SMC system is required for FEA elements
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Direct solver (Pardiso MKL) — iterative solvers diverge on FEA stiffness.
    sys.SetSolver(mkl.ChSolverPardisoMKL())

    # HHT implicit timestepper — stable for stiff Euler beam dynamics.
    # Select via SetTimestepperType (idiomatic), then tune alpha on the live object.
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    stepper = sys.GetTimestepper()
    if isinstance(stepper, chrono.ChTimestepperHHT):
        stepper.SetAlpha(-0.2)   # mild numerical damping for stability
    return sys


def build_beam(sys):
    """Build the FEA beam mesh; return (mesh, tip_node)."""
    # === FEA mesh & beam section === circular Euler-Bernoulli cantilever
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)   # gravity acts on the FEA nodes

    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsCircularSection(BEAM_DIAMETER)
    section.SetDensity(BEAM_DENSITY)
    section.SetYoungModulus(YOUNG_MODULUS)
    section.SetShearModulus(SHEAR_MODULUS)
    section.SetRayleighDamping(RAYLEIGH_DAMPING)

    # FEA beam: no contact material needed — driven by the clamp + gravity only
    # (no rigid-body collision in this scene).
    builder = fea.ChBuilderBeamEuler()
    builder.BuildBeam(mesh, section, N_ELEMENTS, ROOT_POINT, TIP_POINT, LATERAL_DIR)

    # SWIG GC pitfall: store the node container BEFORE indexing into it.
    beam_nodes = builder.GetLastBeamNodes()
    root_node = beam_nodes.front()
    tip_node = beam_nodes.back()
    root_node.SetFixed(True)   # clamp the cantilever root (all 6 DOF)

    sys.Add(mesh)

    # === FEA visualization shapes === make the beam visible in Irrlicht
    vis_speed = chrono.ChVisualShapeFEA()
    vis_speed.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_speed.SetColormapRange(chrono.ChVector2d(0.0, 3.0))
    vis_speed.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_speed)

    vis_wire = chrono.ChVisualShapeFEA()
    vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_wire.SetWireframe(True)
    vis_wire.SetDrawInUndeformedReference(True)
    mesh.AddVisualShapeFEA(vis_wire)

    # Keep strong references (prevent premature GC -> dangling node pointers).
    _keepalive["mesh"] = mesh
    _keepalive["section"] = section
    _keepalive["builder"] = builder
    _keepalive["beam_nodes"] = beam_nodes
    _keepalive["vis"] = (vis_speed, vis_wire)
    return mesh, tip_node


def build_visualization(sys):
    """Full Irrlicht scene: window + sky + camera + lights + grid."""
    # === Visualization === standard Irrlicht block (Initialize FIRST, scene after)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEA Beam Cantilever")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, -4.5, 1.0),
                  chrono.ChVector3d(1.5, 0.0, -0.6))
    vis.AddTypicalLights()
    vis.AddGrid(0.25, 0.25, 32, 32,
                chrono.ChCoordsysd(chrono.ChVector3d(1.5, 0.0, -1.2), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))
    return vis


def main():
    sys = build_system()
    mesh, tip_node = build_beam(sys)

    # cache: tip node fetched once, its pose/velocity read every step
    tip = tip_node
    tip_initial_z = tip.GetPos().z   # precomputed once: undeflected tip height

    vis = None
    if not HEADLESS:
        vis = build_visualization(sys)

    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # motion log lives under cam/

    # === Main loop === render-cadence outer loop; physics in inner batch
    csv_file = None
    motion_file = None
    try:
        csv_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(csv_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "tip_x", "tip_y", "tip_z",
                              "tip_deflection", "tip_speed"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "speed"])

        times, deflections, speeds = [], [], []
        frame = 0

        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()
                pos = tip.GetPos()                 # cache: single getter per step
                speed = tip.GetPosDt().Length()    # cache: single getter per step
                deflection = pos.z - tip_initial_z

                data_writer.writerow([f"{t:.6f}", f"{pos.x:.6f}", f"{pos.y:.6f}",
                                      f"{pos.z:.6f}", f"{deflection:.6f}",
                                      f"{speed:.6f}"])
                motion_writer.writerow([f"{t:.6f}", "beam_tip",
                                        f"{pos.x:.6f}", f"{pos.y:.6f}",
                                        f"{pos.z:.6f}", f"{speed:.6f}"])
                times.append(t)
                deflections.append(deflection)
                speeds.append(speed)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= RUN_END:
                    break

    except (OSError, IOError) as exc:            # disk / permission failure on CSV
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:    # solver divergence / bad FEA state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if csv_file is not None:
            csv_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === timeseries plot of tip deflection and speed
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(times, deflections, color="tab:blue")
        ax1.set_ylabel("tip deflection (m)")
        ax1.set_title("FEA cantilever beam — tip response under gravity")
        ax1.grid(True)
        ax2.plot(times, speeds, color="tab:red")
        ax2.set_ylabel("tip speed (m/s)")
        ax2.set_xlabel("time (s)")
        ax2.grid(True)
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as png:
            fig.savefig(png, dpi=110)
        plt.close(fig)

    print(f"Done. nodes={mesh.GetNumNodes()} elements={mesh.GetNumElements()} "
          f"steps_logged={len(times)} final_deflection="
          f"{deflections[-1] if deflections else float('nan'):.5f} m")


if __name__ == "__main__":
    main()
