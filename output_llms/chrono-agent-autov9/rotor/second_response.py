"""Jeffcott flexible rotor — IGA Cosserat beam driven by a rotary motor.

Models a slender flexible shaft as a single IGA (isogeometric) Cosserat beam
(pychrono.fea ChBuilderBeamIGA + ChBeamSectionCosseratEasyCircular). The beam end
nodes are ChNodeFEAxyzrot bodies (they derive from ChBodyFrame), so a
ChLinkMotorRotationAngle attaches DIRECTLY to the root node to spin the shaft. A
rigid flywheel cylinder is clamped to the mid node; the flywheel is offset
laterally from the shaft axis to introduce a mass unbalance, so spinning the shaft
excites a whirling (Jeffcott) lateral vibration of the mid node.

System type: ChSystemSMC (FEA requires SMC + a direct sparse solver). Solver:
Pardiso MKL. Timestepper: linearized implicit Euler (robust on this stiff
motor-driven IGA beam). Gravity acts along -Y at 3.71 m/s^2 (reduced-gravity
environment).

Expected behavior: the shaft is driven to oscillate angularly by a sinusoidal
motor angle; the unbalanced flywheel makes the mid node trace a whirl orbit in the
plane transverse to the shaft axis. CSV logs the motor angle, the mid-node lateral
displacement (whirl) and the flywheel position; a timeseries PNG plots them.
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

# === Named constants === geometry / material / physics (no bare literals downstream)
BEAM_L = 10.0                      # shaft length along X axis (units)
BEAM_RO = 0.060                    # shaft outer radius
BEAM_RI = 0.055                    # shaft inner radius (thin-wall reference)
BEAM_DIAMETER = 2.0 * BEAM_RO      # section diameter used by the IGA section
BEAM_N_ELEMENTS = 12               # IGA beam elements
BEAM_ORDER = 3                     # cubic IGA basis
BEAM_E = 2.0e11                    # Young's modulus (steel shaft)
BEAM_G = 7.9e10                    # shear modulus (steel)
BEAM_DENSITY = 7800.0             # kg/m^3
BEAM_RAYLEIGH_BETA = 0.002         # Cosserat stiffness-proportional damping

FLYWHEEL_RADIUS = 0.30             # flywheel disk radius
FLYWHEEL_THICKNESS = 0.10          # flywheel disk thickness (along its Y axis)
FLYWHEEL_DENSITY = 7800.0         # kg/m^3
FLYWHEEL_UNBALANCE = 0.01          # lateral offset of flywheel COM -> mass unbalance

GRAVITY_Y = -3.71                  # reduced-gravity environment along -Y

MOTOR_AMPL = 60.0                  # sinusoidal motor angle amplitude (rad)
MOTOR_FREQ = 0.1                   # sinusoidal motor angle frequency (Hz)

TIME_STEP = 5.0e-4                 # FEA-stable step
SIM_END = 6.0                      # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

# Derived placement constants (precomputed once)
ROOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)            # shaft root (motor attaches here)
TIP_POS = chrono.ChVector3d(BEAM_L, 0.0, 0.0)          # shaft free end
BEAM_YDIR = chrono.ChVector3d(0.0, 1.0, 0.0)           # section lateral reference
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Strong-reference keepalive list — defeats SWIG GC of FEA temporaries.
KEEPALIVE = []

# Fast, windowless validation run (short, no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


def build_system():
    """Create the SMC system, MKL solver and HHT timestepper for the FEA shaft."""
    # === System & gravity === FEA needs SMC + a direct sparse solver
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, GRAVITY_Y, 0.0))

    # Direct sparse solver (MKL/Pardiso) — iterative solvers diverge on FEA stiffness.
    try:
        import pychrono.pardisomkl as mkl
        sys.SetSolver(mkl.ChSolverPardisoMKL())
    except ImportError as exc:                 # MKL module not present in this build
        print(f"[warn] Pardiso MKL unavailable ({exc}); using default sparse LU")
        sys.SetSolverType(chrono.ChSolver.Type_SPARSE_LU)

    # Linearized implicit Euler timestepper — robust on this stiff, motor-driven
    # IGA beam (HHT stalls at its minimum step size here in 9.0.1).
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    return sys


def build_rotor(sys):
    """Build the IGA Cosserat beam shaft, the unbalanced flywheel, and the motor."""
    # === FEA mesh & IGA Cosserat beam === slender flexible shaft along +X
    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)
    KEEPALIVE.append(mesh)

    # Solid circular Cosserat section sized by the shaft OUTER diameter. The shaft is
    # a thin-walled tube (outer r=0.060, inner r=0.055) — modeled here by its outer
    # diameter; Rayleigh damping is added on the elasticity term.
    section = fea.ChBeamSectionCosseratEasyCircular(
        BEAM_DIAMETER, BEAM_E, BEAM_G, BEAM_DENSITY
    )
    damping = fea.ChDampingCosseratRayleigh(section.GetElasticity(), BEAM_RAYLEIGH_BETA)
    section.SetDamping(damping)
    KEEPALIVE.extend([section, damping])

    builder = fea.ChBuilderBeamIGA()
    builder.BuildBeam(mesh, section, BEAM_N_ELEMENTS, ROOT_POS, TIP_POS, BEAM_YDIR, BEAM_ORDER)
    KEEPALIVE.append(builder)

    # Keep a strong reference to the node container BEFORE indexing (SWIG GC pitfall).
    beam_nodes_container = builder.GetLastBeamNodes()
    n_nodes = beam_nodes_container.size()
    beam_nodes = [beam_nodes_container[i] for i in range(n_nodes)]
    KEEPALIVE.append(beam_nodes_container)
    KEEPALIVE.extend(beam_nodes)

    root_node = beam_nodes[0]                       # motor attaches here
    mid_node = beam_nodes[n_nodes // 2]             # whirl is measured here
    sys.Add(mesh)

    # FEA beam: no contact material needed — driven by constraints + gravity + motor only.

    # === Bodies === rigid flywheel disk clamped to the shaft mid node
    flywheel = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Y, FLYWHEEL_RADIUS, FLYWHEEL_THICKNESS, FLYWHEEL_DENSITY
    )
    # No ChBody COM-frame setter in 9.0.1 -> realize the unbalance by OFFSETTING the
    # flywheel body laterally (+Z) from the shaft axis at the mid node.
    mid_pos = mid_node.GetPos()
    flywheel.SetPos(chrono.ChVector3d(mid_pos.x, mid_pos.y, mid_pos.z + FLYWHEEL_UNBALANCE))
    sys.Add(flywheel)
    KEEPALIVE.append(flywheel)

    # === Joints / constraints === clamp flywheel rigidly to the mid node (all 6 DOF)
    clamp = chrono.ChLinkMateGeneric()
    clamp.Initialize(flywheel, mid_node, False, flywheel.GetFrameRefToAbs(), mid_node.Frame())
    clamp.SetConstrainedCoords(True, True, True, True, True, True)
    clamp.SetName("flywheel_clamp")
    sys.Add(clamp)
    KEEPALIVE.append(clamp)

    # Fixed ground reference for the motor stator.
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(ROOT_POS)
    sys.Add(ground)
    KEEPALIVE.append(ground)

    # === Actuator === rotary motor drives the root node angle sinusoidally about +X.
    # The motor frame X axis must align with the shaft axis, so rotate frame +Z->+X.
    motor = chrono.ChLinkMotorRotationAngle()
    motor_frame = chrono.ChFramed(
        ROOT_POS, chrono.QuatFromAngleY(math.pi / 2.0)   # spin axis along world +X
    )
    motor.Initialize(root_node, ground, motor_frame)
    motor.SetMotorFunction(chrono.ChFunctionSine(MOTOR_AMPL, MOTOR_FREQ))
    motor.SetName("shaft_motor")
    sys.Add(motor)
    KEEPALIVE.append(motor)

    # === FEA visualization === beam speed colormap + undeformed wireframe overlay
    vis_beam = chrono.ChVisualShapeFEA()
    vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_beam.SetColormapRange(chrono.ChVector2d(0.0, 4.0))
    vis_beam.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_beam)
    vis_wire = chrono.ChVisualShapeFEA()
    vis_wire.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_wire.SetWireframe(True)
    vis_wire.SetDrawInUndeformedReference(True)
    mesh.AddVisualShapeFEA(vis_wire)
    KEEPALIVE.extend([vis_beam, vis_wire])

    return mid_node, flywheel, motor


def build_visualization(sys):
    """Full Irrlicht scene: window + sky + camera + lights + grid (Y-up gravity)."""
    # === Visualization ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y here
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Jeffcott flexible rotor — IGA Cosserat beam")
    vis.Initialize()                                    # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(BEAM_L * 0.5, 0, 0))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, -1.0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))
    return vis


def main():
    sys = build_system()
    mid_node, flywheel, motor = build_rotor(sys)

    # cache: fetch repeated handles once, reused every step (avoid per-step getters)
    mid_node_ref = mid_node          # cache: whirl-measurement node
    flywheel_ref = flywheel          # cache: flywheel body
    motor_ref = motor                # cache: actuator
    # precomputed once: rest position copied to plain floats (avoid SWIG ref aliasing)
    _p0 = mid_node_ref.GetPos()
    mid_y0, mid_z0 = float(_p0.y), float(_p0.z)

    os.makedirs("frames", exist_ok=True)   # guard against missing output dir

    vis = None
    if not HEADLESS:
        vis = build_visualization(sys)

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short check when validating

    # === Main loop === render-cadence outer loop; physics in inner batch; log every step
    csv_file = None
    try:
        csv_file = open("simulation_data.csv", "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "time", "motor_angle",
            "mid_dy", "mid_dz", "whirl_radius",
            "flywheel_x", "flywheel_y", "flywheel_z",
        ])

        def log_row():
            """Write one CSV row of motor angle + mid-node whirl + flywheel pose."""
            t = sys.GetChTime()
            p = mid_node_ref.GetPos()
            fp = flywheel_ref.GetPos()
            dy = p.y - mid_y0
            dz = p.z - mid_z0
            whirl = math.hypot(dy, dz)
            writer.writerow([t, motor_ref.GetMotorAngle(), dy, dz, whirl, fp.x, fp.y, fp.z])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1
            for _ in range(RENDER_EVERY):
                log_row()
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break

    except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"[error] simulation step failed: {exc}")
        raise
    finally:
        if csv_file is not None:
            csv_file.close()                       # flush partial CSV even on divergence

    # === Post-processing === plot motor angle + whirl from the CSV
    try:
        data = np.genfromtxt("simulation_data.csv", delimiter=",", names=True)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(data["time"], data["motor_angle"], color="tab:blue")
        ax1.set_ylabel("motor angle (rad)")
        ax1.grid(True)
        ax1.set_title("Jeffcott rotor — motor drive & mid-node whirl")
        ax2.plot(data["time"], data["mid_dy"], label="mid dy", color="tab:green")
        ax2.plot(data["time"], data["mid_dz"], label="mid dz", color="tab:red")
        ax2.plot(data["time"], data["whirl_radius"], label="whirl radius", color="tab:orange")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("lateral disp (m)")
        ax2.legend()
        ax2.grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
    except (OSError, IOError, ValueError) as exc:   # missing/short CSV or plot I/O
        print(f"[warn] could not write timeseries plot: {exc}")


if __name__ == "__main__":
    main()
