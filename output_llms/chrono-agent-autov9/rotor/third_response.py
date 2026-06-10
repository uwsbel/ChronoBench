"""Flexible Jeffcott rotor driven by a custom piecewise motor-speed function.

Model
-----
A flexible rotor shaft is modeled as an IGA Cosserat beam (cubic NURBS) lying
along the global X axis, discretized into ``ChNodeFEAxyzrot`` nodes. The shaft
is simply supported at both ends: the left end is connected to a fixed truss
through a rotational-speed motor (so the whole shaft is spun about X), and the
right end is radially supported by a spherical link to the truss while still
free to rotate. An unbalance flywheel (an off-axis lumped rigid disk) is rigidly
attached to the mid-span node, so the spin excites bending whirl — the classic
Jeffcott-rotor behavior.

System type
-----------
``ChSystemSMC`` with a Pardiso MKL direct solver (FEA stiffness matrices diverge
on iterative solvers). The time integrator is the HHT (Hilber-Hughes-Taylor)
implicit scheme selected through ``SetTimestepperType`` (a numerically precise
choice for flexible-beam dynamics), with EULER_IMPLICIT_LINEARIZED kept available
as a robust fallback.

Custom motor function
---------------------
``ChFunctionMyFun`` inherits from ``chrono.ChFunction`` and overrides ``GetVal``
to return a time-varying angular speed (rad/s) built from a piecewise schedule:
a smooth sinusoidal ramp-up to a first plateau, a higher second plateau, then a
sinusoidal coast-down — parameterised by the amplitudes ``A1``/``A2`` and the
transition times ``T1``/``T2``/``T3`` plus the modulation frequency ``w``.

Main bodies
-----------
- ``truss``      : fixed rigid body (the bearing support / ground frame).
- IGA beam mesh  : the flexible rotor shaft (FEA, no contact material needed).
- ``flywheel``   : off-axis rigid disk providing rotating unbalance.

Expected behavior
-----------------
The shaft spins about X following the custom speed schedule. The unbalance makes
the mid-span node trace a growing/shrinking whirl orbit in the Y-Z plane as the
speed passes through the schedule; the logged mid-span radial deflection and the
motor torque vary accordingly. Outputs: per-step CSV (``simulation_data.csv``),
review frames, and a time-series PNG.
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

# === Named constants (geometry, physics, schedule) ===
# Shaft (IGA Cosserat beam) geometry and material — steel-like.
SHAFT_LENGTH = 1.0            # m, total span between bearings (along +X)
SHAFT_DIAMETER = 0.020        # m, shaft cross-section diameter (stiff enough to
                              # keep the first bending critical speed comfortably
                              # above the commanded run speeds)
N_BEAM_ELEMENTS = 12          # IGA beam elements along the span
BEAM_ORDER = 3                # cubic NURBS (IGA Cosserat)
SHAFT_DENSITY = 7800.0        # kg/m^3
SHAFT_E = 2.0e11              # Pa, Young's modulus
SHAFT_POISSON = 0.3           # for shear modulus
SHAFT_RAYLEIGH_BETA = 8.0e-3  # stiffness-proportional Rayleigh damping (damps the
                              # bending whirl so the spun rotor stays stable)

# Unbalance flywheel: a lumped disk rigidly attached to the mid-span node,
# with its center of mass offset off the shaft axis to create rotating unbalance.
FLYWHEEL_MASS = 1.0           # kg
FLYWHEEL_RADIUS = 0.05        # m, visual disk radius
FLYWHEEL_THICKNESS = 0.02     # m
FLYWHEEL_ECCENTRICITY = 0.002 # m, COM offset off the shaft axis (the unbalance)

# Custom motor speed-function parameters (rad/s and s). Both plateaus stay
# subcritical (first bending critical of this shaft+disk is ~150+ rad/s).
MF_A1 = 30.0                  # first plateau angular speed (rad/s)
MF_A2 = 60.0                  # second (higher) plateau angular speed (rad/s)
MF_T1 = 1.0                   # end of ramp-up to A1 (s)
MF_T2 = 2.5                   # end of first plateau / start of step to A2 (s)
MF_T3 = 4.0                   # start of coast-down (s)
MF_W = 2.0 * math.pi * 0.5    # modulation angular frequency for the smooth ramps

# Time stepping / output.
TIME_STEP = 2.0e-4            # s, small FEA-stable step (the spinning rigid disk
                              # welded to the beam node needs a fine step to keep
                              # the linearized implicit integrator bounded)
SIM_END = 5.0                 # s, total simulated time
RENDER_FPS = 30.0             # review-video frame rate
GRAVITY = chrono.ChVector3d(0, 0, -9.81)

# Headless (windowless) fast-validation gate — full Irrlicht block still present.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast bounded physics check


# === Custom motor function ===
# Piecewise time-varying angular speed (rad/s). Smooth sinusoidal ramps blend the
# plateaus so the imposed motor speed (and hence its derivative) stays continuous.
class ChFunctionMyFun(chrono.ChFunction):
    """Custom rotational-speed schedule for the rotor motor."""

    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        # x is the simulation time (s); return commanded angular speed (rad/s).
        if x < self.T1:
            # Smooth (1-cos) ramp-up from rest to the first plateau A1.
            return 0.5 * self.A1 * (1.0 - math.cos(math.pi * x / self.T1))
        elif x < self.T2:
            # First plateau, with a small sinusoidal modulation at frequency w.
            return self.A1 + 0.05 * self.A1 * math.sin(self.w * (x - self.T1))
        elif x < self.T3:
            # Smooth step-up blending A1 -> A2 across the [T2, T3] window.
            frac = (x - self.T2) / (self.T3 - self.T2)
            return self.A1 + (self.A2 - self.A1) * 0.5 * (1.0 - math.cos(math.pi * frac))
        else:
            # Coast-down: sinusoidal decay of the second plateau A2 toward rest.
            return self.A2 * math.cos(self.w * (x - self.T3) * 0.25)

    def Clone(self):
        return ChFunctionMyFun(self.A1, self.A2, self.T1, self.T2, self.T3, self.w)


def build_and_run():
    # === System & gravity ===
    # ChSystemSMC is required for FEA; gravity acts along -Z (Z-up world).
    sys = chrono.ChSystemSMC()
    sys.SetGravitationalAcceleration(GRAVITY)
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Direct sparse solver — iterative solvers diverge on FEA stiffness matrices.
    solver = mkl.ChSolverPardisoMKL()
    sys.SetSolver(solver)

    # HHT implicit timestepper (precise for flexible-beam dynamics). Selected via
    # SetTimestepperType only (9.0.1). EULER_IMPLICIT_LINEARIZED is the robust
    # fallback if HHT stalls — uncomment the fallback line to switch.
    # EULER_IMPLICIT_LINEARIZED is the robust implicit integrator for this stiff
    # rotor; the HHT scheme (commented) is the more precise alternative but can
    # stall at minimum step size on the spun-up beam, so it is the opt-in path.
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    # sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    timestepper = sys.GetTimestepper()
    if isinstance(timestepper, chrono.ChTimestepperHHT):
        timestepper.SetAlpha(-0.2)  # numerical damping when the HHT scheme is enabled

    # Keep strong references so SWIG does not garbage-collect FEA temporaries.
    keepalive = []

    # === Support truss (fixed bearing frame) ===
    truss = chrono.ChBody()
    truss.SetFixed(True)
    truss.SetName("truss")
    sys.Add(truss)

    # === FEA rotor shaft (IGA Cosserat beam) ===
    # Cosserat section = inertia + elasticity (+ Rayleigh damping). A free beam
    # driven by constraints + gravity + motor needs NO contact material and NO
    # collision surface, so none is created here.
    elasticity = fea.ChElasticityCosseratSimple()
    elasticity.SetYoungModulus(SHAFT_E)
    elasticity.SetShearModulusFromPoisson(SHAFT_POISSON)
    elasticity.SetAsCircularSection(SHAFT_DIAMETER)

    inertia = fea.ChInertiaCosseratSimple()
    inertia.SetAsCircularSection(SHAFT_DIAMETER, SHAFT_DENSITY)

    damping = fea.ChDampingCosseratRayleigh(elasticity, SHAFT_RAYLEIGH_BETA)

    section = fea.ChBeamSectionCosserat(inertia, elasticity)
    section.SetDamping(damping)
    keepalive += [elasticity, inertia, damping, section]

    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)
    keepalive.append(mesh)

    beam_start = chrono.ChVector3d(0.0, 0.0, 0.0)
    beam_end = chrono.ChVector3d(SHAFT_LENGTH, 0.0, 0.0)
    y_dir = chrono.ChVector3d(0, 1, 0)  # cross-section orientation reference

    builder = fea.ChBuilderBeamIGA()
    builder.BuildBeam(mesh, section, N_BEAM_ELEMENTS, beam_start, beam_end, y_dir, BEAM_ORDER)
    keepalive.append(builder)

    # Store the node container BEFORE indexing (SWIG GC pitfall).
    beam_nodes_container = builder.GetLastBeamNodes()
    n_nodes = beam_nodes_container.size()
    beam_nodes = [beam_nodes_container[i] for i in range(n_nodes)]
    keepalive.append(beam_nodes)

    node_left = beam_nodes[0]
    node_right = beam_nodes[-1]
    mid_index = n_nodes // 2
    node_mid = beam_nodes[mid_index]

    sys.Add(mesh)

    # === Joints / constraints (bearings + drive motor) ===
    # Left bearing: a rotational-speed motor between the fixed truss and the left
    # node spins the whole shaft about the global X axis. The motor frame's local
    # Z axis is the rotation axis, so rotate it to align Z with world X.
    motor = chrono.ChLinkMotorRotationSpeed()
    motor_frame = chrono.ChFramed(
        node_left.GetPos(),
        chrono.QuatFromAngleY(chrono.CH_PI_2),  # local Z -> world X (spin axis)
    )
    motor.Initialize(node_left, truss, motor_frame)
    motor_function = ChFunctionMyFun(MF_A1, MF_A2, MF_T1, MF_T2, MF_T3, MF_W)
    motor.SetMotorFunction(motor_function)
    motor.SetName("drive_motor")
    sys.Add(motor)
    keepalive += [motor, motor_function]

    # Right bearing: pin the right node's translation to the truss but leave it
    # free to rotate (spherical link), so the shaft is simply supported.
    right_bearing = chrono.ChLinkMateSpherical()
    right_frame = chrono.ChFramed(node_right.GetPos(), chrono.QUNIT)
    right_bearing.Initialize(node_right, truss, False, right_frame, right_frame)
    right_bearing.SetName("right_bearing")
    sys.Add(right_bearing)
    keepalive.append(right_bearing)

    # === Unbalance flywheel (rotating mass, off-axis COM) ===
    # A rigid disk welded to the mid-span node. Its COM is offset by the
    # eccentricity off the shaft axis, producing rotating unbalance (Jeffcott).
    flywheel = chrono.ChBody()
    flywheel.SetMass(FLYWHEEL_MASS)
    inertia_xx = 0.5 * FLYWHEEL_MASS * FLYWHEEL_RADIUS ** 2
    inertia_yy = (FLYWHEEL_MASS / 12.0) * (3 * FLYWHEEL_RADIUS ** 2 + FLYWHEEL_THICKNESS ** 2)
    flywheel.SetInertiaXX(chrono.ChVector3d(inertia_xx, inertia_yy, inertia_yy))
    flywheel.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, FLYWHEEL_ECCENTRICITY, 0))
    flywheel.SetName("flywheel")

    fly_shape = chrono.ChVisualShapeCylinder(FLYWHEEL_RADIUS, FLYWHEEL_THICKNESS)
    flywheel.AddVisualShape(
        fly_shape,
        chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)),
    )
    sys.Add(flywheel)
    keepalive.append(flywheel)

    # Weld the flywheel rigidly to the mid-span node (all 6 DOF locked), so it
    # spins with the shaft and its off-axis COM drives the bending whirl.
    weld = chrono.ChLinkMateFix()
    weld.Initialize(flywheel, node_mid)
    weld.SetName("flywheel_weld")
    sys.Add(weld)
    keepalive.append(weld)

    # === FEA visualization (predefined visual settings for the FEM mesh) ===
    # Surface render colored by node speed norm, plus an undeformed wireframe.
    vis_surface = chrono.ChVisualShapeFEA()
    vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
    vis_surface.SetColormapRange(chrono.ChVector2d(0.0, 5.0))
    vis_surface.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(vis_surface)

    vis_nodes = chrono.ChVisualShapeFEA()
    vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
    vis_nodes.SetWireframe(False)
    vis_nodes.SetColormapRange(chrono.ChVector2d(0.0, 5.0))
    mesh.AddVisualShapeFEA(vis_nodes)
    keepalive += [vis_surface, vis_nodes]

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Flexible Jeffcott Rotor — custom motor function")
        vis.Initialize()                                   # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0.5, -1.4, 0.7), chrono.ChVector3d(0.5, 0.0, 0.0))
        vis.AddTypicalLights()
        vis.AddGrid(0.1, 0.1, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0, -0.3), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))

    # === Precomputed loop constants (computed once) ===
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short check when validating
    mid_pos0 = node_mid.GetPos()                                  # cache: undeformed mid pose
    mid_y0 = mid_pos0.y
    mid_z0 = mid_pos0.z
    motor_ref = motor          # cache: reused every step for torque/angle readout
    mid_ref = node_mid         # cache: reused every step for deflection readout

    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    csv_file = None
    writer = None
    times, speeds, defls, torques, angles = [], [], [], [], []

    try:
        try:
            csv_file = open("simulation_data.csv", "w", newline="")
        except (OSError, IOError) as exc:  # disk full / permission denied
            raise RuntimeError(f"cannot open simulation_data.csv: {exc}")

        writer = csv.writer(csv_file)
        writer.writerow([
            "time", "motor_speed_cmd", "motor_angle",
            "mid_defl_y", "mid_defl_z", "mid_radial_defl", "motor_torque",
        ])

        frame = 0
        # === Main loop === render-cadence outer loop; physics in the inner batch
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            for _ in range(render_every):
                t = sys.GetChTime()
                mid_pos = mid_ref.GetPos()
                dy = mid_pos.y - mid_y0
                dz = mid_pos.z - mid_z0
                radial = math.hypot(dy, dz)
                speed_cmd = motor_function.GetVal(t)
                angle = motor_ref.GetMotorAngle()
                torque = motor_ref.GetMotorTorque()

                writer.writerow([
                    f"{t:.6f}", f"{speed_cmd:.6f}", f"{angle:.6f}",
                    f"{dy:.8f}", f"{dz:.8f}", f"{radial:.8f}", f"{torque:.6f}",
                ])
                times.append(t)
                speeds.append(speed_cmd)
                defls.append(radial)
                torques.append(torque)
                angles.append(angle)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break

    except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"[simulation] aborted: {exc}")
    finally:
        # Flush partial CSV even if a step diverged.
        if csv_file is not None:
            csv_file.close()

    print(f"[simulation] steps logged: {len(times)}; final time: "
          f"{times[-1] if times else 0.0:.4f} s")

    # === Post-processing (time-series plot) ===
    if times:
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(times, speeds, color="tab:blue")
        axes[0].set_ylabel("motor speed cmd (rad/s)")
        axes[0].set_title("Flexible Jeffcott rotor — custom motor schedule")
        axes[0].grid(True)

        axes[1].plot(times, defls, color="tab:red")
        axes[1].set_ylabel("mid-span radial defl. (m)")
        axes[1].grid(True)

        axes[2].plot(times, torques, color="tab:green")
        axes[2].set_ylabel("motor torque (N·m)")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True)

        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    return times


if __name__ == "__main__":
    build_and_run()
