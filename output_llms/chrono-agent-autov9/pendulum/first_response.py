"""Simple pendulum simulation (PyChrono, NSC system, Irrlicht visualization).

Models a single rigid pendulum arm hinged to a fixed ground body by a revolute
joint. The arm starts horizontal (along +X) and swings in the world XZ plane
under gravity (-Z), so the physical hinge axis is world +Y. The arm has an
explicit mass and inertia tensor. The script periodically logs the pendulum tip
position and the arm's angular/linear velocity to CSV and renders the motion
with a standard Irrlicht window.

Expected behavior: the arm released from horizontal swings down through the
bottom, overshoots to the far side, and oscillates about the vertical with
(near-)conserved mechanical energy (no joint damping, no contact).
"""

# === Imports ===
import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib
matplotlib.use("Agg")          # headless-safe backend for the timeseries plot
import matplotlib.pyplot as plt

# === Named constants (geometry / physics) ===
time_step = 1.0e-3             # integration step [s]
sim_end = 5.0                  # total simulated time [s]
render_fps = 30.0              # review-video frame rate [frames/s]
gravity_z = -9.81             # gravity along -Z [m/s^2]

arm_length = 1.0              # pendulum arm length [m]
arm_radius = 0.04             # arm cylinder visual radius [m]
arm_mass = 1.0                # pendulum arm mass [kg]
pivot_radius = 0.06           # fixed pivot sphere visual radius [m]

# Inertia of a slender rod about its center (length along body-local X).
# I_axial is about the rod axis; I_transverse about the two perpendicular axes.
inertia_axial = 0.5 * arm_mass * arm_radius ** 2                       # precomputed once
inertia_transverse = (1.0 / 12.0) * arm_mass * arm_length ** 2        # precomputed once

# Derived placement: pivot at world origin, arm extends along +X, COM at L/2.
pivot_pos = chrono.ChVector3d(0, 0, 0)                                 # precomputed once
arm_com = chrono.ChVector3d(arm_length / 2.0, 0, 0)                    # precomputed once

# Render cadence: render once per frame, batch physics in between.
render_every = max(1, round(1.0 / (render_fps * time_step)))          # precomputed once

# Headless validation gate: skip the window for fast, parallel-safe checks.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))                   # windowless run flag

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, gravity_z))
# Pure MBS (joint only, no contact) -> do NOT set a collision system.

# === Bodies ===
# Fixed ground body carrying a small sphere visual at the pivot.
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(pivot_pos)
pivot_vis = chrono.ChVisualShapeSphere(pivot_radius)
pivot_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(pivot_vis)
sys.AddBody(ground)

# Pendulum arm: manual ChBody (orientation matters) with explicit mass/inertia.
arm = chrono.ChBody()
arm.SetMass(arm_mass)
arm.SetInertiaXX(chrono.ChVector3d(inertia_axial, inertia_transverse, inertia_transverse))
arm.SetPos(arm_com)
arm.SetRot(chrono.QUNIT)       # body-local X already aligned with world X (arm direction)
arm.EnableCollision(False)
# Cylinder visual: default axis is body-local Z; rotate Z->X so it spans the arm.
arm_vis = chrono.ChVisualShapeCylinder(arm_radius, arm_length)
arm_vis.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
arm.AddVisualShape(arm_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(arm)

# === Joints / constraints ===
# Revolute hinge at the pivot. Swing is in the XZ plane (gravity -Z), so the
# physical hinge axis is world +Y; map joint local +Z onto +Y (Rule 8 special case).
hinge_axis_quat = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(arm, ground, chrono.ChFramed(pivot_pos, hinge_axis_quat))
sys.AddLink(revolute)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up; BEFORE Initialize
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Simple Pendulum")
    vis.Initialize()                                    # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -3.0, 0.6), chrono.ChVector3d(0, 0, -0.2))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop, physics inner batch, CSV logging
run_end = min(sim_end, 0.5) if HEADLESS else sim_end    # short bounded check when validating
os.makedirs("frames", exist_ok=True)                    # guard against missing output dir
os.makedirs("cam", exist_ok=True)                       # motion log lives under cam/

data_file = None
motion_file = None
try:
    data_file = open("simulation_data.csv", "w", newline="")        # main physics log
    motion_file = open("cam/motion_log.csv", "w", newline="")       # per-body motion contract
    data_writer = csv.DictWriter(
        data_file, fieldnames=["time", "angle_rad", "tip_x", "tip_z", "omega_y", "energy"])
    motion_writer = csv.DictWriter(
        motion_file, fieldnames=["time", "body", "pos_x", "pos_y", "pos_z", "vx", "vy", "vz"])
    data_writer.writeheader()
    motion_writer.writeheader()

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")     # consecutive index -> ffmpeg
            frame += 1
        for _ in range(render_every):
            t = sys.GetChTime()
            com = arm.GetPos()                  # cache: arm COM fetched once this step
            vel = arm.GetPosDt()                # cache: arm COM linear velocity
            wvel = arm.GetAngVelParent()        # cache: arm angular velocity (world)
            # Tip position = pivot + 2*(COM - pivot) since COM is at the arm midpoint.
            tip_x = 2.0 * com.x
            tip_z = 2.0 * com.z
            angle = math.atan2(com.z, com.x)    # arm angle from +X in the XZ plane
            # Mechanical energy: KE (trans+rot about Y) + PE (gravity along -Z).
            ke = 0.5 * arm_mass * (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) \
                + 0.5 * inertia_transverse * wvel.y ** 2
            pe = arm_mass * (-gravity_z) * com.z
            energy = ke + pe
            data_writer.writerow({
                "time": t, "angle_rad": angle, "tip_x": tip_x, "tip_z": tip_z,
                "omega_y": wvel.y, "energy": energy})
            motion_writer.writerow({
                "time": t, "body": "arm", "pos_x": com.x, "pos_y": com.y, "pos_z": com.z,
                "vx": vel.x, "vy": vel.y, "vz": vel.z})
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= run_end:
                break
except (OSError, IOError) as exc:                    # disk / permission failure on CSV
    import traceback; traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:            # solver divergence / bad numeric state
    import traceback; traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot the logged timeseries to a PNG
times, angles, energies, omegas = [], [], [], []
with open("simulation_data.csv", "r", newline="") as f:
    for row in csv.DictReader(f):
        times.append(float(row["time"]))
        angles.append(float(row["angle_rad"]))
        energies.append(float(row["energy"]))
        omegas.append(float(row["omega_y"]))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
ax1.plot(times, angles, label="angle [rad]")
ax1.plot(times, omegas, label="omega_y [rad/s]")
ax1.set_ylabel("angle / rate")
ax1.legend(); ax1.grid(True)
ax2.plot(times, energies, color="tab:green", label="mechanical energy [J]")
ax2.set_xlabel("time [s]"); ax2.set_ylabel("energy [J]")
ax2.legend(); ax2.grid(True)
fig.suptitle("Simple pendulum — angle, angular rate, energy")
fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=120)
plt.close(fig)

print(f"Done: simulated to t={sys.GetChTime():.3f}s, wrote simulation_data.csv + cam/motion_log.csv")
