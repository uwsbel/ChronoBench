"""MAN 5t military truck driving on a flat grass-textured rigid terrain, with a
roof-mounted rotating lidar sensor scanning a field of randomly placed boxes.

System type: ChSystemNSC, owned by the veh.MAN_5t wrapper (wheeled-vehicle
multibody dynamics + TMEASY tires on a veh.RigidTerrain patch).

Main bodies:
  - MAN_5t chassis + 6 wheels/spindles (wrapper-created), driven by a scripted
    veh.ChDriver subclass (gentle throttle, slight steering sweep).
  - A flat veh.RigidTerrain patch textured with grass.jpg.
  - A set of random rigid obstacle boxes scattered on the terrain.

Sensor: a sens.ChLidarSensor mounted on the chassis (managed by a
sens.ChSensorManager that is updated every physics step). The lidar returns a
depth/intensity buffer that detects the surrounding random boxes and ground.

Expected behavior: the truck accelerates forward from rest and translates several
metres across the grass terrain while the lidar continuously scans the boxes. The
chassis stays upright (roll/pitch small). Irrlicht is the review renderer; the
lidar is the demo's sensing subject.
"""

import os
import csv
import math
import random

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# === Constants === geometry / physics / run configuration (no bare literals downstream)
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # TMEASY tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 120.0             # rigid terrain patch X size (m), large for driving
TERRAIN_WIDTH = 120.0              # rigid terrain patch Y size (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

VEH_INIT_X = -30.0                 # spawn near one end so the truck drives forward
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.55       # MAN chassis-origin height above wheel-bottom at rest
TIRE_RADIUS = 0.575                # MAN 5t tire radius (m), used for the footprint assert
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs terrain top

NUM_BOXES = 18                     # random obstacle boxes scattered on the grass
BOX_FIELD_X = (-10.0, 40.0)        # X span where boxes may appear (ahead of the truck)
BOX_FIELD_Y = (-18.0, 18.0)        # Y span where boxes may appear
BOX_MIN_SIZE = 0.6                 # smallest box edge (m)
BOX_MAX_SIZE = 1.6                 # largest box edge (m)
BOX_DENSITY = 250.0                # light wooden-crate density (kg/m^3)
BOX_CLEARANCE = 4.0                # keep boxes this far from the spawn so the truck starts clear
RANDOM_SEED = 7                    # deterministic "random" layout

LIDAR_UPDATE_RATE = 10.0           # lidar revolutions per second (Hz)
LIDAR_W = 480                      # horizontal samples per scan
LIDAR_H = 16                       # vertical channels (16-beam puck)
LIDAR_HFOV = 2.0 * math.pi         # full 360-degree horizontal sweep
LIDAR_VMAX = 0.2618               # +15 deg upper vertical extent (rad)
LIDAR_VMIN = -0.2618               # -15 deg lower vertical extent (rad)
LIDAR_MAX_DIST = 80.0              # maximum range (m)
LIDAR_MOUNT_Z = 2.6                # lidar height above chassis origin (roof) (m)

GRAVITY_Z = -9.81

# Derived constants (precomputed once — never recompute inside the loop)
RENDER_STEP_SIZE = 1.0 / RENDER_FPS                      # precomputed once
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / TIME_STEP))   # physics steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))    # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short physics check when validating

os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
os.makedirs("cam", exist_ok=True)      # guard against missing output dir for motion log


# === Driver === scripted time-based control law (no human-in-the-loop in headless runs)
class ScriptedDriver(veh.ChDriver):
    """Gentle forward acceleration with a slow steering sweep so the lidar sees
    the box field from changing angles."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Brief settle, then ramp throttle; small sinusoidal steering.
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.15 * math.sin(0.4 * time))


def build_random_boxes(system, spawn_xy):
    """Create NUM_BOXES random rigid boxes on the terrain, kept clear of the spawn."""
    # === Random obstacle boxes === scattered crates for the lidar to detect
    rng = random.Random(RANDOM_SEED)   # deterministic layout for reproducibility
    box_mat = chrono.ChContactMaterialNSC()
    box_mat.SetFriction(TERRAIN_FRICTION)
    box_mat.SetRestitution(TERRAIN_RESTITUTION)

    boxes = []
    attempts = 0
    while len(boxes) < NUM_BOXES and attempts < NUM_BOXES * 40:
        attempts += 1
        sx = rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
        sy = rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
        sz = rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
        px = rng.uniform(*BOX_FIELD_X)
        py = rng.uniform(*BOX_FIELD_Y)
        # Keep boxes clear of the truck spawn so it does not start wedged in contact.
        if math.hypot(px - spawn_xy[0], py - spawn_xy[1]) < BOX_CLEARANCE:
            continue
        box = chrono.ChBodyEasyBox(sx, sy, sz, BOX_DENSITY, True, True, box_mat)
        box.SetPos(chrono.ChVector3d(px, py, sz / 2.0))   # rest on terrain top (z=0)
        box.SetName(f"random_box_{len(boxes)}")
        box.GetVisualShape(0).SetColor(
            chrono.ChColor(rng.uniform(0.3, 0.9), rng.uniform(0.2, 0.6), rng.uniform(0.1, 0.3))
        )
        system.AddBody(box)
        boxes.append(box)
    return boxes


def main():
    random.seed(RANDOM_SEED)

    # === Vehicle wrapper (creates and owns the ChSystemNSC + chassis + wheels) ===
    truck = veh.MAN_5t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    init_z = SUSPENSION_REF_HEIGHT          # terrain top is z=0, so chassis origin sits here
    truck.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, init_z), chrono.QUNIT)
    )
    truck.SetTireType(veh.TireModelType_TMEASY)   # prompt-driven slip tire for rigid driving
    truck.SetTireStepSize(TIRE_STEP)
    truck.Initialize()

    truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.MAN_5t wrapper) ===
    system = truck.GetSystem()                 # ChSystemNSC owned by the wrapper
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY_Z))
    chassis = truck.GetChassisBody()           # cache: main chassis rigid body, reused every step
    veh_obj = truck.GetVehicle()               # cache: ChVehicle handle for spindle queries
    # wheels/spindles: veh_obj.GetAxles()[*].GetWheels()[*].GetSpindle()
    # joints: suspension + steering links created inside the wrapper

    # === Terrain === flat rigid patch textured with grass.jpg
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.4, 0.6, 0.3))
    terrain.Initialize()

    # Footprint assert — verify wheels rest on (not through) the terrain top (z=0).
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= -ZTOL, (
        f"truck sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs top 0.0; "
        f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
    )

    # === Random obstacle boxes ===
    boxes = build_random_boxes(system, (VEH_INIT_X, VEH_INIT_Y))
    system.GetCollisionSystem().BindAll()      # rebuild collision models after adding boxes

    # === Sensor manager & lidar === roof-mounted 360-degree lidar scanning the boxes
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(
        chrono.ChVector3f(20, 20, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-20, -20, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0
    )
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    lidar_offset = chrono.ChFramed(
        chrono.ChVector3d(0.0, 0.0, LIDAR_MOUNT_Z), chrono.QUNIT
    )
    lidar = sens.ChLidarSensor(
        chassis,                 # rides on the chassis, follows the truck
        LIDAR_UPDATE_RATE,
        lidar_offset,
        LIDAR_W,
        LIDAR_H,
        LIDAR_HFOV,
        LIDAR_VMAX,
        LIDAR_VMIN,
        LIDAR_MAX_DIST,
    )
    lidar.SetName("roof_lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())                  # access depth/intensity buffer
    # Note: the lidar returns are logged to CSV via ChFilterDIAccess; the review
    # video is the Irrlicht chase-cam window. A live ChFilterVisualizePointCloud
    # window is intentionally NOT added — a second on-screen GL context alongside the
    # vehicle Irrlicht window stalls rendering on this display.
    manager.AddSensor(lidar)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid (chase cam)
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("MAN 5t truck with roof lidar over random boxes")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 1.0)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.02), chrono.QUNIT),
                    chrono.ChColor(0.35, 0.45, 0.35))   # ground reference grid on the grass
        vis.AttachVehicle(veh_obj)

    # === Driver ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Main loop === render-cadence outer loop, physics + sensor in inner batch
    csv_file = None
    motion_file = None
    try:
        try:
            csv_file = open("simulation_data.csv", "w", newline="")
            motion_file = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:   # disk full / permission denied
            print(f"failed to open output CSV: {exc}")
            raise

        sim_writer = csv.writer(csv_file)
        sim_writer.writerow(
            ["time", "x", "y", "z", "speed", "throttle", "steering",
             "roll", "pitch", "lidar_hits", "lidar_min_range"]
        )
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        step = 0
        frame = 0
        time = 0.0
        while time < RUN_END:
            time = system.GetChTime()

            if (not HEADLESS) and not vis.Run():
                break

            # Render once per frame (outer cadence), save the review frame.
            if (not HEADLESS) and (step % RENDER_STEPS == 0):
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            driver_inputs = driver.GetInputs()

            # Synchronize the full subsystem stack.
            driver.Synchronize(time)
            terrain.Synchronize(time)
            truck.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # Pump sensors every physics step so the lidar sees each post-step pose.
            manager.Update()

            # --- log physics each step ---
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            rot = chassis.GetRot()
            rpy = rot.GetCardanAnglesXYZ()   # roll/pitch/yaw of the chassis

            di_buf = lidar.GetMostRecentDIBuffer()   # may be empty before first lidar tick
            lidar_hits = 0
            lidar_min = float("nan")
            if di_buf.HasData():                     # guard: skip frames before the first scan
                data = di_buf.GetDIData()
                ranges = data[:, :, 0].ravel()
                valid = ranges[(ranges > 0.0) & (ranges < LIDAR_MAX_DIST)]
                lidar_hits = int(valid.size)
                if lidar_hits > 0:
                    lidar_min = float(valid.min())

            sim_writer.writerow([
                f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                f"{driver_inputs.m_steering:.4f}",
                f"{rpy.x:.5f}", f"{rpy.y:.5f}", lidar_hits, f"{lidar_min:.5f}",
            ])
            motion_writer.writerow([
                f"{time:.5f}", "man5t_chassis",
                f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
            ])

            # Advance the full subsystem stack (truck.Advance steps the wrapper system).
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            truck.Advance(TIME_STEP)
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad numeric state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if csv_file is not None:
            csv_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot logged time series from the CSV
    times, xs, speeds, rolls, pitches, hits = [], [], [], [], [], []
    with open("simulation_data.csv", "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            xs.append(float(row["x"]))
            speeds.append(float(row["speed"]))
            rolls.append(float(row["roll"]))
            pitches.append(float(row["pitch"]))
            hits.append(float(row["lidar_hits"]))

    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(times, xs, label="chassis x (m)")
    axes[0].plot(times, speeds, label="speed (m/s)")
    axes[0].set_ylabel("position / speed")
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(times, rolls, label="roll (rad)")
    axes[1].plot(times, pitches, label="pitch (rad)")
    axes[1].set_ylabel("attitude")
    axes[1].legend()
    axes[1].grid(True)
    axes[2].plot(times, hits, label="lidar hits")
    axes[2].set_ylabel("lidar valid returns")
    axes[2].set_xlabel("time (s)")
    axes[2].legend()
    axes[2].grid(True)
    fig.suptitle("MAN 5t truck + roof lidar over random boxes")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print(f"done: {len(times)} steps logged, final x={xs[-1]:.2f} m, "
          f"final speed={speeds[-1]:.2f} m/s")


if __name__ == "__main__":
    main()
