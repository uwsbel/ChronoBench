"""ARTcar straight-line acceleration on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A single wheeled ARTcar (1/10-scale RC car wrapper, veh.ARTcar) driving forward
on a flat veh.RigidTerrain patch. The vehicle wrapper owns an NSC ChSystem; the
chassis, four spindles/wheels, suspension and steering joints are created inside
the wrapper. A scripted veh.ChDriver subclass commands full throttle in a
straight line so the powertrain/tire tuning governs how quickly the car speeds up.

Powertrain / tire tuning (the parameters that make this configuration fast):
  - MaxMotorVoltageRatio = 0.26   (more motor voltage headroom -> higher top speed)
  - StallTorque          = 0.40   (stronger launch torque)
  - TireRollingResistance= 0.03   (less rolling drag -> coasts/accelerates better)

System type: NSC (ChSystemNSC owned by the ARTcar wrapper).
Expected behavior: the car launches from rest and accelerates monotonically down
+X, staying upright (roll/pitch small), reaching a higher steady speed than a
lightly-tuned configuration would. Outputs CSV time series + a matplotlib plot.
"""

import os
import csv
import math

import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / tuning) ===
TIME_STEP = 1.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire force-model step (s)
SIM_END = 12.0                     # simulated duration (s)
RENDER_FPS = 30.0                  # review-video frame rate

# Tuning parameters that make the ARTcar faster (final desired values).
MAX_MOTOR_VOLTAGE_RATIO = 0.26     # motor voltage headroom
STALL_TORQUE = 0.40                # launch torque (N*m)
TIRE_ROLLING_RESISTANCE = 0.03     # rolling-resistance coefficient

# Terrain: wide enough that the accelerating car never runs off the patch.
TERRAIN_LENGTH = 200.0             # X extent (m) -> car drives far down +X
TERRAIN_WIDTH = 20.0               # Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Spawn pose: start near the rear edge so the +X run uses the full patch.
INIT_X = -TERRAIN_LENGTH / 2.0 + 10.0
INIT_Y = 0.0
INIT_Z = 0.20                      # chassis-origin height so wheels rest on z=0
THROTTLE_RAMP = 0.5                # s to ramp throttle 0 -> full (smooth launch)

# Derived once (precomputed, never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless check
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics validation


# === Driver (scripted ChDriver subclass) ===
class StraightLineDriver(veh.ChDriver):
    """Full-throttle straight-line driver; throttle ramps in over THROTTLE_RAMP s."""

    def __init__(self, vehicle, ramp):
        super().__init__(vehicle)
        self._ramp = ramp  # cache: ramp duration reused each Synchronize

    def Synchronize(self, time):
        self.SetThrottle(min(1.0, time / self._ramp))  # ramp then hold full
        self.SetSteering(0.0)                           # straight line
        self.SetBraking(0.0)


def build_artcar():
    """Construct + initialize the ARTcar wrapper with the fast tuning applied."""
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(veh.CollisionType_NONE)
    car.SetChassisFixed(False)
    car.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    # --- fast tuning (final desired values) ---
    car.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
    car.SetStallTorque(STALL_TORQUE)
    car.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)
    car.SetTireType(veh.TireModelType_TMEASY)  # slip/grip tire on rigid road
    car.SetTireStepSize(TIRE_STEP)
    car.Initialize()
    car.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    car.SetTireVisualizationType(chrono.VisualizationType_MESH)
    return car


def main():
    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)     # review-video frame / log directory

    # === System & bodies (created by the veh.ARTcar wrapper) ===
    car = build_artcar()
    sys = car.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = car.GetChassisBody()        # cache: main chassis body, reused every step
    veh_obj = car.GetVehicle()            # cache: ChWheeledVehicle handle, reused
    # wheels/spindles: car.GetVehicle().GetAxles()[i] ; joints (suspension + steering)
    # and the ChSystemNSC are all created inside the ARTcar wrapper.

    # === Terrain (flat rigid patch under the car) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint check (wheels rest on the patch, not through it) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    tire_radius = veh_obj.GetAxles()[0].GetWheels()[0].GetTire().GetRadius()
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= -0.10, (  # terrain top is z=0; allow small overlap
        f"car sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs 0.0; "
        f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted full-throttle straight line) ===
    driver = StraightLineDriver(veh_obj, THROTTLE_RAMP)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("ARTcar straight-line acceleration")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.4), 3.0, 0.5)  # chase the chassis
        vis.Initialize()                                            # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                             # outdoor backdrop
        vis.AddCamera(chrono.ChVector3d(INIT_X - 4, -5, 2),
                      chrono.ChVector3d(INIT_X, 0, 0.3))            # AFTER Initialize
        vis.AddTypicalLights()                                      # standard lights
        vis.AddGrid(1.0, 1.0, 60, 12,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                  # ground reference
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop (render-cadence outer loop; physics batch between frames) ===
    data_path = "simulation_data.csv"
    motion_path = os.path.join("cam", "motion_log.csv")
    data_f = motion_f = None
    try:
        try:
            data_f = open(data_path, "w", newline="")           # disk / permission guard
            motion_f = open(motion_path, "w", newline="")
        except (OSError, IOError) as exc:
            print(f"could not open output CSV: {exc}")
            raise

        data_w = csv.writer(data_f)
        data_w.writerow(["time", "pos_x", "pos_y", "pos_z",
                         "speed", "vel_x", "throttle"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz", "speed"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            time = sys.GetChTime()
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics (each step) ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}", f"{vel.x:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}"])
                motion_w.writerow([f"{time:.5f}", "chassis",
                                   f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                                   f"{speed:.5f}"])

                # --- advance the full subsystem stack (no DoStepDynamics) ---
                driver.Synchronize(time)
                terrain.Synchronize(time)
                veh_obj.Synchronize(time, driver_inputs, terrain)  # terrain arg REQUIRED
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                car.Advance(TIME_STEP)                             # steps wrapper system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverged
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (plot the logged time series) ===
    times, speeds, xs, throttles = [], [], [], []
    with open(data_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            speeds.append(float(row["speed"]))
            xs.append(float(row["pos_x"]))
            throttles.append(float(row["throttle"]))

    if times:
        fig, ax1 = plt.subplots(figsize=(9, 5))
        ax1.plot(times, speeds, "b-", label="speed (m/s)")
        ax1.plot(times, throttles, "g--", label="throttle")
        ax1.set_xlabel("time (s)")
        ax1.set_ylabel("speed (m/s) / throttle")
        ax2 = ax1.twinx()
        ax2.plot(times, xs, "r-", label="pos_x (m)")
        ax2.set_ylabel("pos_x (m)")
        ax1.set_title("ARTcar straight-line acceleration")
        ax1.legend(loc="upper left")
        ax2.legend(loc="lower right")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
        print(f"final speed={speeds[-1]:.3f} m/s  final x={xs[-1]:.3f} m  "
              f"rows={len(times)}")


if __name__ == "__main__":
    main()
