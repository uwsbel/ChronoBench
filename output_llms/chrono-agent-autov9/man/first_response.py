"""MAN 10t military truck driving on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
- Vehicle : veh.MAN_10t wrapper (8x8 heavy truck). The wrapper internally creates
  and owns a ChSystemNSC, the chassis rigid body, four axles / eight spindle bodies,
  the suspension + steering joints, the driveline, and the powertrain.
- Tire    : TMEASY handling tire model (slip/grip force curves) on every wheel.
- Terrain : a single flat veh.RigidTerrain patch (rigid ground, Bullet contacts),
  textured and centered at the world origin (Z-up world).
- Driver  : a scripted veh.ChDriver subclass providing time-based steering /
  throttle / braking (no human-in-the-loop, so the run is reproducible headless).
- Render  : veh.ChWheeledVehicleVisualSystemIrrlicht chase-camera window with a
  sky box, directional/typical lighting, a logo, and a ground grid reference.

Expected behavior
-----------------
After a short brake-hold the truck applies throttle and accelerates forward along
+X on the rigid patch, staying upright (chassis Z roughly constant, |roll|/|pitch|
small), so the logged forward displacement and speed grow monotonically while the
truck remains stable. The contact system is NSC (non-smooth rigid contact).
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / control (no bare literals downstream)
TIME_STEP = 2.0e-3                 # integration step (s)
SIM_END = 12.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps/frame

TERRAIN_LENGTH = 200.0             # rigid patch X extent (m)
TERRAIN_WIDTH = 60.0               # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9             # tire-ground friction coefficient
TERRAIN_RESTITUTION = 0.01         # near-inelastic ground

TIRE_RADIUS = 0.629                # MAN_10t TMEASY tire radius (m), from wheel geometry
GROUND_CLEARANCE = 0.02            # wheel-bottom clearance above the patch at spawn (m)
INIT_X = -TERRAIN_LENGTH / 2 + 12.0  # spawn near the rear edge so there is room to drive
INIT_Y = 0.0
# Spindle Z equals the chassis-origin Z for this wrapper, so wheel bottom = init_z - TIRE_RADIUS.
INIT_Z = TIRE_RADIUS + GROUND_CLEARANCE
PATCH_TOP_Z = 0.0                  # flat patch surface height

BRAKE_HOLD_T = 1.0                 # hold brakes for the first second (settle on tires)
RAMP_T = 3.0                       # time to ramp throttle to cruise
CRUISE_THROTTLE = 0.7              # steady-state throttle after the ramp
STEER_AMP = 0.10                   # gentle steering oscillation amplitude (-1..1)
STEER_FREQ = 0.15                  # steering oscillation frequency (Hz)

CHASE_DISTANCE = 14.0              # chase-camera distance behind the truck (m)
CHASE_HEIGHT = 2.2                 # chase-camera height offset (m)
ZTOL = 0.06                        # allowed wheel-bottom overlap/clearance vs patch top

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating


# === Driver === scripted time-based control (autonomous; safe for headless runs)
class ScriptedTruckDriver(veh.ChDriver):
    """Brake-hold, then ramp throttle to cruise with a gentle steering wiggle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_HOLD_T:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            # linear throttle ramp from BRAKE_HOLD_T over RAMP_T seconds, then cruise
            ramp = min(1.0, (time - BRAKE_HOLD_T) / RAMP_T)
            self.SetThrottle(CRUISE_THROTTLE * ramp)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMP * math.sin(2.0 * math.pi * STEER_FREQ * time))


def build_vehicle():
    """Create + initialize the MAN_10t wrapper with configurable viz/collision + TMEASY tires."""
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)        # rigid NSC contact
    truck.SetChassisCollisionType(veh.CollisionType_NONE)     # configurable collision setting
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
                                             chrono.QUNIT))
    truck.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    truck.SetTireType(veh.TireModelType_TMEASY)               # prompt: TMEASY tire model
    truck.SetTireStepSize(TIME_STEP)
    truck.Initialize()

    # configurable visualization settings (chrono.* enum namespace)
    truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    truck.SetTireVisualizationType(chrono.VisualizationType_MESH)
    return truck


def build_terrain(system):
    """Flat textured rigid terrain patch under the truck."""
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    return terrain


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame output dir
    os.makedirs("cam", exist_ok=True)       # guard against missing motion-log output dir

    # === System & bodies (created by the veh.MAN_10t wrapper) ===
    truck = build_vehicle()
    system = truck.GetSystem()              # ChSystemNSC owned by the wrapper
    chassis = truck.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = truck.GetVehicle()            # cache: ChWheeledVehicle handle, reused every step
    # axles/spindles: veh_obj.GetSpindlePos(axle, side) for all 4 axles x 2 sides (8 wheels)
    # joints: suspension + steering links created inside the wrapper
    # terrain: RigidTerrain patch body created just below

    # === Terrain ===
    terrain = build_terrain(system)

    # --- Footprint assert: wheels must rest on (not through) the rigid patch ---
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= PATCH_TOP_Z - ZTOL, (
        f"truck sinks into patch: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs patch top z={PATCH_TOP_Z:.3f}; raise INIT_Z by "
        f"{PATCH_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted autonomous control
    driver = ScriptedTruckDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("MAN 10t truck on rigid terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, CHASE_HEIGHT), CHASE_DISTANCE, 0.6)
        vis.Initialize()                                       # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                        # outdoor sky backdrop
        vis.AddTypicalLights()                                 # directional/typical lighting
        vis.AddLight(chrono.ChVector3d(30, 30, 80), 250, chrono.ChColor(0.9, 0.9, 0.9))
        vis.AddGrid(2.0, 2.0, 60, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.35, 0.35, 0.35))          # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                               # steering/throttle/brake HUD

    # === Main loop === render-cadence outer loop + Synchronize/Advance inner physics batch
    data_csv = None
    motion_csv = None
    data_writer = None
    motion_writer = None
    try:
        data_csv = open("simulation_data.csv", "w", newline="")
        motion_csv = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_csv)
        motion_writer = csv.writer(motion_csv)
        data_writer.writerow(["time", "x", "y", "z", "speed", "throttle", "steering", "braking"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "roll", "pitch", "yaw", "speed"])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics this step ---
                pos = chassis.GetPos()
                speed = veh_obj.GetSpeed()
                rot = chassis.GetRot()
                euler = rot.GetCardanAnglesXYZ()   # roll(x), pitch(y), yaw(z)
                data_writer.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                      f"{pos.z:.5f}", f"{speed:.5f}",
                                      f"{driver_inputs.m_throttle:.4f}",
                                      f"{driver_inputs.m_steering:.4f}",
                                      f"{driver_inputs.m_braking:.4f}"])
                motion_writer.writerow([f"{sim_time:.5f}", "chassis",
                                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                        f"{euler.x:.5f}", f"{euler.y:.5f}", f"{euler.z:.5f}",
                                        f"{speed:.5f}"])

                # --- subsystem synchronize then advance (NO sys.DoStepDynamics) ---
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                truck.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                truck.Advance(TIME_STEP)        # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close any open writers even if a step diverged mid-run
        if data_csv is not None:
            data_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing === plot logged time series from the CSV
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            reader = csv.DictReader(f)
            t, x, z, spd, thr = [], [], [], [], []
            for row in reader:
                t.append(float(row["time"]))
                x.append(float(row["x"]))
                z.append(float(row["z"]))
                spd.append(float(row["speed"]))
                thr.append(float(row["throttle"]))
    except (OSError, IOError, ValueError) as exc:   # missing/corrupt CSV -> skip plot
        import traceback
        traceback.print_exc()
        t = []

    if t:
        ta = np.array(t)
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(ta, np.array(x), label="x (forward)")
        axs[0].plot(ta, np.array(z), label="z (height)")
        axs[0].set_ylabel("position (m)")
        axs[0].legend(); axs[0].grid(True)
        axs[1].plot(ta, np.array(spd), color="tab:green")
        axs[1].set_ylabel("speed (m/s)"); axs[1].grid(True)
        axs[2].plot(ta, np.array(thr), color="tab:red")
        axs[2].set_ylabel("throttle"); axs[2].set_xlabel("time (s)"); axs[2].grid(True)
        fig.suptitle("MAN 10t truck on rigid terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
