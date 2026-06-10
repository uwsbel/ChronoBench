"""HMMWV full-model wheeled vehicle driven by a custom scripted driver.

Model
-----
- A `veh.HMMWV_Full` wrapper vehicle (SMC contact, AWD shaft driveline,
  Pitman-arm steering, TMEASY tires) spawned on a flat rigid terrain patch.
- The vehicle wrapper owns its `ChSystem`; terrain, driver, and visualization
  all attach to that single owned system.

Control law (custom driver `MyDriver`, subclass of `veh.ChDriver`)
------------------------------------------------------------------
The default driver is replaced with a custom scripted driver that overrides
`Synchronize(time)` and sets throttle / steering / braking purely as a function
of simulation time:
  * an input delay of 0.5 s during which the vehicle is held still (brake on),
  * throttle that ramps up to a steady 0.7 once the delay has elapsed and
    0.2 s of ramp time has passed,
  * a sinusoidal steering signal that switches on at 2.0 s.

Expected behavior
------------------
The vehicle stays put for the first 0.5 s, then accelerates forward as the
throttle climbs to 0.7, and begins weaving left/right once sinusoidal steering
engages at 2.0 s. The run ends when simulation time reaches 4.0 s. The chassis
x-displacement should grow monotonically after launch and the yaw should
oscillate after 2.0 s.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# === Named constants (geometry / physics / control schedule) ===
TIME_STEP = 1.0e-3                 # integration step (s)
TIRE_STEP_SIZE = 1.0e-3            # TMEASY tire internal step (s)
SIM_END = 4.0                      # prompt: end the simulation at t = 4 s

DRIVER_DELAY = 0.5                 # prompt: custom driver initialized with delay = 0.5 s
THROTTLE_RAMP_AFTER = 0.2          # prompt: throttle gradually increases after 0.2 s
THROTTLE_TARGET = 0.7             # prompt: throttle reaches 0.7
THROTTLE_RAMP_TIME = 0.6           # seconds to climb 0 -> target (gradual ramp)
STEER_START = 2.0                  # prompt: sinusoidal steering starts at 2 s
STEER_AMPLITUDE = 0.4              # steering magnitude (-1..+1)
STEER_FREQ = 0.5                   # steering oscillation frequency (Hz-ish, rad arg below)

TERRAIN_LENGTH = 200.0             # rigid patch X size (m) — long enough for 4 s run
TERRAIN_WIDTH = 100.0              # rigid patch Y size (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7              # SMC stiffness (Pa)
TERRAIN_TOP_Z = 0.0                # flat patch top plane at z = 0

INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above wheel-bottom at rest
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # precomputed once: chassis spawn height
TIRE_RADIUS = 0.4636               # HMMWV TMEASY tire radius (m) — for footprint assert
ZTOL = 0.08                        # allowed wheel-bottom clearance/overlap vs support top

RENDER_FPS = 50.0                  # review video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Fast, windowless validation run (short bounded sim, no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END


# === Custom scripted driver (replaces the default driver) ===
# MyDriver inherits veh.ChDriver and overrides Synchronize(time) to drive the
# throttle / steering / braking from the time-based schedule above. It is
# constructed with a delay parameter (prompt: 0.5 s); during the delay the
# vehicle is held with full brake and zero throttle.
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay                      # input delay before any motion (s)

    def Synchronize(self, time):
        # Effective control time, shifted by the input delay.
        eff = time - self.delay

        if eff < 0.0:
            # Hold the vehicle still during the delay window.
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
            return

        # Throttle: gradually ramp up to the target once 0.2 s of (effective)
        # time has passed, then hold.
        if eff < THROTTLE_RAMP_AFTER:
            throttle = 0.0
        else:
            ramp = (eff - THROTTLE_RAMP_AFTER) / THROTTLE_RAMP_TIME
            throttle = THROTTLE_TARGET * min(1.0, ramp)
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # Steering: sinusoidal pattern engaging at STEER_START (absolute time).
        if time >= STEER_START:
            self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * STEER_FREQ * (time - STEER_START)))
        else:
            self.SetSteering(0.0)


def main():
    # === Vehicle (veh.HMMWV_Full wrapper owns its ChSystem) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire usable on rigid road
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created internally by the veh.HMMWV_Full wrapper) ===
    sys = hmmwv.GetSystem()                       # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                  # cache: ChWheeledVehicle, reused every step
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
    # links are created inside the wrapper; terrain patch body is added below.

    # === Terrain (flat rigid patch attached to the wrapper-owned system) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Footprint assert: wheels must rest on (not through) the flat support plane.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs support top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver (custom scripted MyDriver, initialized with delay = 0.5 s) ===
    driver = MyDriver(veh_obj, DRIVER_DELAY)
    driver.Initialize()

    # === Visualization === full vehicle-aware Irrlicht scene (window + sky + camera + lights)
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV — custom scripted driver")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                     # steering/throttle/brake HUD bars

    # === Output directories ===
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

    # === Main loop (render-cadence outer loop; Synchronize/Advance per physics step) ===
    data_file = None
    motion_file = None
    times, xs, ys, speeds, throttles, steerings, yaws = [], [], [], [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(
            ["time", "x", "y", "z", "speed", "throttle", "steering", "braking", "yaw_deg"]
        )
        motion_writer.writerow(
            ["time", "chassis_x", "chassis_y", "chassis_z", "vx", "vy", "vz", "speed", "yaw_deg"]
        )

        frame = 0
        running = True
        while running and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                if not vis.Run():
                    break
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()

                # Log physics state every step.
                di = driver.GetInputs()
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                yaw_deg = math.degrees(chassis.GetRot().GetCardanAnglesZYX().z)
                data_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{di.m_throttle:.4f}", f"{di.m_steering:.4f}",
                    f"{di.m_braking:.4f}", f"{yaw_deg:.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                    f"{speed:.5f}", f"{yaw_deg:.4f}",
                ])
                times.append(sim_time)
                xs.append(pos.x); ys.append(pos.y); speeds.append(speed)
                throttles.append(di.m_throttle); steerings.append(di.m_steering)
                yaws.append(yaw_deg)

                # Advance the full subsystem stack (driver/terrain/vehicle/vis).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, di, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, di)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    running = False
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:            # disk / permission errors on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (timeseries plot from the logged CSV data) ===
    if times:
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(times, xs, label="x (m)")
        axs[0].plot(times, ys, label="y (m)")
        axs[0].set_ylabel("position (m)")
        axs[0].legend(loc="best"); axs[0].grid(True)
        axs[1].plot(times, speeds, color="tab:green")
        axs[1].set_ylabel("speed (m/s)"); axs[1].grid(True)
        axs[2].plot(times, throttles, label="throttle")
        axs[2].plot(times, steerings, label="steering")
        axs[2].set_ylabel("driver inputs"); axs[2].set_xlabel("time (s)")
        axs[2].legend(loc="best"); axs[2].grid(True)
        fig.suptitle("HMMWV custom-driver response")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as pf:   # ensure the file handle closes
            fig.savefig(pf, format="png", dpi=110)
        plt.close(fig)

    print(f"Done. logged {len(times)} steps, final x={xs[-1]:.3f} m, "
          f"final speed={speeds[-1]:.3f} m/s" if times else "Done. no steps logged.")


if __name__ == "__main__":
    main()
