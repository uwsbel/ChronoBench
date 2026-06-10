"""ARTcar (small RC-scale wheeled vehicle) driving on a flat rigid terrain.

Model summary
-------------
- System: NSC (non-smooth contact), owned by the veh.ARTcar wrapper.
- Main bodies: the ARTcar chassis + 4 spindles/wheels with TMEASY tires
  (created internally by the wrapper); a flat veh.RigidTerrain patch with a
  tiled texture acting as the ground/support plane.
- Driver: a scripted veh.ChDriver subclass that brakes briefly, then applies
  throttle with a gentle sinusoidal steering sweep so the RC car drives a
  curving path across the terrain.
- Expected behaviour: the car starts at rest with all four wheels resting on
  the terrain (asserted after Initialize), then accelerates forward and steers,
  travelling a measurable distance while remaining upright.

The visualization uses Irrlicht (veh.ChWheeledVehicleVisualSystemIrrlicht) with
a chase camera; review frames are written to frames/ at 50 fps and assembled
into a video downstream. Physics quantities are logged to CSV every step.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / scheduling (no bare literals downstream)
TIME_STEP = 1.0e-3          # integration step (s)
SIM_END = 12.0              # total simulated time (s)
RENDER_FPS = 50.0           # review video frame rate (prompt: display at 50 fps)
TIRE_STEP_SIZE = 1.0e-3     # tire model sub-step (s)

# ARTcar is RC-scale: keep terrain patch + camera proportionally small.
TERRAIN_LENGTH = 20.0       # rigid terrain X extent (m)
TERRAIN_WIDTH = 20.0        # rigid terrain Y extent (m)
TERRAIN_FRICTION = 0.9      # contact friction (-)
TERRAIN_RESTITUTION = 0.01  # contact restitution (-)

TIRE_RADIUS = 0.085         # ARTcar tire radius (m) — from wheel geometry
TERRAIN_TOP_Z = 0.0         # flat patch top surface height (m)
# Chassis-origin height above the wheel-bottom contact plane at rest. ARTcar
# spindle sits ~0.078 m below the chassis origin, so origin = radius + 0.078.
SUSPENSION_REF_HEIGHT = 0.078
INIT_X = -6.0               # spawn near one end of the patch (m)
INIT_Y = 0.0
INIT_Z = TERRAIN_TOP_Z + TIRE_RADIUS + SUSPENSION_REF_HEIGHT  # rest height (m)

ZTOL = 0.03                 # allowed wheel-bottom clearance/overlap vs terrain (m)

BRAKE_PHASE_END = 0.5       # brake-and-settle window before driving (s)
DRIVE_THROTTLE = 0.6        # steady throttle after settling (-)
STEER_AMPLITUDE = 0.35      # peak steering command (-1..+1)
STEER_FREQ = 0.25           # steering sweep frequency (Hz)

# precomputed once: render cadence (physics steps between rendered frames)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Headless validation gate: skip the window + run a short bounded sim for speed.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating


# === Driver === scripted ChDriver subclass: brake-then-drive with steering sweep
class ScriptedDriver(veh.ChDriver):
    """Open-loop, time-based control: brief brake, then throttle + sinusoidal steer."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_PHASE_END:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * STEER_FREQ * time))


def main():
    # === System & bodies (created by the veh.ARTcar wrapper) ===
    # The wrapper builds and owns the ChSystemNSC plus the chassis, four spindles,
    # wheels, TMEASY tires, suspension and steering joints internally.
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC rigid-contact system
    car.SetChassisCollisionType(veh.CollisionType_NONE)
    car.SetChassisFixed(False)
    car.SetTireType(veh.TireModelType_TMEASY)          # prompt: drivable tire on rigid road
    car.SetTireStepSize(TIRE_STEP_SIZE)
    car.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    car.Initialize()

    car.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    car.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = car.GetSystem()              # ChSystemNSC owned by the wrapper
    veh_obj = car.GetVehicle()            # cache: vehicle handle, reused below
    chassis = car.GetChassisBody()        # cache: main chassis rigid body, reused every step

    # === Footprint assertion === verify all four wheels rest on the terrain top
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )
    assert wheel_bottom_z <= TERRAIN_TOP_Z + 2.0 * ZTOL + TIRE_RADIUS, (
        f"vehicle floats above terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        f"lower SUSPENSION_REF_HEIGHT"
    )

    # === Terrain === flat rigid patch with a tiled texture (the ground/support plane)
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(
        chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 20, 20
    )
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.85))
    terrain.Initialize()

    # === Driver === scripted brake-then-drive controller
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht vehicle scene: window + chase cam + sky + lights
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("ARTcar on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        # RC-scale car -> short chase distance / low height offset
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.3), 2.5, 0.4)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(
            0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4),
        )  # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output setup === create dirs / open CSV writers with context managers
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    times, xs, ys, speeds, rolls = [], [], [], [], []  # for the post-run plot

    sim_f = None
    motion_f = None
    try:
        try:
            sim_f = open("simulation_data.csv", "w", newline="")
            motion_f = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        except (OSError, IOError) as exc:   # disk full / permission denied
            print(f"failed to open CSV output: {exc}")
            raise

        sim_writer = csv.writer(sim_f)
        sim_writer.writerow(
            ["time", "pos_x", "pos_y", "pos_z", "speed", "throttle", "steering", "braking"]
        )
        motion_writer = csv.writer(motion_f)
        motion_writer.writerow(
            ["time", "body", "x", "y", "z", "vx", "vy", "vz", "roll_deg"]
        )

        # === Main loop === render-cadence outer loop; Synchronize/Advance inner batch
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()  # cache: one fetch reused this step

                # --- log physics every step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                rot = chassis.GetRot()
                roll_deg = math.degrees(rot.GetCardanAnglesXYZ().x)
                sim_writer.writerow([
                    f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                    f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_braking:.4f}",
                ])
                motion_writer.writerow([
                    f"{time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                    f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                    f"{roll_deg:.4f}",
                ])
                times.append(time); xs.append(pos.x); ys.append(pos.y)
                speeds.append(speed); rolls.append(roll_deg)

                # --- advance the full vehicle subsystem stack (no sys.DoStepDynamics) ---
                driver.Synchronize(time)
                terrain.Synchronize(time)
                veh_obj.Synchronize(time, driver_inputs, terrain)  # tires sample terrain here
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                veh_obj.Advance(TIME_STEP)   # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close partial CSV output even if a step diverged
        if sim_f is not None:
            sim_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing === plot logged time series to PNG
    if times:
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(times, xs, label="pos_x")
        axs[0].plot(times, ys, label="pos_y")
        axs[0].set_ylabel("position (m)"); axs[0].legend(); axs[0].grid(True)
        axs[1].plot(times, speeds, color="tab:green")
        axs[1].set_ylabel("speed (m/s)"); axs[1].grid(True)
        axs[2].plot(times, rolls, color="tab:red")
        axs[2].set_ylabel("roll (deg)"); axs[2].set_xlabel("time (s)"); axs[2].grid(True)
        fig.suptitle("ARTcar on rigid terrain — motion time series")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        # Concise run summary for the physics check
        dist = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        print(f"steps_logged={len(times)} final_time={times[-1]:.3f}")
        print(f"travel_distance={dist:.3f} m max_speed={max(speeds):.3f} m/s "
              f"max_abs_roll={max(abs(r) for r in rolls):.2f} deg")
        print(f"any_nan={bool(np.isnan(np.array(xs + ys + speeds)).any())}")


if __name__ == "__main__":
    main()
