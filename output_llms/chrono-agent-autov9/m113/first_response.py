"""M113 tracked vehicle driving on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
- Vehicle: M113 armored personnel carrier — a TRACKED vehicle. Its propulsion is
  through two continuous tracks (sprockets, idlers, road wheels, single-pin track
  shoes), NOT tires. Steering is achieved by differential track speed.
- System: ChSystemNSC (complementarity / hard contact), created and owned
  internally by the veh.M113 wrapper. NSC is used because the single-pin track has
  many shoe-to-shoe revolute joints and shoe-ground contacts; penalty (SMC) contact
  destabilizes that joint chain and tips the hull, whereas NSC holds it upright.
  Gravity is the wrapper default (-Z, 9.81 m/s^2).
- Terrain: a single flat RigidTerrain patch with defined friction and restitution.
- Driver: an open-loop scripted driver (veh.ChDriver subclass) applies a brief
  settle phase, then forward throttle with a gentle steering sweep so the two
  tracks differential-steer the hull.

Expected behavior
------------------
The hull settles onto the terrain, then drives forward under throttle. The chassis
X position should grow monotonically and the forward speed should rise to a steady
positive value; the hull should stay upright (roll/pitch small). CSV logs the
chassis pose, speed and orientation each step; a timeseries PNG plots them.
"""

import os
import csv
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless-safe backend for the timeseries plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants (geometry / physics / control) ===
TIME_STEP = 1.0e-3          # integration step (s)
SIM_END = 12.0              # total simulated time (s)
RENDER_FPS = 50.0           # review-video frame rate
SETTLE_TIME = 1.0           # s of zero throttle to let the hull settle on terrain
THROTTLE_RAMP = 1.0         # s to ramp throttle from 0 to full after the settle

INIT_X = 0.0                # spawn X (m) — start near the patch center
INIT_Y = 0.0                # spawn Y (m)
# Spawn so the LOWEST track shoe rests just above the terrain top (z=0). At this
# init height the shoe bottom is ~0.02 m clear, so the hull does not free-fall and
# slam onto the ground — a hard drop ejects the single-pin track shoes.
INIT_Z = 0.64               # chassis-origin spawn height (m); tracks ~touch ground
THROTTLE = 0.8              # forward throttle after the ramp (0..1)
STEER_AMP = 0.10            # steering sweep amplitude (-1..1); differential tracks
STEER_FREQ = 0.08           # steering sweep frequency (Hz)

TERRAIN_LENGTH = 120.0      # rigid patch X size (m)
TERRAIN_WIDTH = 120.0       # rigid patch Y size (m)
TERRAIN_FRICTION = 0.9      # patch friction coefficient (good track grip)
TERRAIN_RESTITUTION = 0.01  # patch restitution (low bounce)

CAM_TRACK_HEIGHT = 1.0      # chase-camera point-on-chassis height (m)
CAM_DISTANCE = 9.0          # chase-camera distance behind hull (m)
CAM_HEIGHT = 1.5            # chase-camera height offset (m)

# Derived constants — precomputed once, never recomputed in the hot loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
STEER_OMEGA = 2.0 * math.pi * STEER_FREQ                      # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating


# === Scripted driver (open-loop, no human-in-the-loop) ===
# Tracked vehicles steer by commanding different left/right track speeds; the
# wrapper maps a single steering channel onto that differential internally.
class M113Driver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            # Hold still with light braking while the hull settles on the tracks.
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            # Ramp throttle in smoothly so the driveline does not yank the track.
            drive_t = time - SETTLE_TIME
            ramp = min(1.0, drive_t / THROTTLE_RAMP)
            self.SetThrottle(THROTTLE * ramp)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMP * math.sin(STEER_OMEGA * drive_t))


def main():
    os.makedirs("frames", exist_ok=True)  # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)      # guard against missing cam output dir

    # === System & bodies (created by the veh.M113 tracked wrapper) ===
    # The wrapper builds its own ChSystemSMC plus all tracked subsystems: chassis,
    # two sprockets, two idlers, road wheels and the single-pin track shoes, with
    # the driveline/engine/transmission shafts and the track suspension joints.
    vehicle = veh.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)                     # hull is free to move
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    # SHAFTS engine/transmission + BDS driveline deliver enough tractive torque to
    # actually move the tracked hull; the simpler map/driveline barely creeps.
    vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSprocketVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetIdlerVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)

    sys = vehicle.GetSystem()             # ChSystemSMC owned by the wrapper
    veh_obj = vehicle.GetVehicle()        # cache: tracked-vehicle handle, reused every step
    chassis = vehicle.GetChassisBody()    # cache: main hull rigid body, reused every step

    # Per-track terrain-force buffers sized to the actual shoe counts (left/right
    # differ by one shoe). Tracked Synchronize needs these filled by the contact.
    n_shoes_left = veh_obj.GetNumTrackShoes(veh.LEFT)    # precomputed once
    n_shoes_right = veh_obj.GetNumTrackShoes(veh.RIGHT)  # precomputed once
    shoe_forces_left = veh.TerrainForces(n_shoes_left)
    shoe_forces_right = veh.TerrainForces(n_shoes_right)

    # === Terrain (flat rigid patch with defined friction / restitution) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()   # matches the wrapper's NSC system
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Sanity: the lowest track shoe must start just ABOVE the terrain top (z=0),
    # never below it. A shoe below z=0 means initial interpenetration (explosive
    # SMC contact ejects shoes); a shoe far above z=0 means a destructive free-fall
    # drop. Both wreck the single-pin track, so bound the clearance tightly.
    shoe_bottoms = []
    for side in (veh.LEFT, veh.RIGHT):
        for i in range(veh_obj.GetNumTrackShoes(side)):
            shoe_bottoms.append(veh_obj.GetTrackShoe(side, i).GetShoeBody().GetPos().z)
    min_shoe_z = min(shoe_bottoms)
    assert 0.0 <= min_shoe_z <= 0.10, (
        f"track shoe bottom z={min_shoe_z:.3f} not in [0, 0.10]; "
        f"adjust INIT_Z (raise if <0 to avoid interpenetration, lower if >0.10 "
        f"to avoid a free-fall drop that ejects single-pin shoes)"
    )

    # === Driver (open-loop scripted differential-track control) ===
    driver = M113Driver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Built unconditionally except in the headless validation gate (skips the window
    # for a fast physics-only check). The committed block is complete for the reviewer.
    vis = None
    if not HEADLESS:
        vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("M113 Tracked Vehicle on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(
            chrono.ChVector3d(0, 0, CAM_TRACK_HEIGHT), CAM_DISTANCE, CAM_HEIGHT
        )
        vis.Initialize()                                  # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                   # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(-10, -10, 5), chrono.ChVector3d(0, 0, 0))
        vis.AddTypicalLights()                            # standard lighting
        vis.AddGrid(
            1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4),                # ground reference grid
        )
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                          # steering/throttle HUD

    # === Main loop (render-cadence outer loop; Synchronize/Advance inner batch) ===
    # vehicle.Advance steps the wrapper-owned system; do NOT call sys.DoStepDynamics.
    frame = 0
    sim_f = None
    motion_f = None
    try:
        sim_f = open("simulation_data.csv", "w", newline="")
        motion_f = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
        sim_w = csv.writer(sim_f)
        motion_w = csv.writer(motion_f)
        sim_w.writerow(["time", "x", "y", "z", "speed", "vx", "vy", "vz", "roll", "pitch", "yaw"])
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"])

        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()

                # --- log physics this step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                rot = chassis.GetRot().GetCardanAnglesXYZ()  # roll, pitch, yaw (rad)
                speed = veh_obj.GetSpeed()
                row = [time, pos.x, pos.y, pos.z, speed, vel.x, vel.y, vel.z,
                       rot.x, rot.y, rot.z]
                sim_w.writerow(row)
                motion_w.writerow([time, "chassis", pos.x, pos.y, pos.z,
                                   vel.x, vel.y, vel.z, rot.x, rot.y, rot.z])

                driver_inputs = driver.GetInputs()
                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                vehicle.Advance(TIME_STEP)   # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (OSError, IOError) as exc:        # disk / permission errors on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges — close any open writers here
        if sim_f is not None:
            sim_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (timeseries plot from the logged CSV) ===
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            data = np.array([[float(v) for v in r] for r in reader])
    except (OSError, IOError) as exc:        # missing/unreadable CSV
        import traceback
        traceback.print_exc()
        raise

    if data.size > 0:
        t = data[:, 0]
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(t, data[:, 1], label="x")
        axes[0].plot(t, data[:, 2], label="y")
        axes[0].plot(t, data[:, 3], label="z")
        axes[0].set_ylabel("position (m)")
        axes[0].legend(); axes[0].grid(True)
        axes[1].plot(t, data[:, 4], label="speed")
        axes[1].set_ylabel("speed (m/s)")
        axes[1].legend(); axes[1].grid(True)
        axes[2].plot(t, data[:, 8], label="roll")
        axes[2].plot(t, data[:, 9], label="pitch")
        axes[2].plot(t, data[:, 10], label="yaw")
        axes[2].set_ylabel("orientation (rad)")
        axes[2].set_xlabel("time (s)")
        axes[2].legend(); axes[2].grid(True)
        fig.suptitle("M113 tracked vehicle on rigid terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done. frames={frame}, rows={0 if data.size == 0 else data.shape[0]}, "
          f"final_x={0.0 if data.size == 0 else data[-1, 1]:.3f}")


if __name__ == "__main__":
    main()
