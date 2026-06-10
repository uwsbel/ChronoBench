"""MAN_5t military truck driving across rigid hilly terrain.

Model
-----
A MAN_5t catalog wheeled-vehicle wrapper (NSC contact, internally owned
ChSystemNSC) is spawned at world (-20, 0, 1.5) on a RigidTerrain whose surface
is generated from a height map of rolling hills (grass texture). The truck uses
TMEASY deformable-force tires and is driven open-loop (steady throttle, straight
steering) so it climbs and descends the hills along +X.

System type: NSC (set on the MAN wrapper via SetContactMethod).
Main bodies (created inside the wrapper): chassis, 3 axles / 6 spindles+wheels,
plus the rigid terrain patch body built from the height map.

Expected behavior: the truck accelerates from rest and translates several metres
in +X while staying upright (roll/pitch bounded) as it traverses the hills; its
chassis Z rises and falls following the terrain profile. CSV + plot capture the
chassis trajectory, speed, and orientation.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 2e-3                      # integration step (s)
SIM_END = 16.0                        # simulated duration (s)
TIRE_STEP = 1e-3                      # TMEASY tire substep (s)
RENDER_FPS = 30.0                     # review-video frame rate

# Vehicle spawn (world frame); z high enough to clear the hill crests at spawn.
VEH_INIT_X = -20.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 1.5
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.QUNIT               # facing +X

# Hilly rigid terrain built from a height map (rolling hills), grass-textured.
TERRAIN_LENGTH = 100.0                # X extent of the patch (m)
TERRAIN_WIDTH = 100.0                 # Y extent of the patch (m)
HILL_H_MIN = 0.0                      # height-map black -> this height (m)
HILL_H_MAX = 4.0                      # height-map white -> this height (m)
HEIGHTMAP_FILE = chrono.GetChronoDataFile("vehicle/terrain/height_maps/terrain3.bmp")
GRASS_TEXTURE = chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg")

# Open-loop driver schedule (no human-in-the-loop in headless batch runs).
THROTTLE_RAMP_END = 2.0               # s to reach cruise throttle
CRUISE_THROTTLE = 0.7                 # steady throttle after ramp
STRAIGHT_STEERING = 0.0               # drive straight along +X

# Headless validation gate: skip the window, run a short bounded physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run

# Derived constants (precomputed once; never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating


class OpenLoopDriver(veh.ChDriver):
    """Scripted time-based driver: throttle ramp to cruise, straight steering.

    Subclasses veh.ChDriver and sets inputs through the Set* methods inside
    Synchronize so the base GetInputs() returns the scripted state each step.
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(CRUISE_THROTTLE)
        self.SetBraking(0.0)
        self.SetSteering(STRAIGHT_STEERING)


def main():
    # === Vehicle wrapper (creates + owns its ChSystemNSC, chassis, axles) ===
    truck = veh.MAN_5t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    truck.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    truck.SetTireType(veh.TireModelType_TMEASY)      # deformable-force tires for terrain grip
    truck.SetTireStepSize(TIRE_STEP)
    truck.Initialize()

    truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.MAN_5t wrapper) ===
    sys = truck.GetSystem()                # ChSystemNSC owned by the wrapper
    chassis = truck.GetChassisBody()       # cache: main chassis rigid body, reused every step
    veh_obj = truck.GetVehicle()           # cache: ChWheeledVehicle handle, reused every step
    num_axles = veh_obj.GetNumberAxles()   # 3 axles -> 6 spindles/wheels created inside the wrapper

    # === Terrain (rigid patch from a hilly height map, grass texture) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                   # patch centered at origin, no rotation
        HEIGHTMAP_FILE,                    # rolling-hills height map
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        HILL_H_MIN,
        HILL_H_MAX,
    )
    patch.SetTexture(GRASS_TEXTURE, 200, 200)
    terrain.Initialize()

    # Sanity: the spawn must sit above the local terrain surface so the truck
    # drops onto (not through) the hills.
    spawn_ground_z = terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0.0))
    assert VEH_INIT_Z >= spawn_ground_z, (
        f"spawn z={VEH_INIT_Z:.3f} is below terrain z={spawn_ground_z:.3f} at "
        f"({VEH_INIT_X},{VEH_INIT_Y}); raise VEH_INIT_Z"
    )

    # === Driver (scripted open-loop; valid in headless batch runs) ===
    driver = OpenLoopDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("MAN_5t on hilly rigid terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 1.0)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 10.0, -12.0, 6.0), INIT_LOC)
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    data_f = None
    motion_f = None
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
    except (OSError, IOError) as exc:      # disk / permission error opening CSVs
        print(f"failed to open output CSV: {exc}")
        raise

    times, xs, ys, zs, speeds, rolls, pitches = [], [], [], [], [], [], []

    try:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "x", "y", "z", "speed", "roll_deg", "pitch_deg", "throttle"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg"])

        # === Main loop (render-cadence outer loop; Synchronize/Advance stack) ===
        step = 0
        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log chassis pose / speed every physics step.
                pos = chassis.GetPos()
                rot = chassis.GetRot()
                rpy = rot.GetCardanAnglesXYZ()       # roll/pitch/yaw (rad)
                roll_deg = math.degrees(rpy.x)
                pitch_deg = math.degrees(rpy.y)
                yaw_deg = math.degrees(rpy.z)
                speed = veh_obj.GetSpeed()

                data_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}", f"{roll_deg:.4f}",
                                 f"{pitch_deg:.4f}", f"{driver_inputs.m_throttle:.4f}"])
                motion_w.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}",
                                   f"{pos.y:.5f}", f"{pos.z:.5f}", f"{roll_deg:.4f}",
                                   f"{pitch_deg:.4f}", f"{yaw_deg:.4f}"])

                times.append(time)
                xs.append(pos.x)
                ys.append(pos.y)
                zs.append(pos.z)
                speeds.append(speed)
                rolls.append(roll_deg)
                pitches.append(pitch_deg)

                # Synchronize the full subsystem stack, then advance it.
                driver.Synchronize(time)
                terrain.Synchronize(time)
                truck.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                truck.Advance(TIME_STEP)       # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                step += 1
                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"simulation aborted: {exc}")
        raise
    finally:
        # Flush + close partial CSVs even if a step diverged.
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (timeseries plot from logged arrays) ===
    if times:
        t = np.array(times)
        fig, ax = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
        ax[0].plot(t, xs, label="x")
        ax[0].plot(t, ys, label="y")
        ax[0].plot(t, zs, label="z")
        ax[0].set_ylabel("position (m)")
        ax[0].legend(); ax[0].grid(True)
        ax[1].plot(t, speeds, color="tab:red")
        ax[1].set_ylabel("speed (m/s)"); ax[1].grid(True)
        ax[2].plot(t, rolls, label="roll")
        ax[2].plot(t, pitches, label="pitch")
        ax[2].set_ylabel("orientation (deg)"); ax[2].set_xlabel("time (s)")
        ax[2].legend(); ax[2].grid(True)
        fig.suptitle("MAN_5t on hilly rigid terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        x_travel = xs[-1] - xs[0]
        print(f"axles={num_axles} steps={len(times)} "
              f"x_travel={x_travel:.2f} m final_speed={speeds[-1]:.2f} m/s "
              f"max_|roll|={max(abs(r) for r in rolls):.1f} deg")


if __name__ == "__main__":
    main()
