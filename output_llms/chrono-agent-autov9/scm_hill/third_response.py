"""HMMWV climbing a rigid height-mapped hill (PyChrono 9.0.1 + Irrlicht).

Model
-----
A full HMMWV wheeled vehicle drives up an off-road hill. The ground is a single
RIGID terrain patch generated from a grayscale height map (a sloped hill rising
toward the far side of the patch) with a grass texture. The vehicle uses TMEASY
deformable tires with explicit per-spindle collision cylinders so the wheels grip
the rigid surface.

System type
-----------
NSC (Non-Smooth Contact, complementarity solver). The HMMWV wrapper owns the
ChSystemNSC; the terrain, driver, and visualization are attached to that system.

Main bodies
-----------
- HMMWV chassis + 4 spindles/wheels/tires (created by the veh.HMMWV_Full wrapper)
- RigidTerrain hill patch (a fixed rigid body carrying the height-map mesh)

Expected behavior
------------------
The vehicle starts at the low end of the patch, applies full throttle, and climbs
the hill. The chassis X position and elevation (Z) both increase over the run,
demonstrating a successful ascent of the rigid hill.
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

# === Constants (geometry / physics) ===
TIME_STEP = 2.0e-3                 # integration step (s) — NSC stable for vehicle
TIRE_STEP = 1.0e-3                 # TMEASY tire force update step (s)
SIM_END = 8.0                      # total simulated time (s) — ends on the crest, on-patch
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 60.0              # patch size along X (m)
TERRAIN_WIDTH = 40.0               # patch size along Y (m)
HILL_MIN_H = 0.0                   # height-map black -> min elevation (m)
HILL_MAX_H = 6.0                   # height-map white -> crest elevation (m)

SUSPENSION_REF_HEIGHT = 0.55       # HMMWV chassis origin above wheel-bottom at rest (m)
TIRE_FAMILY = 1                    # collision family for tire cylinders
TIRE_EXTRA_RAD = 0.04              # cylinder radius margin so wheels contact firmly

SPAWN_X = -22.0                    # spawn near the low (uphill-facing) end of the patch
SPAWN_Y = 0.0
FRICTION = 0.9                     # terrain/tire friction coefficient
RESTITUTION = 0.01                 # terrain/tire restitution

HEIGHTMAP_FILE = veh.GetVehicleDataFile("terrain/height_maps/slope.bmp")
TEXTURE_FILE = veh.GetVehicleDataFile("terrain/textures/grass.jpg")

# Derived render cadence — precomputed once, never recomputed in the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Fast windowless validation gate: short bounded run, no Irrlicht window.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


class HillClimbDriver(veh.ChDriver):
    """Scripted open-loop driver: brief settle, then full throttle straight ahead."""

    def __init__(self, vehicle, settle_time):
        super().__init__(vehicle)
        self._settle = settle_time           # cache: constant, reused every Synchronize

    def Synchronize(self, time):
        if time < self._settle:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(1.0)            # full throttle to climb the hill
            self.SetBraking(0.0)
        self.SetSteering(0.0)                # drive straight up the slope


def main():
    os.makedirs("frames", exist_ok=True)     # guard against missing output dir
    os.makedirs("cam", exist_ok=True)         # review-video frame + motion-log dir

    # === System & vehicle (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper builds and owns a ChSystemNSC plus the chassis rigid body, the
    # four spindle/wheel/tire bodies, and all suspension + steering joints.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # prompt: NSC contact method
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)          # deformable tire (grips rigid hill)
    hmmwv.SetTireStepSize(TIRE_STEP)

    # Spawn at the low (black) end of the slope height map, where the surface is at
    # the map's minimum elevation. The chassis origin rests SUSPENSION_REF_HEIGHT
    # above that ground plane so the wheels start on the hill foot.
    spawn_ground_z = HILL_MIN_H                          # low end of the slope height map
    init_z = spawn_ground_z + SUSPENSION_REF_HEIGHT
    init_loc = chrono.ChVector3d(SPAWN_X, SPAWN_Y, init_z)
    init_rot = chrono.QUNIT                               # facing +X (uphill)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.Initialize()

    # === Terrain (single rigid patch from a height map) ===
    # The HMMWV wrapper builds its collision system during Initialize(), so the
    # rigid terrain (which needs that collision system) is created afterward. The
    # patch is a single height-mapped hill rising from the black (low) end toward
    # the white (high) crest along +X, textured with grass.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()             # NSC system -> NSC material
    patch_mat.SetFriction(FRICTION)
    patch_mat.SetRestitution(RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                                  # patch centered at world origin
        HEIGHTMAP_FILE,                                   # grayscale hill height map
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        HILL_MIN_H,
        HILL_MAX_H,
    )
    patch.SetTexture(TEXTURE_FILE, 200, 200)              # grass texture, UV tiled
    patch.SetColor(chrono.ChColor(0.5, 0.6, 0.4))
    terrain.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # Named handles into the wrapper-created components (visible to readers).
    system = hmmwv.GetSystem()                            # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()                      # cache: main chassis body, reused every step
    veh_obj = hmmwv.GetVehicle()                          # cache: vehicle subsystem, reused every step

    # === Tire collision cylinders (TMEASY tires need explicit collision geometry) ===
    tire0 = veh_obj.GetAxles()[0].m_wheels[0].GetTire()
    tire_rad = tire0.GetRadius()
    tire_w = tire0.GetWidth()
    tire_mat = chrono.ChContactMaterialNSC()              # NSC system -> NSC material
    tire_mat.SetFriction(FRICTION)
    tire_mat.SetRestitution(RESTITUTION)
    for axle in veh_obj.GetAxles():
        for iw in range(2):
            spindle = axle.m_wheels[iw].GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + TIRE_EXTRA_RAD, tire_w),
                chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
            )
            spindle.EnableCollision(True)
            sp_cm = spindle.GetCollisionModel()
            sp_cm.SetFamily(TIRE_FAMILY)
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)     # wheels never collide with each other
    # Rebuild all collision models so the new cylinders are visible to contact.
    system.GetCollisionSystem().BindAll()

    # Assert the wheels rest on (not through) the sampled hill surface.
    spindle_z = [veh_obj.GetSpindlePos(a, s).z
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
    wheel_bottom_z = min(spindle_z) - tire_rad
    assert wheel_bottom_z >= spawn_ground_z - 0.10, (
        f"vehicle sinks into hill: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs ground z={spawn_ground_z:.3f}; raise SUSPENSION_REF_HEIGHT"
    )

    # === Driver (scripted open-loop hill climb) ===
    driver = HillClimbDriver(veh_obj, settle_time=0.5)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV Rigid Hill Climb")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
        vis.Initialize()                                  # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVector3d(SPAWN_X - 8.0, -12.0, spawn_ground_z + 6.0),
                      chrono.ChVector3d(SPAWN_X, 0.0, spawn_ground_z + 1.0))
        vis.AddGrid(2.0, 2.0, 30, 20,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, spawn_ground_z + 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    run_end = min(SIM_END, 1.0) if HEADLESS else SIM_END  # short physics check when validating

    # === Main loop (render-cadence outer loop; physics inner batch) ===
    data_file = None
    motion_file = None
    times, xs, zs, speeds, throttles = [], [], [], [], []
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z",
                              "speed", "throttle"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics every step.
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_writer.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}",
                                      f"{pos.y:.5f}", f"{pos.z:.5f}",
                                      f"{speed:.5f}", f"{driver_inputs.m_throttle:.3f}"])
                motion_writer.writerow([f"{sim_time:.5f}", "chassis",
                                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                        f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(sim_time)
                xs.append(pos.x)
                zs.append(pos.z)
                speeds.append(speed)
                throttles.append(driver_inputs.m_throttle)

                # Advance the full subsystem stack (no DoStepDynamics — Advance steps it).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)         # internally calls DoStepDynamics
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverges mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (timeseries plot from logged data) ===
    if times:
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(times, xs, label="chassis X (m)")
        ax[0].plot(times, zs, label="chassis Z (m)")
        ax[0].set_ylabel("position (m)")
        ax[0].legend(); ax[0].grid(True)
        ax[1].plot(times, speeds, color="tab:green")
        ax[1].set_ylabel("speed (m/s)"); ax[1].grid(True)
        ax[2].plot(times, throttles, color="tab:red")
        ax[2].set_ylabel("throttle"); ax[2].set_xlabel("time (s)")
        ax[2].set_ylim(-0.05, 1.05); ax[2].grid(True)
        fig.suptitle("HMMWV Rigid Hill Climb")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        climb = zs[-1] - zs[0]
        advance = xs[-1] - xs[0]
        print(f"[summary] dt-X={advance:.3f} m  dt-Z(climb)={climb:.3f} m  "
              f"final speed={speeds[-1]:.3f} m/s  steps={len(times)}")


if __name__ == "__main__":
    main()
