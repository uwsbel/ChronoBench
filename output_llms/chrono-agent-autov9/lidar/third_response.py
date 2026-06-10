"""ARTcar wheeled vehicle on flat rigid terrain, instrumented with lidar sensors.

Model overview
--------------
* System type: NSC (the ARTcar wrapper owns its ChSystemNSC; contact method SMC
  is selected on the wrapper for stable tire/terrain contact).
* Main bodies (created inside the veh.ARTcar wrapper): chassis rigid body, four
  spindles + wheels with RIGID tires, double-wishbone suspension links, Pitman-arm
  steering. A flat veh.RigidTerrain patch provides the driving surface and, being
  collidable, is what the lidar beams strike.
* Sensors (the subject of this demo, via a sens.ChSensorManager):
  - a 3D scanning lidar (multi-row vertical FOV) attached to the chassis,
  - a 2D planar lidar (single row) attached to the chassis,
  both mounted at the offset pose (1.0, 0, 1) m in the chassis frame, looking
  forward, and a third-person RGB camera attached to the chassis for the review.
* Expected behavior: the ARTcar drives forward under a scripted throttle while the
  lidars continuously sample the terrain/scene; the script logs per-update point
  counts and X/Y/Z/intensity ranges to CSV and renders a third-person review video
  from an Irrlicht window.

Visualization: Irrlicht is the review renderer (window + sky + camera + lights +
grid). The lidars and the third-person camera are OptiX sensors rendered off-screen
by the sensor manager; OptiX only sees bodies that carry COLLISION geometry, so the
terrain patch is collidable.
"""

# === Imports ===
import math
import os
import csv

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend (no display needed for the PNG)
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / timing) ===
TIME_STEP = 2.0e-3              # s, integration step (raised for sensor-scene throughput)
TIRE_STEP = 1.0e-3             # s, tire model sub-step
SIM_END = 8.0                  # s, total simulated time
RENDER_FPS = 30.0              # review-video frame rate

TERRAIN_LENGTH = 100.0         # m, X extent of the rigid terrain patch
TERRAIN_WIDTH = 100.0          # m, Y extent of the rigid terrain patch
TERRAIN_FRICTION = 0.9         # tire/terrain friction coefficient
TERRAIN_RESTITUTION = 0.01     # terrain bounciness
TERRAIN_YOUNG = 2.0e7          # Pa, SMC terrain stiffness

VEH_INIT_X = 0.0               # m, chassis spawn X
VEH_INIT_Y = 0.0               # m, chassis spawn Y
VEH_INIT_Z = 0.2               # m, chassis spawn Z (ARTcar is a small 1/6-scale car)

# Lidar mount offset in the chassis frame (forward-and-up), per the requested pose.
LIDAR_OFFSET = chrono.ChVector3d(1.0, 0.0, 1.0)

# 3D scanning lidar geometry.
LIDAR3D_HFOV = 2.0 * math.pi             # rad, full 360 deg horizontal sweep
LIDAR3D_VMAX = 0.2618                     # rad, +15 deg top beam
LIDAR3D_VMIN = -0.2618                    # rad, -15 deg bottom beam
LIDAR3D_W = 360                           # horizontal samples
LIDAR3D_H = 16                            # vertical channels
LIDAR_MAX_DIST = 100.0                    # m, max return range
LIDAR_UPDATE_RATE = 10.0                  # Hz, lidar scan rate

# 2D planar lidar geometry (single horizontal row).
LIDAR2D_HFOV = math.pi                     # rad, 180 deg forward fan
LIDAR2D_W = 360                            # horizontal samples
LIDAR2D_H = 1                              # single row -> planar scan
LIDAR2D_TILT = -0.05                       # rad, slight downward tilt so the planar row grazes the ground

# Third-person review camera.
CAM_W, CAM_H = 1280, 720
CAM_FOV = 1.408                            # rad, horizontal FOV
CAM_UPDATE_RATE = 30.0                     # Hz
CAM_OFFSET = chrono.ChVector3d(-5.0, 0.0, 2.0)   # behind & above chassis (chassis frame)

# Scripted driving profile.
THROTTLE_DELAY = 0.5           # s, brief settle before driving
CRUISE_THROTTLE = 0.3          # steady throttle once moving (kept modest so the chase camera tracks)
STEER_AMP = 0.15               # gentle sinusoidal steering amplitude
STEER_FREQ = 0.25              # Hz, steering oscillation

# Headless validation gate: a short, windowless physics+sensor check for fast CI.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

# Derived constants (precomputed once).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END         # short physics check when validating


def main():
    # === Vehicle (ARTcar wrapper owns the ChSystem) ===
    # The wrapper internally creates the ChSystemNSC, the chassis rigid body, four
    # wheels/spindles, the suspension + steering joints, and the powertrain.
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_SMC)
    car.SetChassisCollisionType(veh.CollisionType_NONE)
    car.SetChassisFixed(False)
    car.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
    )
    car.SetTireType(veh.TireModelType_RIGID)   # rigid tires are fine on a flat rigid road
    car.SetTireStepSize(TIRE_STEP)
    car.Initialize()

    car.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    car.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.ARTcar wrapper) ===
    system = car.GetSystem()                  # cache: ChSystem owned by the wrapper, reused every step
    chassis_body = car.GetChassisBody()       # cache: main chassis rigid body, reused every step
    # spindles/wheels: car.GetVehicle().GetAxle(i)...; joints: suspension + steering inside the wrapper.

    # Footprint sanity: the spawned wheels must rest on (not through) the flat terrain at z=0.
    veh_obj = car.GetVehicle()
    spindle_zs = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_zs.append(veh_obj.GetSpindlePos(axle, side).z)
    assert min(spindle_zs) > -0.5, (
        f"spindles spawned below terrain (min z={min(spindle_zs):.3f}); raise VEH_INIT_Z"
    )

    # === Terrain (flat rigid patch; collidable so the lidar beams have something to hit) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.7, 0.7, 0.6))
    terrain.Initialize()

    # === Driver (scripted, autonomous — no human-in-the-loop in headless runs) ===
    class ScriptedDriver(veh.ChDriver):
        def __init__(self, vehicle):
            super().__init__(vehicle)

        def Synchronize(self, time):
            if time < THROTTLE_DELAY:
                self.SetThrottle(0.0)
            else:
                self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMP * math.sin(2.0 * math.pi * STEER_FREQ * time))

    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Sensors: ChSensorManager + scene lighting ===
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(
        chrono.ChVector3f(20, 20, 50), chrono.ChColor(1.0, 1.0, 1.0), 500.0
    )
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.4, 0.4, 0.4))

    offset_frame = chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT)

    # 3D scanning lidar attached to the chassis.
    lidar3d = sens.ChLidarSensor(
        chassis_body, LIDAR_UPDATE_RATE, offset_frame,
        LIDAR3D_W, LIDAR3D_H, LIDAR3D_HFOV,
        LIDAR3D_VMAX, LIDAR3D_VMIN, LIDAR_MAX_DIST,
        sens.LidarBeamShape_RECTANGULAR, 1, 0.003, 0.003,
        sens.LidarReturnMode_MEAN_RETURN, 1e-3,
    )
    lidar3d.SetName("lidar_3d")
    lidar3d.PushFilter(sens.ChFilterDIAccess())                 # depth/intensity buffer access
    lidar3d.PushFilter(sens.ChFilterPCfromDepth())              # convert depth -> point cloud
    lidar3d.PushFilter(sens.ChFilterXYZIAccess())               # XYZ + intensity access
    lidar3d.PushFilter(sens.ChFilterSavePtCloud("lidar3d_pc/"))  # PLY point clouds to disk
    manager.AddSensor(lidar3d)

    # 2D planar lidar attached to the chassis (single horizontal row).
    lidar2d = sens.ChLidarSensor(
        chassis_body, LIDAR_UPDATE_RATE, offset_frame,
        LIDAR2D_W, LIDAR2D_H, LIDAR2D_HFOV,
        LIDAR2D_TILT, LIDAR2D_TILT, LIDAR_MAX_DIST,
        sens.LidarBeamShape_RECTANGULAR, 1, 0.003, 0.003,
        sens.LidarReturnMode_MEAN_RETURN, 1e-3,
    )
    lidar2d.SetName("lidar_2d")
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar2d)

    # Third-person RGB camera attached to the chassis.
    third_person = sens.ChCameraSensor(
        chassis_body, CAM_UPDATE_RATE,
        chrono.ChFramed(CAM_OFFSET, chrono.QUNIT),
        CAM_W, CAM_H, CAM_FOV,
    )
    third_person.SetName("third_person_cam")
    third_person.PushFilter(sens.ChFilterSave("cam/third_person/"))  # PNG frames -> mp4 later
    third_person.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(third_person)

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("ARTcar with Lidar Sensors")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.3), 4.0, 1.0)  # third-person chase view
        vis.SetChaseCameraState(veh.ChChaseCamera.Chase)  # actively track the chassis (not free/fixed)
        vis.SetChaseCameraMultipliers(2.0, 4.0)  # responsive chase gains so the camera keeps up
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(
            1.0, 1.0, 60, 60,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4),
        )
        vis.AttachVehicle(veh_obj)

    # === Output dirs + CSV setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
    os.makedirs("cam", exist_ok=True)      # guard against missing output dir for camera/logs

    data_file = None
    motion_file = None
    try:
        try:
            data_file = open("simulation_data.csv", "w", newline="")
            motion_file = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:   # disk full / permission denied
            print(f"failed to open CSV output: {exc}")
            raise

        data_writer = csv.writer(data_file)
        data_writer.writerow([
            "time", "lidar3d_points", "lidar3d_x_min", "lidar3d_x_max",
            "lidar3d_y_min", "lidar3d_y_max", "lidar3d_z_min", "lidar3d_z_max",
            "lidar3d_i_min", "lidar3d_i_max", "lidar2d_points",
            "lidar2d_range_min", "lidar2d_range_max",
        ])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow([
            "time", "chassis_x", "chassis_y", "chassis_z", "speed", "throttle", "steering",
        ])

        # Time-series accumulators for the post-run plot.
        t_hist, pts3d_hist, speed_hist, range2d_hist = [], [], [], []

        # === Main loop (render-cadence outer loop; physics + sensors in inner batch) ===
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

                # Subsystem synchronize order: driver, terrain, vehicle, vis.
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                car.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                manager.Update()   # pump sensors every physics step (sees every post-step pose)

                # --- Log lidar stats + vehicle motion this step ---
                speed = veh_obj.GetSpeed()
                pos = chassis_body.GetPos()

                n3d = 0
                x_rng = (0.0, 0.0)
                y_rng = (0.0, 0.0)
                z_rng = (0.0, 0.0)
                i_rng = (0.0, 0.0)
                xyzi_buf = lidar3d.GetMostRecentXYZIBuffer()  # may be empty before first tick
                if xyzi_buf.HasData():                         # guard: skip unfilled buffers
                    data = xyzi_buf.GetXYZIData()              # safe only after HasData()
                    arr = np.asarray(data, dtype=np.float32).reshape(-1, 4)
                    # Keep only points with non-zero range (hit something within max_distance).
                    hit = arr[np.abs(arr[:, :3]).sum(axis=1) > 1e-6]
                    n3d = int(hit.shape[0])
                    if n3d > 0:
                        x_rng = (float(hit[:, 0].min()), float(hit[:, 0].max()))
                        y_rng = (float(hit[:, 1].min()), float(hit[:, 1].max()))
                        z_rng = (float(hit[:, 2].min()), float(hit[:, 2].max()))
                        i_rng = (float(hit[:, 3].min()), float(hit[:, 3].max()))

                n2d = 0
                r2d_rng = (0.0, 0.0)
                di_buf = lidar2d.GetMostRecentDIBuffer()       # depth/intensity, may be empty
                if di_buf.HasData():                           # guard: skip unfilled buffers
                    di = np.asarray(di_buf.GetDIData(), dtype=np.float32).reshape(-1, 2)
                    valid = di[(di[:, 0] > 1e-6) & (di[:, 0] < LIDAR_MAX_DIST)]
                    n2d = int(valid.shape[0])
                    if n2d > 0:
                        r2d_rng = (float(valid[:, 0].min()), float(valid[:, 0].max()))

                data_writer.writerow([
                    f"{sim_time:.5f}", n3d,
                    f"{x_rng[0]:.4f}", f"{x_rng[1]:.4f}",
                    f"{y_rng[0]:.4f}", f"{y_rng[1]:.4f}",
                    f"{z_rng[0]:.4f}", f"{z_rng[1]:.4f}",
                    f"{i_rng[0]:.4f}", f"{i_rng[1]:.4f}",
                    n2d, f"{r2d_rng[0]:.4f}", f"{r2d_rng[1]:.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}",
                    f"{speed:.4f}", f"{driver_inputs.m_throttle:.3f}",
                    f"{driver_inputs.m_steering:.3f}",
                ])
                t_hist.append(sim_time)
                pts3d_hist.append(n3d)
                speed_hist.append(speed)
                range2d_hist.append(r2d_rng[1])

                # Subsystem advance.
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                car.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
        import traceback
        traceback.print_exc()
        print(f"simulation aborted: {exc}")
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing: time-series plot from the in-memory history ===
    if t_hist:
        fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        ax0.plot(t_hist, pts3d_hist, color="tab:blue")
        ax0.set_ylabel("3D lidar hits")
        ax0.grid(True)
        ax1.plot(t_hist, speed_hist, color="tab:green")
        ax1.set_ylabel("speed (m/s)")
        ax1.grid(True)
        ax2.plot(t_hist, range2d_hist, color="tab:red")
        ax2.set_ylabel("2D lidar max range (m)")
        ax2.set_xlabel("time (s)")
        ax2.grid(True)
        fig.suptitle("ARTcar lidar telemetry")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"done: {len(t_hist)} steps logged; final time {system.GetChTime():.3f} s")


if __name__ == "__main__":
    main()
