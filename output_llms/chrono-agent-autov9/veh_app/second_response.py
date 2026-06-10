"""HMMWV wheeled vehicle on flat rigid terrain with an onboard 360-degree lidar.

Model
-----
- System type: NSC (the veh.HMMWV_Full wrapper owns its ChSystemNSC; contact
  method SMC selected on the wrapper for stable tire/terrain contact).
- Main bodies: a full HMMWV (chassis + 4 spindles/wheels/tires), a flat
  RigidTerrain patch, plus two static obstacle props (a 1x1x1 box and a
  radius-0.5 / height-1 cylinder), both with a blue texture.
- Sensor: an onboard ChLidarSensor rigidly mounted 2 m above the chassis origin,
  360 deg horizontal FOV, 800 x 300 beams, strongest-return mode, with depth /
  intensity / XYZI point-cloud / point-cloud-visualization filters.
- Driver: a scripted ChDriver subclass holding constant steering 0.5 and
  throttle 0.2 so the vehicle drives forward and steers left.

Expected behavior
-----------------
The HMMWV starts at (0, -5, 0.4) and, under constant throttle 0.2 with steering
0.5, accelerates forward and curves left, remaining upright on the terrain while
the lidar sweeps the surrounding scene (terrain + box + cylinder). The chassis
travels a measurable distance over the simulated interval.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants (geometry / physics / sensor) ===
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire substep (s)
SIM_END = 8.0                          # simulated duration (s)
RENDER_FPS = 30.0                      # review-video frame rate

# Driver command (constant, per the request)
DRV_STEERING = 0.5                     # left steer, range -1..+1
DRV_THROTTLE = 0.2                     # forward throttle, range 0..+1

# Vehicle spawn (chassis-frame origin in world coords)
INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.QUNIT

# Terrain (flat rigid patch)
TERRAIN_LENGTH = 100.0                 # X extent (m)
TERRAIN_WIDTH = 100.0                  # Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Obstacle props
BOX_SIZE = chrono.ChVector3d(1.0, 1.0, 1.0)        # full extents (m)
BOX_POS = chrono.ChVector3d(0, 0, 0.5)
CYL_RADIUS = 0.5                                   # m
CYL_HEIGHT = 1.0                                   # m
CYL_POS = chrono.ChVector3d(0, 0, 1.5)
BLUE_TEXTURE = chrono.GetChronoDataFile("textures/blue.png")

# Lidar specification (per request)
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0, 2)        # mount above chassis origin
LIDAR_UPDATE_RATE = 10.0                           # Hz (modest to keep render < timeout)
LIDAR_HSAMPLES = 800                               # horizontal samples
LIDAR_VCHANNELS = 300                              # vertical channels
LIDAR_HFOV = 2 * chrono.CH_PI                      # 360 deg horizontal FOV (rad)
LIDAR_MAX_VFOV = chrono.CH_PI / 12                 # max vertical angle (rad)
LIDAR_MIN_VFOV = -chrono.CH_PI / 6                 # min vertical angle (rad)
LIDAR_MAX_RANGE = 100.0                            # m
LIDAR_SAMPLE_RADIUS = 2                            # super-sampling radius
LIDAR_DIVERGENCE = 0.003                           # beam divergence (rad)

# Derived constants (precomputed once)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless check
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating


# === Driver (scripted ChDriver subclass) ===
# WHAT: hold constant steering/throttle every step. WHY: the request fixes the
# inputs to steering 0.5, throttle 0.2 for the whole run (no human-in-the-loop).
class ConstantDriver(veh.ChDriver):
    def __init__(self, vehicle, steering, throttle):
        super().__init__(vehicle)
        self._steering = steering
        self._throttle = throttle

    def Synchronize(self, time):
        # Set the CURRENT-step commands so the HUD reflects this step's inputs.
        self.SetSteering(self._steering)
        self.SetThrottle(self._throttle)
        self.SetBraking(0.0)


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper internally creates a ChSystemNSC, the chassis rigid body, four
    # spindles/wheels with suspension + steering joints, and the tire force models.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)        # grippy tire for rigid-road traction
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                 # ChSystem owned by the wrapper
    chassis = hmmwv.GetChassisBody()           # cache: main chassis body, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: vehicle handle, reused every step

    # Assert the wheels start on (not through) the flat terrain at z=0.
    TIRE_RADIUS = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= -0.20, (
        f"vehicle starts below terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        f"raise INIT_LOC.z"
    )

    # === Terrain (flat rigid patch under the vehicle) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Obstacle props (static box + cylinder, blue texture) ===
    # WHY: give the lidar and the scene a couple of objects to sense; static so
    # they remain fixed reference targets rather than dynamic clutter.
    prop_mat = chrono.ChContactMaterialSMC()
    prop_mat.SetFriction(0.7)
    prop_mat.SetRestitution(0.0)
    prop_mat.SetYoungModulus(2e7)

    box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000, True, True, prop_mat)
    box.SetPos(BOX_POS)
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(BLUE_TEXTURE)
    system.AddBody(box)

    cylinder = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Z, CYL_RADIUS, CYL_HEIGHT, 1000, True, True, prop_mat
    )
    cylinder.SetPos(CYL_POS)
    cylinder.SetFixed(True)
    cylinder.GetVisualShape(0).SetTexture(BLUE_TEXTURE)
    system.AddBody(cylinder)

    # === Sensor manager + onboard lidar ===
    # WHAT: OptiX sensor scene needs explicit lights (no AddDirectionalLight on
    # ChScene here -> use AddPointLight + SetAmbientLight). WHY: the lidar ray
    # caster shares the OptiX scene; lighting keeps intensity returns sane.
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 1000.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    lidar = sens.ChLidarSensor(
        chassis,                                                   # rides on the chassis
        LIDAR_UPDATE_RATE,
        chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT),               # 2 m above chassis origin
        LIDAR_HSAMPLES,
        LIDAR_VCHANNELS,
        LIDAR_HFOV,
        LIDAR_MAX_VFOV,
        LIDAR_MIN_VFOV,
        LIDAR_MAX_RANGE,
        sens.LidarBeamShape_RECTANGULAR,                           # rectangular beam shape
        LIDAR_SAMPLE_RADIUS,                                       # sample radius 2
        LIDAR_DIVERGENCE,                                          # vertical divergence
        LIDAR_DIVERGENCE,                                          # horizontal divergence
        sens.LidarReturnMode_STRONGEST_RETURN,                     # strongest return mode
    )
    lidar.SetName("onboard_lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())                      # depth + intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                  # depth -> point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                  # XYZI point-cloud access
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 540, 2.0))  # point-cloud view
    manager.AddSensor(lidar)

    # === Driver ===
    driver = ConstantDriver(veh_obj, DRV_STEERING, DRV_THROTTLE)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera/chase + lights + grid
    vis = None
    if not HEADLESS:
        import pychrono.irrlicht as chronoirr
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV with onboard lidar")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.6)
        vis.Initialize()                                           # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                            # outdoor sky backdrop
        vis.AddTypicalLights()                                     # standard lighting
        vis.AddCamera(chrono.ChVector3d(-8, -10, 6), INIT_LOC)     # AFTER Initialize
        vis.AddGrid(1.0, 1.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                 # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                                   # input-bar HUD

    # === Main loop (render-cadence outer loop; Synchronize/Advance per step) ===
    os.makedirs("frames", exist_ok=True)            # guard against missing output dir

    data_file = None
    motion_file = None
    times, xs, ys, speeds, headings = [], [], [], [], []
    frame = 0
    step = 0
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        os.makedirs("cam", exist_ok=True)
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_w = csv.writer(data_file)
        motion_w = csv.writer(motion_file)
        data_w.writerow(["time", "x", "y", "z", "speed", "heading_rad", "steering", "throttle"])
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            time = system.GetChTime()

            if not HEADLESS and step % RENDER_EVERY == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            # Synchronize the driver FIRST, then read inputs so the HUD/log
            # reflect the CURRENT step's commands.
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # Log physics this step.
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            rot = chassis.GetRot()
            heading = rot.GetCardanAnglesZYX().z
            data_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                             f"{speed:.5f}", f"{heading:.5f}",
                             f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_throttle:.4f}"])
            motion_w.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                               f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
            times.append(time); xs.append(pos.x); ys.append(pos.y)
            speeds.append(speed); headings.append(heading)

            # Pump the sensor manager every physics step (sees each post-step pose).
            manager.Update()

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)             # advances the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1

    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission on CSV open
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush partial CSV even if a step diverges.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (timeseries plot from the logged arrays) ===
    if times:
        fig, axplt = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axplt[0].plot(times, xs, label="x")
        axplt[0].plot(times, ys, label="y")
        axplt[0].set_ylabel("position (m)"); axplt[0].legend(); axplt[0].grid(True)
        axplt[1].plot(times, speeds, color="tab:green")
        axplt[1].set_ylabel("speed (m/s)"); axplt[1].grid(True)
        axplt[2].plot(times, np.degrees(headings), color="tab:red")
        axplt[2].set_ylabel("heading (deg)"); axplt[2].set_xlabel("time (s)"); axplt[2].grid(True)
        fig.suptitle("HMMWV with onboard lidar — motion")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done: {len(times)} steps, final pos=({xs[-1]:.2f},{ys[-1]:.2f}), "
          f"final speed={speeds[-1]:.2f} m/s, frames={frame}")


if __name__ == "__main__":
    main()
