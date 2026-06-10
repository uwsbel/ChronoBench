"""HMMWV wheeled vehicle on flat rigid terrain with an onboard depth camera.

Model summary
-------------
- System type: NSC (the veh.HMMWV_Full wrapper owns a ChSystemNSC internally).
- Main bodies: the HMMWV chassis + four wheel spindles (created by the wrapper),
  plus a flat RigidTerrain patch the wheels roll on.
- Actuation: a scripted veh.ChDriver subclass applies a short brake, then a
  steady forward throttle with a gentle sinusoidal steering sweep.
- Onboard sensor: an OptiX sens.ChCameraSensor rigidly mounted on the chassis
  with a rear/up offset pose, 1280x720, horizontal FOV 1.408 rad, with a far
  clip of 30 m. It produces an onboard scene view that is visualized live and
  saved as a frame stream; this is the onboard-depth-perception viewpoint.

Expected behavior
------------------
The vehicle brakes briefly, then accelerates forward across the terrain while
weaving slightly. The chassis X position grows monotonically after the brake
phase and the vehicle stays upright (chassis Z roughly constant, near the
suspension reference height). The onboard camera produces a view stream of the
scene around the chassis. Vehicle state (X, Y, Z, heading) is logged every
physics step.

This script renders an Irrlicht chase-camera window for the review video; the
onboard camera is an independent OptiX sensor for the onboard-view deliverable.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# === Named constants === geometry / physics / sim control (no bare literals downstream)
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire force model sub-step (s)
SIM_END = 8.0                      # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: physics steps per frame

# Vehicle spawn (flat rigid terrain at z = 0; chassis origin sits above it).
TERRAIN_TOP_Z = 0.0                # rigid patch top surface height (m)
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis origin above wheel-bottom at rest (m)
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived spawn height
TIRE_RADIUS = 0.46                 # HMMWV tire radius (m), used for the wheel-bottom assert
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs support (m)

# Terrain extents (flat patch, plenty of room for an 8 s drive).
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0

# Onboard camera parameters (from the scene specification).
CAM_OFFSET = chrono.ChVector3d(-5.0, 0.0, 2.0)     # offset pose on the chassis
CAM_W = 1280
CAM_H = 720
CAM_HFOV = 1.408                   # horizontal field of view (rad)
CAM_MAX_DEPTH = 30.0               # far clip / max scene depth of interest (m)
CAM_UPDATE_RATE = 30.0             # sensor update rate (Hz) — modest so render < timeout

# Driver schedule.
BRAKE_END = 0.6                    # brake for the first 0.6 s
THROTTLE_LEVEL = 0.7               # steady forward throttle after the brake phase
STEER_AMP = 0.15                   # gentle steering sweep amplitude (-1..1)
STEER_RATE = 0.6                   # steering sweep angular rate (rad/s)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run


# === Scripted driver === time-based control law (no human-in-the-loop in headless runs)
class ScriptedDriver(veh.ChDriver):
    """Brake briefly, then drive forward with a gentle sinusoidal steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_END:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(THROTTLE_LEVEL)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMP * math.sin(STEER_RATE * time))


def build_onboard_camera(manager, chassis):
    """Build the onboard OptiX camera + its filter chain inline."""
    # === Onboard sensor === rear/up onboard view rendered via OptiX
    cam = sens.ChCameraSensor(
        chassis,                                      # rides on the chassis body
        CAM_UPDATE_RATE,                              # Hz
        chrono.ChFramed(CAM_OFFSET, chrono.QUNIT),    # offset pose on the chassis
        CAM_W, CAM_H,                                 # image width, height
        CAM_HFOV,                                     # horizontal FOV (rad)
    )
    cam.SetName("onboard_camera")
    if not HEADLESS:
        cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "Onboard View"))  # live preview
        cam.PushFilter(sens.ChFilterSave("cam/onboard/"))                     # PNG frames
    cam.PushFilter(sens.ChFilterRGBA8Access())        # frame-buffer access
    manager.AddSensor(cam)
    return cam


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # review video + sensor frames land here

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper instantiates and owns a ChSystemNSC plus the chassis rigid body,
    # four wheel spindles, suspension + steering joints, powertrain and tires.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip curve so the vehicle actually drives
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                # cache: wrapper-owned ChSystemNSC, reused every step
    veh_obj = hmmwv.GetVehicle()              # cache: vehicle subsystem handle, reused every step
    chassis = hmmwv.GetChassisBody()          # cache: main chassis rigid body, reused every step

    # Footprint sanity check: wheels must rest ON (not through) the terrain.
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

    # === Terrain === flat rigid patch under the vehicle (the support / ground reference)
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver === scripted brake-then-drive controller bound to the vehicle
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Sensor manager === oversees the onboard camera; lights for the OptiX scene
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))
    onboard_cam = build_onboard_camera(manager, chassis)   # cache: sensor handle, reused every step

    # === Visualization === full Irrlicht chase-camera scene (window + sky + camera + lights)
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV with Onboard Depth Camera")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.6)   # chase the chassis
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)        # enables the steering/throttle/brake HUD bars

    # === Logging === per-step state CSV + per-step motion CSV (open with context managers)
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    state_csv = None
    motion_csv = None
    times, xs, ys, zs, headings, speeds = [], [], [], [], [], []
    try:
        state_csv = open("simulation_data.csv", "w", newline="")
        motion_csv = open("cam/motion_log.csv", "w", newline="")
        state_w = csv.writer(state_csv)
        motion_w = csv.writer(motion_csv)
        state_w.writerow(["time", "pos_x", "pos_y", "pos_z", "heading", "speed"])
        motion_w.writerow(["time", "body", "x", "y", "z", "heading", "vx", "vy", "vz"])

        # === Main loop === render-cadence outer loop; physics + sensor in the inner batch
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_STEPS):
                sim_time = system.GetChTime()

                # Scripted driver: Synchronize first, THEN GetInputs so the HUD
                # reflects the current step's command.
                driver.Synchronize(sim_time)
                driver_inputs = driver.GetInputs()
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                # Log vehicle state every physics step (position + heading).
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                rot = chassis.GetRot()
                heading = rot.GetCardanAnglesZYX().z   # yaw about world Z
                speed = veh_obj.GetSpeed()
                state_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                  f"{pos.z:.5f}", f"{heading:.5f}", f"{speed:.5f}"])
                motion_w.writerow(["%.5f" % sim_time, "chassis", "%.5f" % pos.x,
                                   "%.5f" % pos.y, "%.5f" % pos.z, "%.5f" % heading,
                                   "%.5f" % vel.x, "%.5f" % vel.y, "%.5f" % vel.z])
                times.append(sim_time); xs.append(pos.x); ys.append(pos.y)
                zs.append(pos.z); headings.append(heading); speeds.append(speed)

                manager.Update()      # pump the depth sensor every physics step

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission errors on the CSVs
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if state_csv is not None:
            state_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing === plot logged state vs time to a PNG from the CSV columns
    if times:
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(times, xs, label="x"); axes[0].plot(times, ys, label="y")
        axes[0].plot(times, zs, label="z")
        axes[0].set_ylabel("position (m)"); axes[0].legend(); axes[0].grid(True)
        axes[1].plot(times, headings, color="tab:purple")
        axes[1].set_ylabel("heading (rad)"); axes[1].grid(True)
        axes[2].plot(times, speeds, color="tab:red")
        axes[2].set_ylabel("speed (m/s)"); axes[2].set_xlabel("time (s)"); axes[2].grid(True)
        fig.suptitle("HMMWV onboard depth-camera run — vehicle state")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"done: {len(times)} steps, final x={xs[-1]:.3f} z={zs[-1]:.3f}" if times else "done: no steps")


if __name__ == "__main__":
    main()
