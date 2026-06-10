"""Gator wheeled-vehicle simulation with an onboard chassis camera sensor.

Model
-----
A John Deere Gator UTV (PyChrono ``veh.Gator`` wrapper, NSC contact) driving on a
flat rigid terrain patch. The vehicle's component subsystems are rendered with
DIFFERENT visualization types (chassis = MESH, suspension/steering = PRIMITIVES,
wheels/tires = MESH) so the wrapper-created parts are visibly distinguished.

System type
-----------
``ChSystemNSC`` — owned internally by the ``veh.Gator`` wrapper. Terrain, sensor
manager and visualization are all attached to that single owned system.

Main bodies
-----------
- chassis rigid body (wrapper-created)
- four wheel spindles + tires (wrapper-created, TMEASY tire model)
- a flat rigid terrain patch under the vehicle

Sensors
-------
A ``sens.ChCameraSensor`` is rigidly attached to the chassis body (onboard,
forward-looking) and feeds a ChFilterVisualize + ChFilterSave + ChFilterRGBA8Access
filter chain. The sensor scene is lit with point lights + ambient light (this build's
ChScene exposes AddPointLight / SetAmbientLight, not a directional-light API). OptiX
renders only bodies that carry collision geometry, so the terrain patch is collidable.

Driving
-------
A scripted time-based driver (subclass of ``veh.ChDriver``) supplies steering /
throttle / braking each step — the autonomous, headless-safe analogue of a
hand-driven interactive controller (a keyboard driver produces zero input with no
GUI). The Gator brakes briefly, then accelerates with a gentle sinusoidal steer.

Expected behavior
-----------------
The Gator pulls away from rest and travels several metres forward while weaving
gently; the chassis stays upright. Chassis pose/velocity are logged to CSV and the
onboard camera writes RGB frames.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants (geometry / physics / derived once) ===
TIME_STEP = 1.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 50.0                  # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: physics steps per frame

TERRAIN_LENGTH = 100.0             # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0              # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                # top surface height of the flat patch (m)

GATOR_REF_HEIGHT = 0.5             # chassis-origin height above wheel-bottom at rest (m)
INIT_X = 0.0                       # spawn X (m)
INIT_Y = 0.0                       # spawn Y (m)
INIT_Z = TERRAIN_TOP_Z + GATOR_REF_HEIGHT          # derived spawn height
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)        # identity (facing +X)

CAM_W, CAM_H = 1280, 720           # sensor image resolution
CAM_FOV = 1.408                    # horizontal field of view (rad)
CAM_UPDATE_RATE = 1.0 / TIME_STEP  # sensor tick rate (Hz)
CAM_OFFSET = chrono.ChVector3d(1.5, 0.0, 0.6)      # onboard camera offset on chassis (m)

ITER_DIR = os.path.dirname(os.path.abspath(__file__))  # resolve outputs next to this script
FRAMES_DIR = os.path.join(ITER_DIR, "frames")
CAM_DIR = os.path.join(ITER_DIR, "cam")
SIM_CSV = os.path.join(ITER_DIR, "simulation_data.csv")
MOTION_CSV = os.path.join(CAM_DIR, "motion_log.csv")
PLOT_PNG = os.path.join(ITER_DIR, "simulation_timeseries.png")

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating


# === Driver (scripted, headless-safe analogue of an interactive controller) ===
class ScriptedDriver(veh.ChDriver):
    """Time-based control law: brake briefly, then accelerate while weaving.

    A keyboard ``ChInteractiveDriver`` yields zero input in a windowless run,
    so this scripted subclass drives the vehicle deterministically instead.
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.3 * math.sin(0.5 * time))  # gentle sinusoidal weave


def main():
    os.makedirs(FRAMES_DIR, exist_ok=True)   # guard against missing output dir
    os.makedirs(CAM_DIR, exist_ok=True)
    os.makedirs(os.path.join(CAM_DIR, "chassis_cam"), exist_ok=True)  # ChFilterSave needs an existing dir

    # === Vehicle (Gator wrapper owns the ChSystemNSC + all sub-bodies/joints) ===
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    gator.SetTireType(veh.TireModelType_TMEASY)   # prompt: rolling tire on rigid road
    gator.SetTireStepSize(TIRE_STEP)
    gator.Initialize()

    # Visualization types differ per subsystem so the parts are visibly distinct.
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Gator wrapper) ===
    system = gator.GetSystem()                 # ChSystemNSC owned by the wrapper
    veh_obj = gator.GetVehicle()               # underlying ChWheeledVehicle
    chassis = gator.GetChassisBody()           # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
    # links created inside the wrapper; terrain patch body added below.

    # Footprint sanity: wheel bottoms must rest on (not through) the rigid patch.
    TIRE_RADIUS = veh_obj.GetAxle(0).GetWheels()[0].GetTire().GetRadius()
    ZTOL = 0.08
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs support top z={TERRAIN_TOP_Z:.3f}; raise GATOR_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid patch under the vehicle) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted time-based control) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Sensor manager + onboard chassis camera ===
    manager = sens.ChSensorManager(system)
    # ChScene in this build exposes point lights + ambient (no directional-light API).
    manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 50), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(-20, -20, 50), chrono.ChColor(0.8, 0.8, 0.8), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    # Camera rides on the chassis body, looking forward (+X) and slightly down.
    cam_forward = chrono.ChVector3d(1, 0, -0.1).GetNormalized()  # precomputed once
    cam_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), cam_forward)
    camera = sens.ChCameraSensor(
        chassis,                                       # attach to chassis -> follows the vehicle
        CAM_UPDATE_RATE,
        chrono.ChFramed(CAM_OFFSET, cam_quat),
        CAM_W, CAM_H, CAM_FOV,
    )
    camera.SetName("chassis_cam")
    if not HEADLESS:
        camera.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))   # live preview window
    camera.PushFilter(sens.ChFilterSave(os.path.join(CAM_DIR, "chassis_cam") + "/"))  # PNG frames
    camera.PushFilter(sens.ChFilterRGBA8Access())                 # frame-buffer access
    manager.AddSensor(camera)

    # === Visualization (full Irrlicht vehicle scene; skipped on the validation run) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator UTV + onboard camera")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.0), 6.0, 0.5)
        vis.Initialize()                                   # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid
        vis.AttachVehicle(veh_obj)

    # === Main loop (render-cadence outer loop; Synchronize/Advance the subsystem stack) ===
    sim_f = None
    motion_f = None
    try:
        sim_f = open(SIM_CSV, "w", newline="")           # main physics log
        motion_f = open(MOTION_CSV, "w", newline="")     # chassis pose/velocity log
        sim_w = csv.writer(sim_f)
        motion_w = csv.writer(motion_f)
        sim_w.writerow(["time", "x", "y", "z", "speed", "throttle", "steering", "braking"])
        motion_w.writerow(["time", "x", "y", "z", "vx", "vy", "vz", "speed"])

        frame = 0
        step = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(os.path.join(FRAMES_DIR, f"img_{frame:06d}.png"))  # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()

                # Synchronize the subsystem stack (driver -> terrain -> vehicle -> vis).
                # Synchronize the driver FIRST, then read inputs, so the vehicle and the
                # HUD reflect THIS step's control law (not a one-step-stale value).
                driver.Synchronize(sim_time)
                driver_inputs = driver.GetInputs()
                terrain.Synchronize(sim_time)
                gator.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                manager.Update()   # pump the sensor every physics step (sees post-step pose)

                # Log physics each step.
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                sim_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}",
                                f"{driver_inputs.m_steering:.4f}", f"{driver_inputs.m_braking:.4f}"])
                motion_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}"])

                # Advance the subsystem stack (gator.Advance steps the wrapper-owned system).
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                gator.Advance(TIME_STEP)
                if not HEADLESS:
                    vis.Advance(TIME_STEP)
                step += 1
                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        if sim_f is not None:
            sim_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (plot the logged time series) ===
    try:
        with open(SIM_CSV, "r", newline="") as f:   # re-read for plotting
            reader = csv.DictReader(f)
            rows = list(reader)
        if rows:
            t = np.array([float(r["time"]) for r in rows])
            x = np.array([float(r["x"]) for r in rows])
            spd = np.array([float(r["speed"]) for r in rows])
            steer = np.array([float(r["steering"]) for r in rows])

            fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
            ax[0].plot(t, x, color="tab:blue"); ax[0].set_ylabel("x (m)"); ax[0].grid(True)
            ax[1].plot(t, spd, color="tab:green"); ax[1].set_ylabel("speed (m/s)"); ax[1].grid(True)
            ax[2].plot(t, steer, color="tab:red"); ax[2].set_ylabel("steering"); ax[2].set_xlabel("time (s)"); ax[2].grid(True)
            fig.suptitle("Gator UTV — chassis motion")
            fig.tight_layout()
            fig.savefig(PLOT_PNG, dpi=110)
            plt.close(fig)
    except (OSError, IOError) as exc:               # plot/output failure should not mask a good run
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
