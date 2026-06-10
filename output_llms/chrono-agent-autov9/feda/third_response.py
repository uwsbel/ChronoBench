"""FEDA wheeled vehicle on grass-textured rigid terrain with an onboard FPV camera sensor.

Model
-----
- A FEDA (Ford Expedition Defense / Army) wheeled vehicle built with the
  ``veh.FEDA`` wrapper. The wrapper creates and owns its own SMC ``ChSystem``,
  the chassis rigid body, the four suspension/steering/spindle assemblies, the
  TMEASY tires, and the engine + transmission powertrain.
- A flat ``veh.RigidTerrain`` patch textured with the shipped grass texture so
  the vehicle drives over a grass surface.
- A scripted ``veh.ChDriver`` subclass that accelerates the vehicle forward with
  a gentle steering sweep so the first-person camera shows the scene streaming
  past.

Sensing
-------
- A ``sens.ChSensorManager`` oversees the scene's sensors. Point lights plus an
  ambient term illuminate the sensor scene so the camera image is well exposed.
- An RGB ``sens.ChCameraSensor`` is rigidly mounted on the vehicle chassis body
  pointing forward, giving a first-person (driver) view. It uses a high
  resolution (1920x1080) and an ~85 deg horizontal field of view.
- A ``ChFilterVisualize`` filter renders the camera image to a live preview
  window; ``ChFilterSave`` writes PNG frames and ``ChFilterRGBA8Access`` exposes
  the frame buffer. The sensor is registered with the manager, which is updated
  once per physics step in the main loop so the camera tracks the moving vehicle.

Visualization & output
-----------------------
Irrlicht (``veh.ChWheeledVehicleVisualSystemIrrlicht`` with a chase camera) is the
standard review renderer; the OptiX camera sensor is the demo subject. Physics
quantities are logged to ``simulation_data.csv`` and the chassis pose/velocity to
``cam/motion_log.csv``; a time-series plot is written to
``simulation_timeseries.png``. Expected behavior: the FEDA accelerates from rest
and translates forward over the grass terrain, the camera image staying populated.

System type: SMC (vehicle wrapper default for FEDA).
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")            # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants (geometry / physics / sensor) ===
TIME_STEP = 1.0e-3               # integration step (s)
TIRE_STEP = 1.0e-3               # tire model sub-step (s)
SIM_END = 6.0                    # total simulated time (s)
RENDER_FPS = 30.0                # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: steps per frame

GRAVITY_Z = -9.81                # m/s^2, Z-up world

TERRAIN_LENGTH = 200.0           # m, X extent (enlarged so the vehicle stays on it while moving/turning)
TERRAIN_WIDTH = 200.0            # m, Y extent
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG = 2.0e7            # SMC contact stiffness (Pa)

INIT_X, INIT_Y = 0.0, 0.0        # vehicle spawn X/Y (world)
SUSPENSION_REF_HEIGHT = 0.5      # FEDA chassis-origin height above wheel-bottom at rest (m)
TERRAIN_TOP_Z = 0.0              # flat patch top sits at z = 0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis spawn height
TIRE_RADIUS = 0.499              # FEDA TMEASY tire radius (m), used for wheel-bottom assert
ZTOL = 0.10                      # allowed wheel-bottom clearance/overlap vs terrain top

# Onboard FPV camera sensor parameters
CAM_RES_W, CAM_RES_H = 1920, 1080            # high resolution
CAM_FOV = 85.0 * math.pi / 180.0             # ~85 deg horizontal FOV (rad), appropriate for FPV
# OptiX renders one image per camera tick; ticking at the physics rate (1000 Hz) makes a
# high-res FPV render the wall-clock bottleneck. Tick at the review frame rate instead.
CAM_UPDATE_RATE = RENDER_FPS                 # Hz, camera image cadence
CAM_OFFSET = chrono.ChVector3d(1.6, 0.0, 1.1)   # forward+up on the chassis -> driver-like POV
LIGHT_INTENSITY = 1.5            # point-light intensity so the sensor scene is well lit

# Validation gate: fast windowless physics check when SIMBENCH_VALIDATE is set.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short bounded sim when validating


# === Scripted driver (forward accel + gentle steering sweep) ===
class FpvDriver(veh.ChDriver):
    """Open-loop driver: ramp throttle up, sweep steering slowly for a moving FPV."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Ease throttle in over the first second, then hold; gentle steering sweep.
        if time < 1.0:
            self.SetThrottle(0.4 * time)        # ramp 0 -> 0.4
        else:
            self.SetThrottle(0.5)
        self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))   # slow left/right sweep


def main():
    # === Vehicle (FEDA wrapper owns the system + bodies + powertrain) ===
    feda = veh.FEDA()
    feda.SetContactMethod(chrono.ChContactMethod_SMC)
    feda.SetChassisCollisionType(veh.CollisionType_NONE)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    # TMEASY tire: stable, well-conditioned slip model for the rigid road patch
    # (the shipped FEDA Pacejka file is an older version that diverges here).
    feda.SetTireType(veh.TireModelType_TMEASY)
    feda.SetTireStepSize(TIRE_STEP)
    feda.Initialize()

    feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.FEDA wrapper) ===
    sys = feda.GetSystem()                       # ChSystemSMC owned by the wrapper
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY_Z))
    chassis = feda.GetChassisBody()              # cache: main chassis rigid body, reused every step
    veh_obj = feda.GetVehicle()                  # cache: ChWheeledVehicle handle, reused every step
    # spindles/wheels: veh_obj.GetAxle(i)...; joints: suspension + steering links inside the wrapper

    # Footprint assert: wheels must rest on (not through) the terrain top.
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

    # === Terrain (flat rigid patch with grass texture) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass surface
    patch.SetColor(chrono.ChColor(0.4, 0.6, 0.3))
    terrain.Initialize()

    # === Driver (scripted forward accel + steering sweep) ===
    driver = FpvDriver(veh_obj)
    driver.Initialize()

    # === Sensor manager + scene lighting ===
    manager = sens.ChSensorManager(sys)
    # Point lights illuminate the sensor scene so the camera image is well exposed.
    manager.scene.AddPointLight(
        chrono.ChVector3f(50, 50, 100),
        chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
        1000.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-50, -50, 100),
        chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
        1000.0,
    )
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # fill light (ChVector3f, not ChColor)

    # === Onboard FPV camera sensor (mounted on the chassis, looking forward) ===
    # Identity offset rotation -> camera local +X (its view axis) points along chassis +X (forward).
    fpv_cam = sens.ChCameraSensor(
        chassis,                                         # rides on the chassis -> first-person view
        CAM_UPDATE_RATE,                                 # Hz
        chrono.ChFramed(CAM_OFFSET, chrono.QUNIT),       # forward/up offset on the chassis
        CAM_RES_W, CAM_RES_H,                            # high resolution
        CAM_FOV,                                         # ~85 deg horizontal FOV
    )
    fpv_cam.SetName("fpv_camera")
    fpv_cam.PushFilter(sens.ChFilterVisualize(CAM_RES_W, CAM_RES_H))   # render image to a preview window
    fpv_cam.PushFilter(sens.ChFilterSave("cam/fpv/"))                  # PNG frames -> mp4 by RUN stage
    fpv_cam.PushFilter(sens.ChFilterRGBA8Access())                     # frame-buffer access
    manager.AddSensor(fpv_cam)                                         # register with the manager

    # === Visualization (Irrlicht chase-camera window; full scene block) ===
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("FEDA FPV camera on grass terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)    # chase cam behind the chassis
        vis.Initialize()                                              # Initialize FIRST (Irrlicht order)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                               # outdoor sky backdrop
        vis.AddTypicalLights()                                        # standard window lighting
        vis.AddCamera(chrono.ChVector3d(-10, -8, 4), chrono.ChVector3d(INIT_X, INIT_Y, 1.0))
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.35, 0.45, 0.30))                 # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                                      # input-bar HUD

    # === Main loop (render-cadence; sensor + physics every step) ===
    os.makedirs("frames", exist_ok=True)         # guard against missing output dir for review frames
    os.makedirs("cam", exist_ok=True)            # guard against missing output dir for sensor + csv

    data_f = None
    motion_f = None
    try:
        data_f = open("simulation_data.csv", "w", newline="")            # disk/permission guarded below
        motion_f = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:             # disk full / permission denied on CSV open
        print(f"Could not open CSV output: {exc}")
        raise

    times, speeds, xpos = [], [], []              # for the post-run plot
    try:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "x", "y", "z", "speed", "throttle", "steering"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz", "speed"])

        step = 0
        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            time = sys.GetChTime()

            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            driver_inputs = driver.GetInputs()

            # Subsystem synchronize (driver -> terrain -> vehicle -> vis)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            feda.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            # Inner physics batch: advance once per step + pump the sensor manager.
            for _ in range(RENDER_EVERY):
                manager.Update()                  # update the camera so it follows the moving chassis

                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                spd = vel.Length()
                data_w.writerow([f"{sys.GetChTime():.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{spd:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}", f"{driver_inputs.m_steering:.4f}"])
                motion_w.writerow([f"{sys.GetChTime():.5f}", "chassis",
                                   f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{spd:.5f}"])
                times.append(sys.GetChTime()); speeds.append(spd); xpos.append(pos.x)

                # Subsystem advance (driver/terrain/vehicle/vis). feda.Advance steps the
                # wrapper-owned system -> do NOT also call sys.DoStepDynamics.
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                feda.Advance(TIME_STEP)
                if not HEADLESS:
                    vis.Advance(TIME_STEP)
                step += 1
                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:     # solver divergence / bad simulation state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverged mid-run.
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (time-series plot from the logged data) ===
    if times:
        fig, ax1 = plt.subplots(figsize=(9, 5))
        ax1.plot(times, speeds, color="tab:blue", label="speed (m/s)")
        ax1.set_xlabel("time (s)")
        ax1.set_ylabel("speed (m/s)", color="tab:blue")
        ax2 = ax1.twinx()
        ax2.plot(times, xpos, color="tab:red", label="x position (m)")
        ax2.set_ylabel("x position (m)", color="tab:red")
        fig.suptitle("FEDA chassis speed and forward travel")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as pf:   # context-managed image write
            fig.savefig(pf, dpi=110)
        plt.close(fig)

    print(f"Done: {len(times)} steps logged, final x={xpos[-1]:.2f} m, "
          f"final speed={speeds[-1]:.2f} m/s" if times else "Done: no steps logged")


if __name__ == "__main__":
    main()
