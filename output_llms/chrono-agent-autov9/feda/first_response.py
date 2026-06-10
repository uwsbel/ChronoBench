"""FEDA wheeled vehicle on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A catalog FEDA (Ford Expedition-class) wheeled vehicle is created through the
``veh.FEDA`` wrapper, which builds and owns an SMC (smooth-contact) ChSystem
containing the chassis rigid body, four suspension corners with spindles, the
steering linkage, the powertrain (engine + transmission), and the wheel/tire
subsystems. The vehicle is initialized at a chosen world location/orientation
with a PAC02 (Pacejka) tire model and MESH visualization for every part.

Environment
-----------
A single flat ``veh.RigidTerrain`` patch (SMC contact material, custom dirt
texture) provides the driving surface at z = 0.

Control
-------
A scripted ``veh.ChDriver`` subclass plays the role of the interactive driver
(steering / throttle / braking) without any human-in-the-loop keyboard input,
so the run is reproducible headless: a short brake settle, then steady throttle
with a gentle sinusoidal steering sweep.

Expected behavior
-----------------
After release the vehicle accelerates forward along +X and weaves slightly under
the sinusoidal steering, remaining upright on the rigid patch. The render loop
runs at 50 frames/second, advancing the full vehicle subsystem stack and saving
review frames; physics state is logged to CSV every step.
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# === Named constants: timing, geometry, physics ===
TIME_STEP = 2e-3                       # integration step (s)
SIM_END = 10.0                         # simulated duration (s)
RENDER_FPS = 50.0                      # review-video frame rate (per prompt)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps per frame

TERRAIN_LENGTH = 200.0                 # rigid patch X size (m)
TERRAIN_WIDTH = 200.0                  # rigid patch Y size (m)
TERRAIN_TOP_Z = 0.0                    # patch top plane (m)

INIT_X = 0.0                           # spawn X (m)
INIT_Y = 0.0                           # spawn Y (m)
INIT_HEADING = 0.0                     # spawn yaw (rad), facing +X
SUSPENSION_REF_HEIGHT = 0.5            # chassis-origin height above wheel-bottom at rest (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT  # derived chassis spawn height
ZTOL = 0.15                            # allowed wheel-bottom clearance/overlap vs terrain (m)

# Scripted driver schedule (replaces interactive keyboard input in headless runs).
RELEASE_TIME = 1.0                     # brake-held settle window (s)
CRUISE_THROTTLE = 0.6                  # steady throttle after release
STEER_AMPLITUDE = 0.25                 # peak steering (-1..1)
STEER_FREQ = 0.25                      # steering sine frequency (Hz)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating


# === Scripted driver (interactive-style control, headless-safe) ===
class ScriptedDriver(veh.ChDriver):
    """Time-based steering / throttle / braking law via ChDriver setters."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < RELEASE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * STEER_FREQ * time))


def main():
    # === Vehicle wrapper: system + bodies created by veh.FEDA ===
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.QuatFromAngleZ(INIT_HEADING)

    vehicle = veh.FEDA()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetTireType(veh.TireModelType_PAC02)         # prompt: tire model on rigid road
    vehicle.SetTireStepSize(TIME_STEP)
    vehicle.Initialize()

    # Mesh visualization for ALL vehicle parts (per prompt).
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # Enumerate wrapper-created essentials into named locals (visible to reader/reviewer).
    sys = vehicle.GetSystem()                      # ChSystemSMC owned by the FEDA wrapper
    chassis = vehicle.GetChassisBody()             # cache: main chassis rigid body, reused every step
    veh_obj = vehicle.GetVehicle()                 # cache: ChWheeledVehicle, reused every step
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links (wrapper-internal)

    # === Footprint assert: wheels rest on (not through) the terrain at spawn ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    tire_radius = veh_obj.GetAxle(0).GetWheels()[0].GetTire().GetRadius()  # cache: from tire JSON
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
    )

    # === Terrain: flat rigid patch with a custom dirt texture ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)  # custom texture
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver: scripted stand-in for the interactive steering/throttle/brake ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + chase camera + sky + lights + logo
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("FEDA on Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.6)  # follow from behind/above
        vis.Initialize()                                   # Initialize FIRST (Irrlicht)
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVector3d(-8, -8, 4), chrono.ChVector3d(0, 0, 0.5))
        vis.AddGrid(1.0, 1.0, 60, 60,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                           # steering/throttle/brake HUD bars

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

    data_path = "simulation_data.csv"
    motion_path = os.path.join("cam", "motion_log.csv")
    times, speeds, xs = [], [], []         # for the post-run timeseries plot

    try:
        data_f = open(data_path, "w", newline="")
        motion_f = open(motion_path, "w", newline="")
    except (OSError, IOError) as exc:      # disk full / permission denied
        print(f"Cannot open CSV output: {exc}")
        raise

    try:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "x", "y", "z", "speed", "throttle", "steering", "braking"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        # === Main loop: render-cadence outer, Synchronize/Advance inner ===
        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                # Log physics every step.
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}",
                                 f"{driver_inputs.m_steering:.4f}",
                                 f"{driver_inputs.m_braking:.4f}"])
                motion_w.writerow([f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}",
                                   f"{pos.y:.5f}", f"{pos.z:.5f}", f"{vel.x:.5f}",
                                   f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(sim_time)
                speeds.append(speed)
                xs.append(pos.x)

                # Advance the full subsystem stack (vehicle.Advance steps the system).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                vehicle.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                vehicle.Advance(TIME_STEP)        # internally steps the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if sys.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # Flush + close writers even if a step diverges mid-run.
        data_f.close()
        motion_f.close()

    # === Post-processing: timeseries plot from the logged data ===
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax1.plot(times, speeds, color="tab:blue")
        ax1.set_ylabel("speed (m/s)")
        ax1.grid(True)
        ax2.plot(times, xs, color="tab:orange")
        ax2.set_ylabel("x position (m)")
        ax2.set_xlabel("time (s)")
        ax2.grid(True)
        fig.suptitle("FEDA on rigid terrain")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as pf:   # ensure file handle closes
            fig.savefig(pf, format="png", dpi=110)
        plt.close(fig)

    print(f"Done: t={times[-1] if times else 0.0:.3f}s, "
          f"final speed={speeds[-1] if speeds else 0.0:.3f} m/s, "
          f"final x={xs[-1] if xs else 0.0:.3f} m")


if __name__ == "__main__":
    main()
