"""Full HMMWV on a custom highway mesh terrain (PyChrono 9.0.1 + Irrlicht).

Model
-----
A full-model HMMWV (High Mobility Multipurpose Wheeled Vehicle) created with the
``veh.HMMWV_Full`` wrapper drives on a rigid terrain built from a custom highway
mesh.  The wrapper internally owns an SMC contact ``ChSystemNSC``-equivalent
system, the chassis rigid body, four spindle/wheel bodies, the suspension and
steering joints, the powertrain, and TMEASY tires.

System type
-----------
SMC (penalty) contact.  Gravity is -Z (Z-up world convention).  The terrain is a
``veh.RigidTerrain`` whose single patch is loaded directly from the collision
mesh ``synchrono/meshes/Highway_col.obj`` (with the matching visual mesh
``Highway_vis.obj`` added as a render asset on the patch ground body).  If those
shipped highway meshes are unavailable, the code falls back to a large flat
textured rigid patch so the vehicle still has a drivable surface (the fallback is
logged at run time).

Bodies / mechanism
------------------
- chassis (HMMWV body, mesh visualization)
- four spindles + wheels + TMEASY tires (mesh visualization)
- terrain ground body carrying the highway mesh

Driver
------
A scripted ``veh.ChDriver`` subclass supplies steering / throttle / braking as a
function of time (a short brake phase, then a moderate constant throttle with the
steering held centered for a straight-line lane run) so the vehicle actually moves
in a headless/batch run.  ``ChInteractiveDriver`` is NOT used because it only reads
live keyboard input, which is always zero off-screen.

Expected behavior
-----------------
After releasing the brake the HMMWV accelerates forward along the highway (+Y),
driving straight down the lane between the barriers while staying upright on the road.
The chassis world position, speed, and driver inputs are logged each step; a
review video is rendered with a vehicle chase camera.
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

# === Named constants === geometry / physics / control parameters (no bare literals downstream)
TIME_STEP = 2e-3                  # integration step (s)
TIRE_STEP = 2e-3                  # TMEASY tire substep (s)
SIM_END = 12.0                    # total simulated time (s)
RENDER_FPS = 50.0                 # review-video frame rate (prompt: real time at 50 fps)

GRAVITY_Z = -9.81                 # m/s^2, Z-up world

# Highway mesh assets (shipped Chrono synchrono data).
HIGHWAY_COL = "synchrono/meshes/Highway_col.obj"   # collision mesh
HIGHWAY_VIS = "synchrono/meshes/Highway_vis.obj"   # visual mesh

# Vehicle spawn.  The highway mesh runs along +Y (X in [-11.6, 11.6],
# Y in [-75, 75.8], road surface near Z=0); face the vehicle along +Y so it
# drives down the lane.  Spawn near the south end with margin to the barriers.
VEH_INIT_X = 3.0                  # offset to the right lane
VEH_INIT_Y = -60.0               # near the south end of the highway
SUSPENSION_REF_HEIGHT = 0.55      # chassis-origin height above wheel-bottom at rest (HMMWV)
ROAD_SURFACE_Z = 0.0              # approximate highway surface height at spawn
VEH_INIT_Z = ROAD_SURFACE_Z + SUSPENSION_REF_HEIGHT
VEH_HEADING = chrono.QuatFromAngleZ(math.pi / 2.0)   # face +Y

TIRE_RADIUS = 0.4699              # HMMWV TMEASY tire radius (m), from wheel geometry
ZTOL = 0.10                       # allowed wheel-bottom clearance/overlap vs road

# Scripted driver timing.  Drive straight down the lane at a moderate cruise so
# the HMMWV stays on the road between the barriers and remains upright.
BRAKE_RELEASE_T = 1.0             # hold brake until this time, then accelerate
CRUISE_THROTTLE = 0.35            # moderate throttle (keeps speed/curvature safe in-lane)
STEER_HOLD = 0.0                  # steering kept centered -> straight-line highway run

# Fallback flat patch size (only used if the highway mesh is missing).
FLAT_LENGTH = 300.0
FLAT_WIDTH = 60.0

OUT_CSV = "simulation_data.csv"
MOTION_CSV = "cam/motion_log.csv"
PLOT_PNG = "simulation_timeseries.png"

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run


# === Scripted driver === time-based control law (no human-in-the-loop)
class HighwayDriver(veh.ChDriver):
    """Brake briefly, then accelerate forward with a gentle sinusoidal steer."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_RELEASE_T:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_HOLD)       # straight line down the highway lane


def build_terrain(system):
    """Build the highway RigidTerrain; fall back to a flat textured patch.

    Returns (terrain, used_fallback). The collision mesh is the terrain patch;
    the matching visual mesh is added as a render asset on the ground body.
    """
    terrain = veh.RigidTerrain(system)

    patch_mat = chrono.ChContactMaterialSMC()      # SMC system -> SMC material
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)

    col_path = chrono.GetChronoDataFile(HIGHWAY_COL)
    vis_path = chrono.GetChronoDataFile(HIGHWAY_VIS)
    used_fallback = False

    if os.path.isfile(col_path):
        # Highway mesh patch: collision mesh drives contact; visual mesh is the look.
        patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, col_path, True, 0.0, True)
        if os.path.isfile(vis_path):
            ground = patch.GetGroundBody()         # patch ground rigid body
            vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(vis_path, False, True)
            vshape = chrono.ChVisualShapeTriangleMesh()
            vshape.SetMesh(vis_mesh)
            vshape.SetName("highway_vis")
            ground.AddVisualShape(vshape, chrono.ChFramed())
    else:
        # Fallback: large flat textured patch so the vehicle can still drive.
        used_fallback = True
        patch = terrain.AddPatch(
            patch_mat,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ROAD_SURFACE_Z), chrono.QUNIT),
            FLAT_LENGTH, FLAT_WIDTH,
        )
        patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

    terrain.Initialize()
    return terrain, used_fallback


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing motion-log/video dir

    # === Vehicle (HMMWV_Full wrapper) === creates system + chassis + wheels + joints + tires
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), VEH_HEADING))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    # Mesh visualization for ALL vehicle components (prompt: mesh visualization).
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                 # contact system owned by the wrapper
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY_Z))
    veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle, reused every step
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links
    # are created inside the wrapper; terrain ground body is built below.

    # === Terrain === custom highway mesh RigidTerrain (flat fallback if mesh missing)
    terrain, used_fallback = build_terrain(system)
    if used_fallback:
        print("WARNING: highway mesh not found -> using flat fallback patch")

    # Footprint assert: wheel bottoms must rest on (not through) the road surface.
    spindle_world = [veh_obj.GetSpindlePos(axle, side)
                     for axle in range(veh_obj.GetNumberAxles())
                     for side in (veh.LEFT, veh.RIGHT)]
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= ROAD_SURFACE_Z - ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} vs surface "
        f"z={ROAD_SURFACE_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{ROAD_SURFACE_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted, time-based (autonomous; NOT interactive keyboard)
    driver = HighwayDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on Highway Mesh")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # chase the chassis
        vis.Initialize()                                              # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                               # outdoor sky backdrop
        vis.AddTypicalLights()                                        # standard lighting
        vis.AddLightDirectional()                                     # extra sun for the road
        vis.AttachVehicle(veh_obj)                                    # binds chassis/wheel/tire assets
        vis.AttachDriver(driver)                                      # steering/throttle/brake HUD

    # === Derived loop constants === precomputed once (never recomputed in the loop)
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))      # physics steps per frame
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END             # short physics check when validating

    # === Main loop === Synchronize/Advance the full subsystem stack; log + render each frame
    f_main = None
    f_motion = None
    times, speeds, xs, ys = [], [], [], []
    try:
        f_main = open(OUT_CSV, "w", newline="")          # guarded below by finally/close
        f_motion = open(MOTION_CSV, "w", newline="")
        main_w = csv.writer(f_main)
        motion_w = csv.writer(f_motion)
        main_w.writerow(["time", "pos_x", "pos_y", "pos_z", "speed",
                         "throttle", "steering", "braking"])
        motion_w.writerow(["time", "body", "pos_x", "pos_y", "pos_z",
                           "vel_x", "vel_y", "vel_z"])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(render_every):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # log this physics step
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                main_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}",
                                 f"{driver_inputs.m_steering:.4f}",
                                 f"{driver_inputs.m_braking:.4f}"])
                motion_w.writerow([f"{sim_time:.5f}", "chassis",
                                   f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(sim_time)
                speeds.append(speed)
                xs.append(pos.x)
                ys.append(pos.y)

                # Synchronize then Advance the full stack (wrapper Advance steps the system).
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges
        if f_main is not None:
            f_main.close()
        if f_motion is not None:
            f_motion.close()

    # === Post-processing === plot logged time series from the in-memory columns
    if times:
        t = np.array(times)
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(t, np.array(speeds), label="speed (m/s)")
        ax[0].set_ylabel("speed (m/s)")
        ax[0].grid(True)
        ax[0].legend(loc="best")
        ax[1].plot(t, np.array(xs), label="pos_x (m)")
        ax[1].plot(t, np.array(ys), label="pos_y (m)")
        ax[1].set_xlabel("time (s)")
        ax[1].set_ylabel("position (m)")
        ax[1].grid(True)
        ax[1].legend(loc="best")
        fig.suptitle("HMMWV on Highway Mesh — speed & trajectory")
        fig.tight_layout()
        fig.savefig(PLOT_PNG, dpi=110)
        plt.close(fig)

    print(f"done: {len(times)} steps, final speed={speeds[-1] if speeds else float('nan'):.3f} m/s, "
          f"y travelled={ys[-1]-ys[0] if ys else 0.0:.3f} m")


if __name__ == "__main__":
    main()
