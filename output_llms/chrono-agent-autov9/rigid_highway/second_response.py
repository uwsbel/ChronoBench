"""Wheeled vehicle (HMMWV) driving straight on a rigid highway terrain.

Model
-----
- System: ChSystemNSC (rigid, non-smooth contact), gravity -Z (Z-up world).
- Moving body: a full HMMWV wheeled vehicle (chassis + 4 axles/spindles +
  suspension/steering joints) created and owned by the ``veh.HMMWV_Full``
  wrapper. TMEASY tires on a rigid road.
- Terrain: a ``veh.RigidTerrain`` carrying two patches:
    1. a large flat rigid road patch (dirt-textured) as the drivable, level
       lane the wheels ride on; the shipped highway mesh ``Highway_vis.obj`` is
       added as a non-colliding visual backdrop when available; and
    2. an additional terrain patch built from ``vehicle/terrain/meshes/bump.obj``
       located at (0, -42, 0), colored (0.5, 0.5, 0.8) and textured with
       ``dirt.jpg`` at UV scaling (6.0, 6.0).
- Driver: a scripted ``veh.ChDriver`` subclass that holds the steering centered
  (straight lane, stays on-road) and ramps the throttle up after a short
  settling phase.

Expected behavior
------------------
The HMMWV settles onto the highway, then accelerates forward in a straight line,
its forward speed and travelled distance increasing monotonically while the
chassis stays upright. The bump patch is visible off to the side of the road.
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

# === Named constants: physics, geometry, derived positions ===
TIME_STEP = 2.0e-3                      # integration step (s)
SIM_END = 10.0                          # total simulated time (s)
RENDER_FPS = 30.0                       # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps per frame

# Vehicle spawn (HMMWV origin = geometric center) on the flat road plane.
ROAD_TOP_Z = 0.0                        # highway mesh top sits at world z=0
SUSPENSION_REF_HEIGHT = 0.5             # HMMWV chassis-origin height above wheel-bottom at rest
TIRE_RADIUS = 0.46                      # HMMWV tire radius (m), used for the footprint assert
ZTOL = 0.10                             # allowed wheel-bottom clearance/overlap vs road top
VEH_INIT_X = -30.0                      # spawn near the start of the highway, drive +X
VEH_INIT_Y = 0.0
VEH_INIT_Z = ROAD_TOP_Z + SUSPENSION_REF_HEIGHT

# Additional bump terrain patch (per the patch specification).
BUMP_POS = chrono.ChVector3d(0.0, -42.0, 0.0)
BUMP_COLOR = chrono.ChColor(0.5, 0.5, 0.8)
BUMP_TEX_SCALE_U = 6.0
BUMP_TEX_SCALE_V = 6.0

# Flat fallback patch dimensions (used only if the highway mesh is missing).
FALLBACK_LENGTH = 300.0
FALLBACK_WIDTH = 120.0

# Driver schedule.
SETTLE_TIME = 0.5                       # let suspension settle before driving
THROTTLE_LEVEL = 0.7                    # forward throttle after settling

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run

# Asset paths (resolved through the Chrono data directory; no absolute paths).
HIGHWAY_COL = "synchrono/meshes/Highway_col.obj"
HIGHWAY_VIS = "synchrono/meshes/Highway_vis.obj"
BUMP_MESH = "vehicle/terrain/meshes/bump.obj"
DIRT_TEX = "vehicle/terrain/textures/dirt.jpg"
LOGO = "logo_chrono_alpha.png"


# === Driver: scripted straight-line cruise (centered steering) ===
class StraightLineDriver(veh.ChDriver):
    """Keeps steering centered and ramps throttle up after a settling phase."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Centered steering => the vehicle tracks the straight lane and does not
        # veer off the road / over a barrier.
        self.SetSteering(0.0)
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
        else:
            self.SetThrottle(THROTTLE_LEVEL)
            self.SetBraking(0.0)


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper instantiates the ChSystemNSC, the chassis rigid body, four
    # axles with spindle bodies, and the suspension/steering joints internally.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT
        )
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # rolling tire model for a rigid road
    hmmwv.SetTireStepSize(TIME_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()          # cache: main chassis body, reused every step
    veh_obj = hmmwv.GetVehicle()              # cache: low-level vehicle handle, reused
    # spindles/wheels live under veh_obj.GetAxles(); joints are the suspension +
    # steering links created inside the wrapper.

    # === Footprint assert: wheels rest on (not through) the road ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z={ROAD_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{ROAD_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain: rigid highway patch + additional bump patch ===
    terrain = veh.RigidTerrain(system)

    road_mat = chrono.ChContactMaterialNSC()
    road_mat.SetFriction(0.9)
    road_mat.SetRestitution(0.01)

    # Drivable surface: a large flat rigid road patch at z=0. This is the
    # collision surface the wheels ride on, so the vehicle tracks a straight,
    # level lane and stays upright (the shipped highway mesh below is a curved,
    # barrier-lined model whose grade and barriers would launch/flip a vehicle
    # driven straight across it, so it is used as a visual backdrop only).
    road_patch = terrain.AddPatch(
        road_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        FALLBACK_LENGTH,
        FALLBACK_WIDTH,
    )
    road_patch.SetTexture(chrono.GetChronoDataFile(DIRT_TEX), 200, 200)
    road_patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

    # Highway mesh as a non-colliding visual backdrop (when shipped).
    highway_vis_path = chrono.GetChronoDataFile(HIGHWAY_VIS)
    if os.path.isfile(highway_vis_path):
        highway_body = chrono.ChBody()
        highway_body.SetFixed(True)
        highway_body.SetPos(chrono.ChVector3d(0, 0, 0))
        highway_body.EnableCollision(False)        # visual only — wheels ride the flat patch
        highway_shape = chrono.ChVisualShapeModelFile()
        highway_shape.SetFilename(highway_vis_path)
        highway_body.AddVisualShape(highway_shape, chrono.ChFramed())
        system.AddBody(highway_body)

    # Additional terrain patch from bump.obj, placed at (0, -42, 0).
    bump_mat = chrono.ChContactMaterialNSC()
    bump_mat.SetFriction(0.9)
    bump_mat.SetRestitution(0.01)
    bump_patch = terrain.AddPatch(
        bump_mat,
        chrono.ChCoordsysd(BUMP_POS, chrono.QUNIT),
        chrono.GetChronoDataFile(BUMP_MESH),   # mesh-file overload (resolved absolute path)
        True,                       # connected mesh
        0.0,                        # sweep-sphere radius
        True,                       # visualization
    )
    bump_patch.SetColor(BUMP_COLOR)
    bump_patch.SetTexture(chrono.GetChronoDataFile(DIRT_TEX),
                          BUMP_TEX_SCALE_U, BUMP_TEX_SCALE_V)

    terrain.Initialize()

    # === Driver ===
    driver = StraightLineDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on rigid highway")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)   # follow the chassis
        vis.Initialize()                                              # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile(LOGO))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 60, 60,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    times, speeds, xs, zs = [], [], [], []
    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
    except (OSError, IOError) as exc:        # disk / permission failure opening CSVs
        print(f"Failed to open output CSV: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "speed", "throttle"])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    # === Main loop === render-cadence outer loop; vehicle Synchronize/Advance inner batch
    frame = 0
    try:
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
                data_writer.writerow(
                    [f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                     f"{pos.z:.5f}", f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}"]
                )
                motion_writer.writerow(
                    [f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                     f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"]
                )
                times.append(sim_time)
                speeds.append(speed)
                xs.append(pos.x)
                zs.append(pos.z)

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # Flush + close writers even if a step diverges.
        if data_file is not None:
            data_file.flush()
            data_file.close()
        if motion_file is not None:
            motion_file.flush()
            motion_file.close()

    # === Post-processing: timeseries plot from the logged data ===
    if times:
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(times, speeds, color="tab:blue")
        ax[0].set_ylabel("speed (m/s)")
        ax[0].grid(True)
        ax[1].plot(times, xs, color="tab:green")
        ax[1].set_ylabel("pos_x (m)")
        ax[1].grid(True)
        ax[2].plot(times, zs, color="tab:red")
        ax[2].set_ylabel("pos_z (m)")
        ax[2].set_xlabel("time (s)")
        ax[2].grid(True)
        fig.suptitle("HMMWV on rigid highway — straight-line cruise")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done: {len(times)} steps logged, final x={xs[-1]:.2f} m, "
          f"final speed={speeds[-1]:.2f} m/s" if times else "Done: no steps logged")


if __name__ == "__main__":
    main()
