"""HMMWV wheeled vehicle driving on a single mesh-based RigidTerrain patch.

Model
-----
* Vehicle : veh.HMMWV_Full (SMC contact), AWD driveline, TMEASY tires,
  spawned at world position (6, -70, 0.5).
* Terrain : a SINGLE veh.RigidTerrain patch defined by a triangle-mesh surface
  (collision enabled, contact thickness 0.01). The same surface mesh is also
  attached to the terrain ground body as an explicit ChVisualShapeTriangleMesh
  so the road has a textured visual representation.
* Driver  : a scripted veh.ChDriver subclass (brief brake, then modest forward
  throttle with a gentle steering sweep) so the run stays within the patch
  extents and does not drive off into empty space.

System type
-----------
ChSystemSMC owned by the HMMWV_Full wrapper (penalty contact). Gravity -Z, Z-up.

Expected behavior
-----------------
The HMMWV settles onto the mesh terrain, then accelerates forward a modest
distance while remaining upright. CSV logs chassis pose/speed every step and the
script renders a chase-camera review video via Irrlicht.
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


# === Named constants === geometry / physics / control (no bare literals downstream)
TIME_STEP = 2.0e-3                       # integrator step (s)
SIM_END = 12.0                           # total simulated time (s)
RENDER_FPS = 30.0                        # review-video frame rate

# Vehicle spawn (world frame) — requested initial position.
VEH_INIT_X = 6.0
VEH_INIT_Y = -70.0
VEH_INIT_Z = 0.5
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.QuatFromAngleZ(math.pi / 2.0)   # face +Y (along the terrain length)

# Terrain mesh surface (single patch). Collision contact thickness as requested.
TERRAIN_MESH = "vehicle/terrain/meshes/test.obj"   # large drivable surface mesh
TERRAIN_THICKNESS = 0.01                            # contact mesh thickness (m)
TERRAIN_FRICTION = 0.9                              # patch material friction
TERRAIN_RESTITUTION = 0.01                          # patch material restitution
TERRAIN_TEXTURE = "vehicle/terrain/textures/dirt.jpg"

# Tire geometry for the wheel-bottom support assertion.
TIRE_RADIUS = 0.46                       # HMMWV tire radius (m), approx
ZTOL = 0.30                              # allowed wheel-bottom clearance vs terrain top

# Control schedule.
BRAKE_UNTIL = 0.6                        # hold brake while the chassis settles (s)
DRIVE_THROTTLE = 0.45                    # modest throttle to stay within patch
STEER_AMPLITUDE = 0.12                   # gentle steering sweep amplitude
STEER_RATE = 0.4                         # steering sweep angular rate (rad/s)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

CSV_MAIN = "simulation_data.csv"
CSV_MOTION = "cam/motion_log.csv"
PLOT_PATH = "simulation_timeseries.png"


# === Driver === scripted ChDriver subclass (time-based open-loop control)
class ScriptedDriver(veh.ChDriver):
    """Brief brake to settle, then modest throttle with a gentle steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_UNTIL:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * (time - BRAKE_UNTIL)))


def build_terrain_visual_mesh(ground_body):
    """Create and attach a visual triangle-mesh surface to the terrain ground body.

    Mirrors the "add a visual mesh to the terrain" requirement: load the surface
    mesh and wrap it in a ChVisualShapeTriangleMesh on the existing ground body.
    """
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(TERRAIN_MESH), True, True)
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(vis_mesh)
    vis_shape.SetName("terrain_visual_mesh")
    vis_shape.SetMutable(False)
    ground_body.AddVisualShape(vis_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing output dir

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper internally builds the ChSystemSMC, the chassis rigid body, the
    # four suspension/spindle sub-assemblies, the steering links, and the
    # powertrain. We enumerate the real handles below so they are explicit.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire for rigid road
    hmmwv.SetTireStepSize(TIME_STEP)
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()             # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()       # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()           # cache: vehicle subsystem handle, reused every step
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering inside wrapper

    # === Terrain === single mesh patch (collision) + explicit visual mesh on ground body
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                              # mesh already in world coords
        chrono.GetChronoDataFile(TERRAIN_MESH),       # collision surface mesh
        True,                                         # connected mesh
        TERRAIN_THICKNESS,                            # contact thickness
    )
    patch.SetColor(chrono.ChColor(0.7, 0.7, 0.6))
    build_terrain_visual_mesh(patch.GetGroundBody())   # add ChVisualShapeTriangleMesh
    patch.SetTexture(chrono.GetChronoDataFile(TERRAIN_TEXTURE), 200, 200)
    terrain.Initialize()                               # initialize after the patch is added

    # === Footprint assertion === wheel bottoms must rest on (not through) the mesh
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    terrain_top_z = terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 100.0))
    assert wheel_bottom_z >= terrain_top_z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={terrain_top_z:.3f}; raise VEH_INIT_Z"
    )

    # === Driver === scripted open-loop control
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht vehicle scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("HMMWV on single mesh RigidTerrain")
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0.0), chrono.QUNIT),
                    chrono.ChColor(0.35, 0.35, 0.35))   # ground reference grid at spawn
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; Synchronize/Advance subsystem stack
    main_file = None
    motion_file = None
    times, speeds, xs, ys, zs = [], [], [], [], []
    try:
        main_file = open(CSV_MAIN, "w", newline="")
        motion_file = open(CSV_MOTION, "w", newline="")
        main_writer = csv.writer(main_file)
        motion_writer = csv.writer(motion_file)
        main_writer.writerow(["time", "x", "y", "z", "speed", "throttle", "steering"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        step = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS and step % RENDER_EVERY == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            # log physics each step
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            main_writer.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                  f"{pos.z:.5f}", f"{speed:.5f}",
                                  f"{driver_inputs.m_throttle:.4f}",
                                  f"{driver_inputs.m_steering:.4f}"])
            motion_writer.writerow([f"{sim_time:.5f}", "chassis",
                                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
            times.append(sim_time)
            speeds.append(speed)
            xs.append(pos.x)
            ys.append(pos.y)
            zs.append(pos.z)

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned ChSystemSMC
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1
    except (OSError, IOError) as exc:                 # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:         # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges
        if main_file is not None:
            main_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot chassis kinematics vs time
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        ax1.plot(times, speeds, label="speed (m/s)", color="tab:blue")
        ax1.set_ylabel("speed (m/s)")
        ax1.grid(True)
        ax1.legend(loc="best")
        ax2.plot(times, xs, label="x", color="tab:red")
        ax2.plot(times, ys, label="y", color="tab:green")
        ax2.plot(times, zs, label="z", color="tab:purple")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("position (m)")
        ax2.grid(True)
        ax2.legend(loc="best")
        fig.suptitle("HMMWV on single mesh RigidTerrain")
        fig.tight_layout()
        fig.savefig(PLOT_PATH, dpi=110)
        plt.close(fig)

    print(f"Done. steps_logged={len(times)} final_x={xs[-1]:.3f} final_y={ys[-1]:.3f} "
          f"final_speed={speeds[-1]:.3f}")


if __name__ == "__main__":
    main()
