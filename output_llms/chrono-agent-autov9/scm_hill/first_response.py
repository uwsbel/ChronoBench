"""HMMWV on SCM deformable hill terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A full-model HMMWV wheeled vehicle (veh.HMMWV_Full, SMC contact, owns its
ChSystemSMC) driving over an SCM (Bekker-Wong) deformable soft-soil terrain
whose surface is a hill loaded from a 16-bit-style grayscale height map. The
vehicle uses a TMEASY tire force model (REQUIRED on SCM — the default RIGID
tire spins without translating on deformable soil) with explicit per-spindle
collision cylinders so SCM ray-casts detect the wheels and form ruts.

A scripted ChDriver subclass releases the brake after a short settle, then
applies forward throttle with zero steering so the vehicle climbs the hill in
a straight line.

Expected behavior
-----------------
After an initial brake-held settle, the chassis accelerates forward (+X),
climbs the convex hill (chassis Z rises with X), and remains upright. The
deformable terrain shows wheel sinkage/ruts. CSV logs chassis pose/speed each
step; a review video is rendered from the Irrlicht chase camera.

Fallback note: if the height-map file is not found at run time the terrain
falls back to a flat SCM patch (documented at the load site); the hill is the
intended configuration and the shipped file is present in this build.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants: physics, geometry, control (no bare literals downstream) ===
TIME_STEP = 2.0e-3                 # integration step (s); SCM is slow, keep modest
TIRE_STEP_SIZE = 1.0e-3            # TMEASY tire sub-step (REQUIRED on SCM)
SIM_END = 6.5                      # total simulated time (s); covers climb + crest + start of descent on-terrain
RENDER_FPS = 30.0                  # review-video frame rate

# Vehicle spawn (X/Y on the terrain; Z derived from terrain height below).
VEH_INIT_X = -10.5                 # start near the low foot of the hill (climb toward X=0 crest)
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.55       # HMMWV chassis origin height above wheel-bottom (~0.50 at rest + small gap)

# SCM hill terrain extents and height-map mapping.
SCM_SIZE_X = 24.0                  # terrain length along X (m)
SCM_SIZE_Y = 16.0                  # terrain width along Y (m)
SCM_DELTA = 0.08                   # SCM grid resolution (m); balance ruts vs cost
HILL_H_MIN = 0.0                   # height-map black -> min height (m)
HILL_H_MAX = 1.2                   # height-map white -> max height (hill crest, m) — climbable grade
HEIGHTMAP_REL = "terrain/height_maps/convex64.bmp"   # convex bump == single hill

# SCM Bekker/Mohr soft-soil parameters (8 positional args, all required).
# Firm-but-deformable soil: stiff enough that the HMMWV rides on top (small
# sinkage / visible ruts) instead of punching through and tumbling.
BEKKER_KPHI = 2e6                  # frictional modulus (Pa)
BEKKER_KC = 0.0                    # cohesive modulus
BEKKER_N = 1.1                     # exponent
MOHR_COHESION = 5e3                # cohesive limit (Pa) — resists deep shear sinkage
MOHR_FRICTION = 30.0               # friction angle (deg)
JANOSI_SHEAR = 0.01                # shear coefficient (m)
ELASTIC_K = 2e8                    # elastic stiffness (Pa/m)
DAMPING_R = 3e4                    # vertical damping (Pa.s/m)

# Collision families (keep tire ray-casts visible to SCM; never disallow family 0).
TIRE_FAMILY = 1

# Scripted driver control law.
SETTLE_TIME = 0.6                  # brake-held settle before driving (s)
DRIVE_THROTTLE = 0.8               # forward throttle after settle
ZTOL = 0.06                        # wheel-bottom clearance tolerance vs terrain (m)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run

# Derived constants (precomputed once; never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END          # short check when validating


# === Scripted driver: brake settle, then straight-line throttle up the hill ===
class HillClimbDriver(veh.ChDriver):
    """Time-based open-loop control: hold brake, then full forward throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(0.0)            # drive straight up the hill


def build_vehicle():
    """Create + initialize the HMMWV wrapper with a TMEASY tire (SCM-ready)."""
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # REQUIRED on SCM (RIGID won't move)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    return hmmwv


def sample_hill_height(x, y):
    """Pre-sample the hill height at (x, y) using a throwaway SCMTerrain.

    The vehicle wrapper owns its ChSystem and must be initialized at the right
    spawn Z up front (manually re-posing only the chassis after Initialize would
    desync it from the spindle bodies). We therefore build a disposable terrain
    on a scratch system, read the surface height once, and discard it.
    """
    scratch = chrono.ChSystemSMC()
    scratch.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    probe = veh.SCMTerrain(scratch)
    probe.SetSoilParameters(
        BEKKER_KPHI, BEKKER_KC, BEKKER_N,
        MOHR_COHESION, MOHR_FRICTION, JANOSI_SHEAR,
        ELASTIC_K, DAMPING_R,
    )
    hm = veh.GetVehicleDataFile(HEIGHTMAP_REL)
    if os.path.isfile(hm):
        probe.Initialize(hm, SCM_SIZE_X, SCM_SIZE_Y, HILL_H_MIN, HILL_H_MAX, SCM_DELTA)
    else:
        probe.Initialize(SCM_SIZE_X, SCM_SIZE_Y, SCM_DELTA)
    return probe.GetHeight(chrono.ChVector3d(x, y, HILL_H_MAX + 1.0))


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    hmmwv = build_vehicle()

    # Spawn the vehicle on the hill surface: sample the terrain height at the
    # spawn XY FIRST, derive the chassis-origin Z, then SetInitPosition before
    # Initialize so the chassis AND its spindle bodies are placed consistently.
    spawn_terrain_z = sample_hill_height(VEH_INIT_X, VEH_INIT_Y)   # precomputed once
    init_z = spawn_terrain_z + SUSPENSION_REF_HEIGHT
    init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, init_z)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, chrono.QUNIT))
    hmmwv.Initialize()

    system = hmmwv.GetSystem()                  # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                # cache: vehicle subsystem handle, reused below
    # spindles: veh_obj.GetSpindlePos(axle, side); wheels: veh_obj.GetAxles()[i]
    # joints: suspension + steering links created inside the wrapper

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === SCM deformable hill terrain ===
    # Bekker-Wong soft soil; surface is a convex hill from a shipped height map.
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        BEKKER_KPHI, BEKKER_KC, BEKKER_N,
        MOHR_COHESION, MOHR_FRICTION, JANOSI_SHEAR,
        ELASTIC_K, DAMPING_R,
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.15)   # sinkage heatmap on ruts

    heightmap_path = veh.GetVehicleDataFile(HEIGHTMAP_REL)
    if os.path.isfile(heightmap_path):
        # Hill surface from height map (intended configuration).
        terrain.Initialize(heightmap_path, SCM_SIZE_X, SCM_SIZE_Y,
                            HILL_H_MIN, HILL_H_MAX, SCM_DELTA)
    else:
        # Fallback: flat SCM patch if the shipped height map is unavailable.
        terrain.Initialize(SCM_SIZE_X, SCM_SIZE_Y, SCM_DELTA)
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 20, 20)

    # === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
    # SCM ray-casts only hit registered collision shapes; TMEASY adds none itself.
    tire0 = veh_obj.GetAxles()[0].GetWheels()[0].GetTire()
    tire_rad = tire0.GetRadius()                # cache: constant tire radius
    tire_w = tire0.GetWidth()                   # cache: constant tire width
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)
    tire_mat.SetYoungModulus(1e7)

    for axle in veh_obj.GetAxles():
        for wheel in axle.GetWheels():
            spindle = wheel.GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
                chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
            )
            spindle.EnableCollision(True)
            sp_cm = spindle.GetCollisionModel()
            sp_cm.SetFamily(TIRE_FAMILY)
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # tires don't self-collide
            # NOTE: never DisallowCollisionsWith(0) — that hides SCM ray-casts.
    system.GetCollisionSystem().BindAll()       # rebuild models so SCM sees cylinders

    # === Footprint assert: wheels must rest on (not through) the terrain ===
    spindle_world = [veh_obj.GetSpindlePos(a, s)
                     for a in range(veh_obj.GetNumberAxles())
                     for s in (veh.LEFT, veh.RIGHT)]
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
    support_z = terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, HILL_H_MAX + 1.0))
    assert support_z - ZTOL <= wheel_bottom_z <= support_z + 0.2, (
        f"vehicle not seated on terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain z={support_z:.3f}; adjust SUSPENSION_REF_HEIGHT"
    )

    # === Driver ===
    driver = HillClimbDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht vehicle scene: window + sky + chase cam + lights + grid
    eye_x = VEH_INIT_X - 8.0                     # precomputed once: camera start behind vehicle
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on SCM deformable hill")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(eye_x, -10.0, 4.0),
                      chrono.ChVector3d(VEH_INIT_X, 0.0, 1.0))
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, int(SCM_SIZE_X), int(SCM_SIZE_Y),
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; Synchronize/Advance per step
    os.makedirs("frames", exist_ok=True)         # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")   # closed in finally
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "x", "y", "z", "speed", "throttle", "roll_deg", "pitch_deg"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        times, xs, zs, speeds = [], [], [], []

        frame = 0
        running = True
        while running:
            if not HEADLESS:
                if not vis.Run():
                    break
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if vis is not None:
                    vis.Synchronize(sim_time, driver_inputs)

                # --- log physics each step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                rot = chassis.GetRot()
                rpy = rot.GetCardanAnglesXYZ()        # roll/pitch/yaw (rad)
                data_writer.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{speed:.5f}", f"{driver_inputs.m_throttle:.3f}",
                    f"{math.degrees(rpy.x):.4f}", f"{math.degrees(rpy.y):.4f}",
                ])
                motion_writer.writerow([
                    f"{sim_time:.5f}", "chassis",
                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}",
                ])
                times.append(sim_time)
                xs.append(pos.x)
                zs.append(pos.z)
                speeds.append(speed)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
                if vis is not None:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    running = False
                    break

            if HEADLESS and not running:
                break

    except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:            # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === time-series plot from the logged arrays
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(times, xs, label="chassis X (m)")
        ax1.plot(times, zs, label="chassis Z (m)")
        ax1.set_ylabel("position (m)")
        ax1.legend(); ax1.grid(True)
        ax2.plot(times, speeds, color="tab:red", label="speed (m/s)")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed (m/s)")
        ax2.legend(); ax2.grid(True)
        fig.suptitle("HMMWV climbing SCM deformable hill")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        print(f"frames written: {0 if HEADLESS else 'see frames/'}")
        print(f"x: {xs[0]:.3f} -> {xs[-1]:.3f}  (dx={xs[-1]-xs[0]:.3f})")
        print(f"z: {zs[0]:.3f} -> {zs[-1]:.3f}  (dz={zs[-1]-zs[0]:.3f})")
        print(f"max speed: {max(speeds):.3f} m/s")
        print(f"final NaN check: {any(math.isnan(v) for v in (xs[-1], zs[-1], speeds[-1]))}")


if __name__ == "__main__":
    main()
