"""HMMWV wheeled vehicle on a multi-patch rigid terrain (PyChrono 9.0.1, Irrlicht).

Model
-----
A full HMMWV wheeled-vehicle wrapper (SMC contact, TMEASY tires) drives over a
``veh.RigidTerrain`` composed of FOUR independently-placed patches, each with its
own contact material and visual texture:

  * Patch 1 — flat box patch (concrete texture)   centered at (-20,  5, 0.0)
  * Patch 2 — flat box patch (tile texture)        centered at ( 20, -5, 0.2)
  * Patch 3 — triangular-mesh patch (grass)        centered at (  5,-45, 0.0)
  * Patch 4 — height-map patch (dirt)              centered at ( 10, 40, 0.0)

The vehicle spawns on Patch 1 and is driven forward (modest throttle, mild
steering) by a scripted ``veh.ChDriver`` subclass so it stays inside the Patch-1
footprint. The simulation is purely rigid multi-body contact dynamics; expected
behavior is a stable forward roll across the patch with the chassis remaining
upright. Physics outputs (chassis pose, speed, driver inputs) are logged to CSV
and plotted; per-frame images feed a review video.
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

# === Named constants (geometry / physics) ===
# All physics-critical values are fixed here; positions are derived from them.
TIME_STEP = 2e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # tire model sub-step (s)
SIM_END = 8.0                    # simulated duration (s)
RENDER_FPS = 30.0                # review-video frame rate

GRAVITY = -9.81                  # m/s^2, world Z-up

# Final patch placements (world XYZ, meters) — each patch its own material/texture.
PATCH1_POS = chrono.ChVector3d(-20.0,  5.0, 0.0)   # flat box (spawn patch)
PATCH2_POS = chrono.ChVector3d( 20.0, -5.0, 0.2)   # flat box (raised)
PATCH3_POS = chrono.ChVector3d(  5.0, -45.0, 0.0)  # mesh patch
PATCH4_POS = chrono.ChVector3d( 10.0,  40.0, 0.0)  # height-map patch

PATCH1_LEN, PATCH1_WID = 40.0, 40.0   # flat box extents (m)
PATCH2_LEN, PATCH2_WID = 30.0, 30.0
PATCH_HM_LEN, PATCH_HM_WID = 64.0, 64.0   # height-map patch extents (m)
PATCH_HM_HMIN, PATCH_HM_HMAX = 0.0, 1.0   # height-map vertical range (m)

FRICTION = 0.9
RESTITUTION = 0.01
YOUNG_MODULUS = 2e7              # SMC stiffness (Pa)

SUSPENSION_REF_HEIGHT = 0.5      # HMMWV chassis-origin height above wheel-bottom at rest (m)
TIRE_RADIUS = 0.46               # approximate HMMWV tire radius (m), used for footprint assert
ZTOL = 0.10                      # allowed wheel-bottom clearance/overlap vs patch top (m)

# Vehicle spawn: on Patch 1, near its trailing edge, heading +X across the patch.
VEH_INIT_X = PATCH1_POS.x - 12.0                     # start near the -X side of Patch 1
VEH_INIT_Y = PATCH1_POS.y
VEH_INIT_Z = PATCH1_POS.z + SUSPENSION_REF_HEIGHT    # derived chassis-origin height
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.QUNIT                              # heading along +X

# Precomputed once (never recomputed in the hot loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

DATA_TERRAIN_TEX = "vehicle/terrain/textures/"
MESH_FILE = "vehicle/terrain/meshes/test.obj"
HEIGHTMAP_FILE = "vehicle/terrain/height_maps/bump64.bmp"


# === Scripted driver (veh.ChDriver subclass) ===
# Time-based open-loop control: brief settle, then forward throttle with mild
# steering. Drives the vehicle through Set* setters per the ChDriver contract.
class MultiPatchDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)        # settle on the patch first
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.4)        # modest throttle keeps it within patch extents
            self.SetBraking(0.0)
        self.SetSteering(0.15 * math.sin(0.4 * time))   # gentle weave


def build_patch_material():
    # cache: contact-material factory; one SMC material per patch (rigid contact)
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(FRICTION)
    mat.SetRestitution(RESTITUTION)
    mat.SetYoungModulus(YOUNG_MODULUS)
    return mat


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
    os.makedirs("cam", exist_ok=True)       # guard against missing output dir for motion log

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper owns the ChSystemSMC, the chassis rigid body, four wheel
    # spindles + tires, and the suspension/steering joints. We fetch real handles
    # into named locals so the system-init and body-creation are visible.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip curve for rigid-terrain driving
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem handle, reused every step
    # spindles: veh_obj.GetSpindlePos(axle, side) ; joints: suspension + steering links inside wrapper

    # === Terrain: four rigid patches, each own material + texture ===
    terrain = veh.RigidTerrain(system)

    # Patch 1 — flat box (vehicle spawns here).
    patch1 = terrain.AddPatch(build_patch_material(),
                              chrono.ChCoordsysd(PATCH1_POS, chrono.QUNIT),
                              PATCH1_LEN, PATCH1_WID)
    patch1.SetTexture(chrono.GetChronoDataFile(DATA_TERRAIN_TEX + "concrete.jpg"), 20, 20)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

    # Patch 2 — flat box, raised 0.2 m.
    patch2 = terrain.AddPatch(build_patch_material(),
                              chrono.ChCoordsysd(PATCH2_POS, chrono.QUNIT),
                              PATCH2_LEN, PATCH2_WID)
    patch2.SetTexture(chrono.GetChronoDataFile(DATA_TERRAIN_TEX + "tile4.jpg"), 20, 20)
    patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.5))

    # Patch 3 — triangular mesh patch.
    patch3 = terrain.AddPatch(build_patch_material(),
                              chrono.ChCoordsysd(PATCH3_POS, chrono.QUNIT),
                              chrono.GetChronoDataFile(MESH_FILE))
    patch3.SetTexture(chrono.GetChronoDataFile(DATA_TERRAIN_TEX + "grass.jpg"), 20, 20)

    # Patch 4 — height-map patch.
    patch4 = terrain.AddPatch(build_patch_material(),
                              chrono.ChCoordsysd(PATCH4_POS, chrono.QUNIT),
                              chrono.GetChronoDataFile(HEIGHTMAP_FILE),
                              PATCH_HM_LEN, PATCH_HM_WID,
                              PATCH_HM_HMIN, PATCH_HM_HMAX)
    patch4.SetTexture(chrono.GetChronoDataFile(DATA_TERRAIN_TEX + "dirt.jpg"), 20, 20)

    terrain.Initialize()   # build all patches AFTER they are all added

    # Footprint assert: wheels must rest on (not through) the spawn patch top.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= PATCH1_POS.z - ZTOL, (
        f"vehicle sinks into Patch 1: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs patch top z={PATCH1_POS.z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{PATCH1_POS.z - wheel_bottom_z:.3f} m"
    )

    # === Driver ===
    driver = MultiPatchDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full vehicle-aware Irrlicht scene: window + sky + chase camera + lights + logo
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on multi-patch rigid terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)   # chase cam behind chassis
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddLightWithShadow(chrono.ChVector3d(-20.0, 30.0, 40.0),
                               chrono.ChVector3d(-20.0, 5.0, 0.0),
                               60, 20, 80, 40, 512)
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; physics advanced in inner batch
    sim_data = open("simulation_data.csv", "w", newline="")   # primary physics log
    motion_log = open("cam/motion_log.csv", "w", newline="")  # per-body motion contract log
    sim_writer = csv.writer(sim_data)
    motion_writer = csv.writer(motion_log)
    sim_writer.writerow(["time", "x", "y", "z", "speed",
                         "throttle", "steering", "braking"])
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    times, xs, ys, zs, speeds = [], [], [], [], []
    frame = 0
    try:
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics this step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                sim_writer.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                     f"{pos.z:.5f}", f"{speed:.5f}",
                                     f"{driver_inputs.m_throttle:.4f}",
                                     f"{driver_inputs.m_steering:.4f}",
                                     f"{driver_inputs.m_braking:.4f}"])
                motion_writer.writerow([f"{time:.5f}", "chassis",
                                        f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                        f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(time)
                xs.append(pos.x); ys.append(pos.y); zs.append(pos.z)
                speeds.append(speed)

                driver.Synchronize(time)
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close partial CSV even if a step diverges
        sim_data.flush(); sim_data.close()
        motion_log.flush(); motion_log.close()

    # === Post-processing === plot the logged time series
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        t = np.array(times)
        ax1.plot(t, np.array(xs), label="x")
        ax1.plot(t, np.array(ys), label="y")
        ax1.plot(t, np.array(zs), label="z")
        ax1.set_ylabel("chassis position (m)")
        ax1.legend(); ax1.grid(True)
        ax2.plot(t, np.array(speeds), color="tab:red", label="speed")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed (m/s)")
        ax2.legend(); ax2.grid(True)
        fig.suptitle("HMMWV on multi-patch rigid terrain")
        fig.tight_layout()
        with open("simulation_timeseries.png", "wb") as imgf:   # explicit file handle for the plot
            fig.savefig(imgf, format="png", dpi=110)
        plt.close(fig)

    print(f"done: {len(times)} steps, {frame} frames, "
          f"final x={xs[-1] if xs else float('nan'):.3f}, "
          f"final speed={speeds[-1] if speeds else float('nan'):.3f}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, IOError) as exc:   # disk / permission failure on output dirs or CSV
        import traceback
        traceback.print_exc()
        raise
