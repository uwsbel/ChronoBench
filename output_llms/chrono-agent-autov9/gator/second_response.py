"""Gator wheeled vehicle driving across a multi-patch rigid terrain.

Model
-----
A Chrono::Vehicle ``veh.Gator`` (small electric utility vehicle) is driven in a
straight line across a ``veh.RigidTerrain`` that is composed of FOUR distinct
patches, each with its own surface texture:

  * patch 0 (start)  : flat DIRT       — spawn / acceleration zone
  * patch 1          : flat GRASS      — second surface, different texture
  * patch 2          : SLOPE heightmap — an inclined ramp used to test the
                       vehicle's gradability (its ability to climb a grade)
  * patch 3          : flat CONCRETE   — final surface, different texture

A small rigid BUMP body is placed on the patches so the vehicle's suspension is
excited as it rolls over an obstacle (a speed-bump style perturbation).

System / contact
----------------
The Gator wrapper owns a single ``ChSystemNSC`` (NSC contact). RigidTerrain and
the bump body are attached to that wrapper-owned system. Tires are TMEASY so the
vehicle develops real grip on the rigid road and can climb the slope.

Expected behavior
------------------
Starting from rest on the dirt patch, the Gator accelerates forward (+X), crosses
the grass patch, climbs the slope-heightmap patch (gaining elevation — the
gradability test), rides over the bump (a momentary pitch/heave of the chassis),
and continues onto the concrete patch. The chassis stays upright (world Z up)
throughout. Position, speed and chassis height are logged to CSV and plotted.
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

# === Constants === geometry / physics / patch layout (no bare literals downstream)
TIME_STEP = 1.0e-3              # integration step (s) — small to avoid heightmap tunneling
TIRE_STEP = 1.0e-3             # tire substep (s)
SIM_END = 22.0                 # simulation duration (s)
RENDER_FPS = 30.0              # review-video frame rate

# Headless validation gate: a fast, windowless physics check (no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # set by the validation run

# --- Terrain patch layout (RigidTerrain is the union of 4 patches along +X) ---
# Layout along travel (+X):  dirt -> grass -> SLOPE(heightmap) -> concrete(at slope top).
# The slope heightmap ramps its own footprint from its low (-X) edge up to its
# high (+X) edge, so the concrete patch is raised to SLOPE_HMAX to receive the
# vehicle at the top of the climb (gradability test).
PATCH_LEN = 30.0               # X length of each flat patch (m)
PATCH_WID = 20.0               # Y width of every patch (m)
PATCH_THICK = 1.0              # patch slab thickness (m)
# Patch centers tile end-to-end along +X (spacing = PATCH_LEN).
PATCH0_CX = 0.0                # dirt   (spawn)
PATCH1_CX = PATCH0_CX + PATCH_LEN     # grass
PATCH2_CX = PATCH1_CX + PATCH_LEN     # slope heightmap (gradability)
PATCH3_CX = PATCH2_CX + PATCH_LEN     # concrete (elevated to slope top)
# Slope patch elevation mapping (heightmap black->hMin, white->hMax).
SLOPE_HMIN = 0.0
SLOPE_HMAX = 2.0               # ramp rises 2 m over its length -> gentle climbable grade
TERRAIN_TOP_Z = 0.0            # top of the flat dirt/grass patches at the spawn
# Slope low edge sits at TERRAIN_TOP_Z; its high edge (and the concrete) at +SLOPE_HMAX.
CONCRETE_TOP_Z = TERRAIN_TOP_Z + SLOPE_HMAX

# --- Bump obstacle (a speed-bump style rigid body on the grass patch) ---
BUMP_HALF_X = 0.25             # half length along travel (m)
BUMP_HALF_Y = PATCH_WID / 2.0  # spans the patch width (m)
BUMP_HALF_Z = 0.08             # bump height above the road (m)
BUMP_CX = PATCH1_CX            # centered on the grass patch
BUMP_DENSITY = 2000.0

# --- Vehicle spawn (front-of-patch-0, on the dirt) ---
VEH_INIT_X = PATCH0_CX - PATCH_LEN / 2.0 + 4.0   # 4 m in from the start edge
VEH_INIT_Y = 0.0
GATOR_REF_HEIGHT = 0.55        # chassis-origin height above wheel-bottom at rest
VEH_INIT_Z = TERRAIN_TOP_Z + GATOR_REF_HEIGHT
TIRE_RADIUS = 0.285            # Gator tire radius (m), for the footprint assert
ZTOL = 0.15                    # allowed wheel-bottom clearance vs support top

# --- Driver schedule (closed loop on speed: hold a moderate cruise) ---
TARGET_SPEED = 5.0             # m/s — moderate so the wheels track the heightmap surface
THROTTLE_GAIN = 0.6            # proportional throttle gain on the speed error

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
RUN_END = min(SIM_END, 1.0) if HEADLESS else SIM_END           # short check when validating


# === Scripted driver === closed-loop speed hold (no human-in-the-loop)
class GatorDriver(veh.ChDriver):
    """Proportional speed controller: throttle to hold TARGET_SPEED, drive straight.

    A capped, moderate speed keeps the wheels in continuous contact with the
    slope heightmap (a high-speed approach can tunnel through the thin surface).
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)
        self._veh = vehicle        # cache: vehicle handle for the speed getter

    def Synchronize(self, time):
        err = TARGET_SPEED - self._veh.GetSpeed()
        throttle = max(0.0, min(1.0, THROTTLE_GAIN * err))
        self.SetThrottle(throttle)
        self.SetBraking(0.0)
        self.SetSteering(0.0)          # straight-line gradability run


def main():
    # === Vehicle (creates & owns its ChSystemNSC + chassis/spindle bodies) ===
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
                           chrono.QUNIT)
    )
    gator.SetTireType(veh.TireModelType_TMEASY)   # grip on rigid road + slope
    gator.SetTireStepSize(TIRE_STEP)
    gator.Initialize()

    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Gator wrapper) ===
    system = gator.GetSystem()              # ChSystemNSC owned by the wrapper
    chassis = gator.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = gator.GetVehicle()            # cache: vehicle subsystem handle
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension +
    # steering links created inside the wrapper; terrain patches added below.

    # === Terrain === one RigidTerrain made of 4 textured patches (+ slope heightmap)
    terrain = veh.RigidTerrain(system)

    def make_material():
        # Fresh NSC material per patch (shared owner pointers must not be reused).
        m = chrono.ChContactMaterialNSC()
        m.SetFriction(0.9)
        m.SetRestitution(0.01)
        return m

    # patch 0 — flat dirt (spawn / acceleration)
    patch0 = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(PATCH0_CX, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    patch0.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 20, 20)

    # patch 1 — flat grass (different texture)
    patch1 = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(PATCH1_CX, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    patch1.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 20, 20)

    # patch 2 — SLOPE heightmap (gradability ramp). The heightmap overload maps
    # the image grayscale to [SLOPE_HMIN, SLOPE_HMAX] over a LEN x WID footprint.
    patch2 = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(PATCH2_CX, 0, TERRAIN_TOP_Z), chrono.QUNIT),
        veh.GetVehicleDataFile("terrain/height_maps/slope.bmp"),
        PATCH_LEN, PATCH_WID, SLOPE_HMIN, SLOPE_HMAX,
    )
    patch2.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 20, 20)

    # patch 3 — flat concrete (different texture), raised to the slope top so it
    # catches the vehicle at the crest of the gradability climb.
    patch3 = terrain.AddPatch(
        make_material(),
        chrono.ChCoordsysd(chrono.ChVector3d(PATCH3_CX, 0, CONCRETE_TOP_Z), chrono.QUNIT),
        PATCH_LEN, PATCH_WID, PATCH_THICK,
    )
    patch3.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 20, 20)

    terrain.Initialize()

    # === Bump body === a rigid speed-bump obstacle on the grass patch
    bump_mat = chrono.ChContactMaterialNSC()
    bump_mat.SetFriction(0.9)
    bump_mat.SetRestitution(0.01)
    bump = chrono.ChBodyEasyBox(
        2 * BUMP_HALF_X, 2 * BUMP_HALF_Y, 2 * BUMP_HALF_Z,
        BUMP_DENSITY, True, True, bump_mat,
    )
    bump.SetName("speed_bump")
    bump.SetPos(chrono.ChVector3d(BUMP_CX, 0.0, TERRAIN_TOP_Z + BUMP_HALF_Z))
    bump.SetFixed(True)               # anchored obstacle, not a free body
    bump.EnableCollision(True)
    system.AddBody(bump)

    # === Footprint check === verify the wheels rest on (not through) the support
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

    # === Driver === scripted open-loop throttle ramp, straight-line run
    driver = GatorDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator on multi-patch terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 8.0, 0.8)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8, -8, 4),
                      chrono.ChVector3d(VEH_INIT_X, 0, 0))
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 30, 20,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output dirs / CSV writers ===
    os.makedirs("frames", exist_ok=True)    # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)       # review video + motion log live here

    data_f = None
    motion_f = None
    try:
        try:
            data_f = open("simulation_data.csv", "w", newline="")
            motion_f = open("cam/motion_log.csv", "w", newline="")
        except (OSError, IOError) as exc:    # disk full / permission denied
            print(f"cannot open CSV output: {exc}")
            raise

        data_w = csv.writer(data_f)
        data_w.writerow(["time", "pos_x", "pos_y", "pos_z", "speed", "throttle"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        times, xs, zs, speeds = [], [], [], []

        # === Main loop === render-cadence outer loop; vehicle Synchronize/Advance inner
        step = 0
        frame = 0
        running = True
        while running:
            sim_time = system.GetChTime()
            if sim_time >= RUN_END:
                break
            if not HEADLESS and not vis.Run():
                break

            if not HEADLESS and step % RENDER_EVERY == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            driver_inputs = driver.GetInputs()

            # log physics every step
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            data_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                             f"{pos.z:.5f}", f"{speed:.5f}",
                             f"{driver_inputs.m_throttle:.4f}"])
            motion_w.writerow([f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}",
                               f"{pos.y:.5f}", f"{pos.z:.5f}", f"{vel.x:.5f}",
                               f"{vel.y:.5f}", f"{vel.z:.5f}"])
            times.append(sim_time); xs.append(pos.x); zs.append(pos.z); speeds.append(speed)

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            gator.Synchronize(sim_time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            gator.Advance(TIME_STEP)          # advances the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1

    except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close partial CSV even if a step diverges mid-run
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing === plot the logged time series
    if times:
        t = np.array(times)
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(t, np.array(xs), color="tab:blue")
        ax[0].set_ylabel("pos_x (m)"); ax[0].grid(True)
        ax[0].set_title("Gator across multi-patch terrain")
        ax[1].plot(t, np.array(zs), color="tab:green")
        ax[1].set_ylabel("pos_z (m)"); ax[1].grid(True)
        ax[2].plot(t, np.array(speeds), color="tab:red")
        ax[2].set_ylabel("speed (m/s)"); ax[2].set_xlabel("time (s)"); ax[2].grid(True)
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        print(f"steps={len(times)} final_x={xs[-1]:.3f} "
              f"final_z={zs[-1]:.3f} max_speed={max(speeds):.3f}")


if __name__ == "__main__":
    main()
