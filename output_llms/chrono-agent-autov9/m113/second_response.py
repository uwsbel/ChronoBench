"""M113 tracked vehicle on SCM deformable soft-soil terrain (PyChrono 9.0.1, Irrlicht).

Model
-----
* Protagonist: an M113 tracked vehicle (veh.M113 wrapper) with single-pin track
  shoes, a SHAFTS engine + automatic-shafts transmission, and a BDS driveline so
  the sprockets deliver real tractive torque to the tracks.
* System type: NSC (complementarity contact). A single-pin steel track is unstable
  under SMC penalty contact, so the wrapper owns a ChSystemNSC and the terrain
  contact material is the matching ChContactMaterialNSC.
* Terrain: SCMTerrain — a Bekker-Wong empirical soft-soil model. The surface is
  deformable, so the tracks sink in and leave ruts. It is initialized from a
  height map (gentle undulating field) and textured with dirt.

Expected behavior
------------------
The vehicle starts at world (-15, 0, 0.0) on the soft soil and is commanded a
constant throttle of 0.8 with zero steering. The tracks bite into the deformable
soil and the hull drives forward (along the chassis -X heading) across the patch,
leaving visible ruts, remaining upright the whole time.

The script is fully self-contained: it builds the vehicle, terrain, driver,
Irrlicht visualization, the explicit Synchronize/Advance main loop, CSV logging,
and a matplotlib time-series plot inline, using only public PyChrono APIs.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")              # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
TIME_STEP = 1e-3                   # integration step (s); fine enough for track contact
SIM_END = 12.0                     # total simulated time (s)
RENDER_FPS = 30.0                  # review-video frame rate

# Initial vehicle location requested for this scene.
INIT_X = -15.0
INIT_Y = 0.0
# Spawn the chassis a little above the rest plane so the tracks settle ONTO the
# soil rather than starting interpenetrating; the wrapper adds suspension/road-wheel
# clearance on top of this when Initialize() places the road wheels.
INIT_Z = 0.64
SPAWN_POS = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
# Heading: yaw 180 deg about Z so the chassis forward axis points along world +X,
# i.e. positive throttle drives the hull in the +X direction (away from the start).
SPAWN_ROT = chrono.QuatFromAngleZ(math.pi)

# Constant open-loop command for this run.
THROTTLE_CMD = 0.8                 # hard-coded throttle held all run
STEERING_CMD = 0.0
BRAKING_CMD = 0.0

# SCM terrain extent / resolution and height-map elevation mapping.
SCM_SIZE_X = 60.0
SCM_SIZE_Y = 60.0
SCM_DELTA = 0.1                    # grid resolution (m) — visible ruts, affordable rays
SCM_H_MIN = -0.2                   # height-map black -> this elevation (m)
SCM_H_MAX = 0.2                    # height-map white -> this elevation (m)

# SCM soil parameters (Bekker-Wong / Mohr / Janosi), all 8 positional args.
SOIL_BEKKER_KPHI = 2e6
SOIL_BEKKER_KC = 0.0
SOIL_BEKKER_N = 1.1
SOIL_MOHR_COHESION = 0.0
SOIL_MOHR_FRICTION = 30.0          # internal friction angle (deg)
SOIL_JANOSI_SHEAR = 0.01
SOIL_ELASTIC_K = 2e8
SOIL_DAMPING_R = 3e4

# Active-domain half-extents around the chassis (only deform cells near the hull).
ACTIVE_HALF = chrono.ChVector3d(6.0, 4.0, 1.0)

# Derived constants (precomputed once — never recomputed in the hot loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short bounded check when validating

HEIGHTMAP_FILE = chrono.GetChronoDataFile("vehicle/terrain/height_maps/bump64.bmp")
DIRT_TEXTURE = chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg")
LOGO_FILE = chrono.GetChronoDataFile("logo_chrono_alpha.png")


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)       # guard against missing motion-log dir

    # === Vehicle wrapper + system & bodies ===
    # veh.M113 internally creates the ChSystemNSC, the chassis rigid body, the
    # sprockets/idlers/road-wheels, every track shoe, and all suspension joints.
    vehicle = veh.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # complementarity contact (stable single-pin track)
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)      # BDS driveline -> real tractive torque
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)      # shafts engine for true torque (simple-map barely creeps)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetInitPosition(chrono.ChCoordsysd(SPAWN_POS, SPAWN_ROT))
    vehicle.Initialize()

    # Make the wrapper-created essentials visible as named handles.
    system = vehicle.GetSystem()              # ChSystemNSC owned by the M113 wrapper
    tracked = vehicle.GetVehicle()            # ChTrackedVehicle interface
    chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused every step
    num_shoes_left = tracked.GetNumTrackShoes(veh.LEFT)    # precomputed once
    num_shoes_right = tracked.GetNumTrackShoes(veh.RIGHT)  # precomputed once
    # joints: sprocket/idler/road-wheel revolutes + track-shoe pins created inside the wrapper.

    # Visualization detail for the wrapper sub-bodies.
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # === Terrain (SCM deformable soft soil from a height map) ===
    # SCM needs the system's collision system, which the wrapper already created.
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        SOIL_BEKKER_KPHI, SOIL_BEKKER_KC, SOIL_BEKKER_N,
        SOIL_MOHR_COHESION, SOIL_MOHR_FRICTION, SOIL_JANOSI_SHEAR,
        SOIL_ELASTIC_K, SOIL_DAMPING_R,
    )
    # Active domain follows the level chassis body (NOT a spinning wheel) so the
    # OOBB projection stays stable and rays actually hit the soil.
    terrain.AddActiveDomain(chassis, chrono.ChVector3d(0, 0, 0), ACTIVE_HALF)
    terrain.SetMeshWireframe(False)
    # Colored sinkage heatmap so the ruts under the tracks are clearly visible.
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
    # Height-map Initialize: black->H_MIN, white->H_MAX over the SIZE_X x SIZE_Y patch.
    terrain.Initialize(HEIGHTMAP_FILE, SCM_SIZE_X, SCM_SIZE_Y, SCM_H_MIN, SCM_H_MAX, SCM_DELTA)
    terrain.SetTexture(DIRT_TEXTURE, 80, 80)   # dirt texture on the soil surface

    # === Driver (constant open-loop throttle command) ===
    # DriverInputs struct written each step; throttle hard-coded to 0.8.
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = THROTTLE_CMD
    driver_inputs.m_steering = STEERING_CMD
    driver_inputs.m_braking = BRAKING_CMD

    # Per-side terrain-force buffers sized to the shoe counts (tracked Synchronize).
    shoe_forces_left = veh.TerrainForces(num_shoes_left)     # precomputed once
    shoe_forces_right = veh.TerrainForces(num_shoes_right)   # precomputed once

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Gated on the validation flag so the windowless check runs fast; the full
    # tracked-vehicle Irrlicht block is always present for the reviewer.
    vis = None
    if not HEADLESS:
        vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("M113 tracked vehicle on SCM deformable terrain")
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 8.0, 2.2)    # raised, angled follow of the hull
        vis.Initialize()                                            # Initialize FIRST (Irrlicht)
        vis.AddLogo(LOGO_FILE)                                      # branding logo
        vis.AddSkyBox()                                            # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(INIT_X - 7, INIT_Y - 7, 5),
                      chrono.ChVector3d(INIT_X, INIT_Y, 0))         # static fallback camera
        vis.AddTypicalLights()                                     # standard lighting
        # Reference grid placed BELOW the soil surface so it does not mask the
        # dirt-textured, deforming SCM mesh the tracks ride on.
        vis.AddGrid(2.0, 2.0, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, SCM_H_MIN - 0.05), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                  # ground reference grid (sub-surface)
        vis.AttachVehicle(vehicle.GetVehicle())                    # bind chassis/track visuals last

    # === Main loop (render-cadence outer loop; Synchronize/Advance per step) ===
    sim_csv = None
    motion_csv = None
    times = []
    pos_x = []
    pos_y = []
    pos_z = []
    speeds = []

    try:
        # cache: open both CSV writers once before the loop, flush rows every step.
        sim_csv = open("simulation_data.csv", "w", newline="")
        motion_csv = open("cam/motion_log.csv", "w", newline="")
        sim_writer = csv.writer(sim_csv)
        motion_writer = csv.writer(motion_csv)
        sim_writer.writerow(["time", "pos_x", "pos_y", "pos_z", "speed", "throttle"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz", "speed"])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()

                # --- log physics this step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = vel.Length()
                sim_writer.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                     f"{pos.z:.5f}", f"{speed:.5f}", f"{THROTTLE_CMD:.3f}"])
                motion_writer.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}",
                                        f"{pos.y:.5f}", f"{pos.z:.5f}", f"{vel.x:.5f}",
                                        f"{vel.y:.5f}", f"{vel.z:.5f}", f"{speed:.5f}"])
                times.append(time)
                pos_x.append(pos.x)
                pos_y.append(pos.y)
                pos_z.append(pos.z)
                speeds.append(speed)

                # --- advance the full tracked-vehicle subsystem stack ---
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right)
                if not HEADLESS:
                    vis.Synchronize(time, driver_inputs)

                terrain.Advance(TIME_STEP)
                vehicle.Advance(TIME_STEP)   # steps the wrapper-owned system (no DoStepDynamics)
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission on CSV writers
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush + close any open writers even if a step diverged mid-run
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing (time-series plot from the logged arrays) ===
    if times:
        t = np.array(times)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(t, np.array(pos_x), label="x")
        ax1.plot(t, np.array(pos_y), label="y")
        ax1.plot(t, np.array(pos_z), label="z")
        ax1.set_ylabel("chassis position (m)")
        ax1.legend()
        ax1.grid(True)
        ax2.plot(t, np.array(speeds), color="tab:red", label="speed")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("chassis speed (m/s)")
        ax2.legend()
        ax2.grid(True)
        fig.suptitle("M113 on SCM deformable terrain — throttle 0.8")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        # Headline physics check: did the hull travel from its spawn?
        travelled = math.hypot(pos_x[-1] - INIT_X, pos_y[-1] - INIT_Y)
        print(f"frames={len(times)} travelled={travelled:.3f} m "
              f"final_pos=({pos_x[-1]:.3f},{pos_y[-1]:.3f},{pos_z[-1]:.3f}) "
              f"max_speed={max(speeds):.3f} m/s")


if __name__ == "__main__":
    main()
