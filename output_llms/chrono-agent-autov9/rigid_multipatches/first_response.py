"""HMMWV on a multi-patch RigidTerrain (PyChrono 9.0.1, NSC system, Irrlicht).

Model
-----
An HMMWV_Full wheeled vehicle (chassis + 4 suspended axles, TMEASY tires,
SIMPLE engine + AUTOMATIC_SIMPLE_MAP transmission, AWD driveline, Pitman-arm
steering) driving across a complex rigid terrain made of FOUR distinct patches
registered on a single veh.RigidTerrain:

  1. a flat box patch with a "tile4" texture,
  2. a second flat box patch (slightly raised) with a "concrete" texture,
  3. a triangle-mesh patch built from bump.obj (a dirt bump),
  4. a heightmap patch built from bump64.bmp (grass, varying elevation).

System type
-----------
ChSystemNSC (non-smooth contact) — owned internally by the HMMWV_Full wrapper;
all contact materials are ChContactMaterialNSC to match.

Driver / control
----------------
A scripted veh.ChDriver subclass supplies steering, throttle and braking as a
function of time (gentle launch, mild slalom steering, light brake near the
end), exercising the interactive control channels in a headless-friendly way.

Expected behavior
------------------
The four wheels rest on the terrain at spawn; under throttle the chassis
translates forward (+X) along the road, the body stays upright, and the
steering input produces lateral heading change. Per-step physics are logged to
simulation_data.csv and cam/motion_log.csv; a timeseries plot is written at the
end.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend (no display needed for the plot)
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics) ===
STEP_SIZE = 2e-3                 # integration step (s)
TIRE_STEP_SIZE = 1e-3            # tire sub-step (s)
SIM_END = 8.0                    # simulated duration (s): sized so the vehicle
                                 # stays on the [-32, +32] m road span and never
                                 # runs off the patch edges (see throttle law)
RENDER_FPS = 50.0                # review-video frame rate

# Vehicle spawn: start on patch 1 (the tiled flat patch centered at x=-16),
# facing +X so it drives across the road. Z lifted to rest wheels on z=0.
VEH_INIT_X = -22.0
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.6      # HMMWV chassis-origin height above wheel-bottom
VEH_INIT_Z = 0.0 + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.46               # approx HMMWV tire radius (m), validated below
ZTOL = 0.10                      # allowed wheel-bottom clearance vs support top

# Terrain patch geometry (centers + sizes), kept as constants so no bare
# literals appear downstream. Patches do not overlap in X/Y.
PATCH1_CENTER = chrono.ChVector3d(-16, 0, 0)      # tiled flat patch
PATCH1_LEN, PATCH1_WID = 32.0, 20.0
PATCH2_CENTER = chrono.ChVector3d(16, 0, 0.15)    # concrete flat patch (raised)
PATCH2_LEN, PATCH2_WID = 32.0, 30.0
PATCH3_CENTER = chrono.ChVector3d(0, -42, 0)      # dirt bump (mesh patch)
PATCH4_CENTER = chrono.ChVector3d(0, 42, 0)       # grass heightmap patch
PATCH4_LEN, PATCH4_WID = 64.0, 64.0
PATCH4_HMIN, PATCH4_HMAX = 0.0, 3.0

# Asset files (resolved against the vehicle data path; never hardcoded abs paths)
TEX_TILE = veh.GetVehicleDataFile("terrain/textures/tile4.jpg")
TEX_CONCRETE = veh.GetVehicleDataFile("terrain/textures/concrete.jpg")
TEX_DIRT = veh.GetVehicleDataFile("terrain/textures/dirt.jpg")
TEX_GRASS = veh.GetVehicleDataFile("terrain/textures/grass.jpg")
MESH_BUMP = veh.GetVehicleDataFile("terrain/meshes/bump.obj")
HEIGHTMAP_BUMP = veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp")

# Headless validation gate: skip the window + run a short bounded sim for speed.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

# Derived constants (precomputed once — never recomputed in the hot loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END         # short check when validating


# === Driver: scripted time-based control (steering / throttle / braking) ===
class ScriptedDriver(veh.ChDriver):
    """Interactive control channels driven by a scripted time law.

    Gentle throttle ramp, a mild sinusoidal slalom on the steering channel, and
    a light brake near the end. Drives the inputs through the ChDriver setters
    so GetInputs() returns the scripted values each step.
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle: ramp 0 -> 0.4 over the first 2 s, hold a moderate cruise,
        # then release and brake so the vehicle halts on the road (not past the
        # +32 m edge). The cruise speed * cruise time keeps travel within span.
        if time < 2.0:
            throttle = 0.20 * time          # 0 -> 0.4
            braking = 0.0
        elif time < SIM_END - 3.0:
            throttle = 0.4
            braking = 0.0
        else:
            throttle = 0.0
            braking = 0.6                   # firmer brake to stop before the edge
        # Steering: mild slalom, zero before motion starts.
        steering = 0.0 if time < 1.0 else 0.25 * math.sin(0.6 * time)

        self.SetThrottle(max(0.0, min(1.0, throttle)))
        self.SetBraking(max(0.0, min(1.0, braking)))
        self.SetSteering(max(-1.0, min(1.0, steering)))


def build_terrain(system):
    """Build a RigidTerrain with four distinct patches (flat tiled, flat
    concrete, mesh bump, heightmap) — each with its own contact material and
    texture. Returns the initialized terrain."""
    terrain = veh.RigidTerrain(system)

    # Patch 1 — flat box, tiled texture.
    mat1 = chrono.ChContactMaterialNSC()
    mat1.SetFriction(0.9)
    mat1.SetRestitution(0.01)
    patch1 = terrain.AddPatch(mat1, chrono.ChCoordsysd(PATCH1_CENTER, chrono.QUNIT),
                              PATCH1_LEN, PATCH1_WID)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(TEX_TILE, 20, 20)

    # Patch 2 — flat box (slightly raised), concrete texture.
    mat2 = chrono.ChContactMaterialNSC()
    mat2.SetFriction(0.9)
    mat2.SetRestitution(0.01)
    patch2 = terrain.AddPatch(mat2, chrono.ChCoordsysd(PATCH2_CENTER, chrono.QUNIT),
                              PATCH2_LEN, PATCH2_WID)
    patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    patch2.SetTexture(TEX_CONCRETE, 20, 20)

    # Patch 3 — triangle-mesh patch (a dirt bump) loaded from bump.obj.
    mat3 = chrono.ChContactMaterialNSC()
    mat3.SetFriction(0.9)
    mat3.SetRestitution(0.01)
    patch3 = terrain.AddPatch(mat3, chrono.ChCoordsysd(PATCH3_CENTER, chrono.QUNIT),
                              MESH_BUMP)
    patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    patch3.SetTexture(TEX_DIRT, 6.0, 6.0)

    # Patch 4 — heightmap patch (varying grass elevation) from bump64.bmp.
    mat4 = chrono.ChContactMaterialNSC()
    mat4.SetFriction(0.9)
    mat4.SetRestitution(0.01)
    patch4 = terrain.AddPatch(mat4, chrono.ChCoordsysd(PATCH4_CENTER, chrono.QUNIT),
                              HEIGHTMAP_BUMP, PATCH4_LEN, PATCH4_WID,
                              PATCH4_HMIN, PATCH4_HMAX)
    patch4.SetTexture(TEX_GRASS, 6.0, 6.0)

    terrain.Initialize()  # call AFTER all patches are added
    return terrain


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper internally creates a ChSystemNSC plus the chassis rigid body,
    # four axles/spindles, suspension + steering joints, and the powertrain.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
                           chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # grip model suited to rigid road
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    # Mesh visualization on all vehicle components, per the objective.
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()                # cache: vehicle handle, reused every step

    # === Terrain (multi-patch RigidTerrain) ===
    terrain = build_terrain(system)

    # Footprint check: read actual spindle world positions after Initialize and
    # assert the wheels rest on the road (z ~ 0), not buried or floating.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= 0.0 - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z=0.0; raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m")
    assert wheel_bottom_z <= 0.0 + 4.0 * ZTOL, (
        f"vehicle floats above terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        f"lower SUSPENSION_REF_HEIGHT")

    # === Driver (scripted control of steering / throttle / braking) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + chase cam
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on Multi-Patch Rigid Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 8.0, 0.6)  # chase camera view
        vis.Initialize()                                                  # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                                   # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8, -8, 4),
                      chrono.ChVector3d(VEH_INIT_X, 0, 0))                # explicit camera
        vis.AddTypicalLights()                                            # standard lighting
        vis.AddGrid(2.0, 2.0, 30, 30,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                        # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop (render-cadence outer loop; Synchronize/Advance inner batch) ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    sim_csv = None
    motion_csv = None
    times, speeds, xs, ys, zs = [], [], [], [], []
    frame = 0
    try:
        sim_csv = open("simulation_data.csv", "w", newline="")      # main physics log
        motion_csv = open("cam/motion_log.csv", "w", newline="")    # per-body motion contract
        sim_writer = csv.writer(sim_csv)
        motion_writer = csv.writer(motion_csv)
        sim_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z",
                             "speed", "throttle", "steering", "braking"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        def log_step(t):
            # cache: fetch pose/velocity once per step from the cached handles
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            spd = veh_obj.GetSpeed()
            di = driver.GetInputs()
            sim_writer.writerow([f"{t:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{spd:.5f}",
                                 f"{di.m_throttle:.4f}", f"{di.m_steering:.4f}",
                                 f"{di.m_braking:.4f}"])
            motion_writer.writerow([f"{t:.5f}", "chassis", f"{pos.x:.5f}",
                                    f"{pos.y:.5f}", f"{pos.z:.5f}",
                                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
            times.append(t); speeds.append(spd)
            xs.append(pos.x); ys.append(pos.y); zs.append(pos.z)

        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1
            for _ in range(RENDER_EVERY):
                t = system.GetChTime()
                driver_inputs = driver.GetInputs()
                log_step(t)
                # Synchronize the full subsystem stack, then advance each one.
                driver.Synchronize(t)
                terrain.Synchronize(t)
                hmmwv.Synchronize(t, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(t, driver_inputs)
                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                hmmwv.Advance(STEP_SIZE)        # advances the wrapper-owned ChSystem
                if not HEADLESS:
                    vis.Advance(STEP_SIZE)
                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:            # disk / permission while writing CSV
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if sim_csv is not None:
            sim_csv.close()
        if motion_csv is not None:
            motion_csv.close()

    # === Post-processing (timeseries plot from logged data) ===
    if times:
        t_arr = np.array(times)
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(t_arr, speeds, label="speed (m/s)", color="tab:blue")
        ax[0].set_ylabel("speed (m/s)")
        ax[0].grid(True); ax[0].legend(loc="best")
        ax[1].plot(t_arr, xs, label="chassis x", color="tab:red")
        ax[1].plot(t_arr, ys, label="chassis y", color="tab:green")
        ax[1].plot(t_arr, zs, label="chassis z", color="tab:purple")
        ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("position (m)")
        ax[1].grid(True); ax[1].legend(loc="best")
        fig.suptitle("HMMWV on multi-patch rigid terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        # Quick console summary of the motion contract.
        dx = xs[-1] - xs[0]
        print(f"frames={frame} steps={len(times)} "
              f"x0={xs[0]:.2f} x_end={xs[-1]:.2f} dx={dx:.2f} "
              f"max_speed={max(speeds):.2f} final_z={zs[-1]:.3f}")


if __name__ == "__main__":
    main()
