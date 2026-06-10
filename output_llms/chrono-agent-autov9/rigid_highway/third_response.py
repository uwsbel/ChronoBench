"""HMMWV wheeled vehicle driving on a rigid highway mesh terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
- System: NSC contact, owned by the veh.HMMWV_Full wrapper (chassis, four
  spindles/wheels, suspension + steering links are all created inside it).
- Terrain: a single rigid patch built from the Highway collision/visual meshes
  (synchrono/meshes/Highway_col.obj + Highway_vis.obj). The patch contact
  material uses friction 0.4 and restitution 0.05. The patch frame is rotated
  -90 degrees about the world Z axis and positioned at (6, -70, 0) so the
  vehicle sits at the cross-roads of the highway layout. If the meshes are
  missing the script falls back to a flat rigid patch at the same pose (noted
  at runtime) so it still runs standalone.
- Driver: a scripted ChDriver subclass holding steering centered (0) and a
  gentle constant throttle so the vehicle rolls forward and stays on the road.

Expected behavior
------------------
The HMMWV spawns on the highway surface, accelerates from rest under constant
throttle with zero steering, and translates forward in a straight line while
staying upright (roll/pitch near zero). CSV logs verify forward travel and an
upright chassis; the Irrlicht chase camera produces the review frames.
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

# === Named constants (geometry / physics / control) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 30.0                  # review video frame rate

PATCH_FRICTION = 0.4               # prompt: terrain patch friction
PATCH_RESTITUTION = 0.05           # prompt: terrain patch restitution
PATCH_YAW_DEG = -90.0              # prompt: rotate patch -90 deg about Z
PATCH_POS = chrono.ChVector3d(6.0, -70.0, 0.0)   # prompt: patch at cross-roads

FLAT_PATCH_LEN = 200.0             # fallback flat-patch X extent (m)
FLAT_PATCH_WID = 200.0             # fallback flat-patch Y extent (m)

DRIVE_THROTTLE = 0.4               # constant forward throttle
DRIVE_STEERING = 0.0               # centered steering -> stay on road

SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis origin above wheel-bottom at rest
TIRE_RADIUS = 0.46                 # nominal HMMWV tire radius (m), for footprint check
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs road (m)

HIGHWAY_COL = "synchrono/meshes/Highway_col.obj"
HIGHWAY_VIS = "synchrono/meshes/Highway_vis.obj"

# Derived once (precomputed) -------------------------------------------------
PATCH_ROT = chrono.QuatFromAngleZ(math.radians(PATCH_YAW_DEG))   # precomputed once
PATCH_CSYS = chrono.ChCoordsysd(PATCH_POS, PATCH_ROT)            # precomputed once
# Spawn the vehicle just above the patch position so it starts on the road.
# The Highway mesh is a long strip; the -90 deg patch yaw rotates its 150 m
# length onto the world +X axis, so the vehicle faces +X (yaw 0) to drive down
# the road's length and stay between the lane edges.
VEH_INIT_LOC = chrono.ChVector3d(PATCH_POS.x, PATCH_POS.y,
                                 PATCH_POS.z + SUSPENSION_REF_HEIGHT)
VEH_INIT_ROT = chrono.QUNIT                                       # face +X, along road length
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))       # precomputed once

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating


# === Driver (scripted: centered steering, constant throttle) ===
class HighwayDriver(veh.ChDriver):
    """Open-loop driver: hold steering centered, ramp to a constant throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Smoothly ramp throttle over the first second to avoid a wheel-spin jerk.
        ramp = min(1.0, time / 1.0)
        self.SetThrottle(DRIVE_THROTTLE * ramp)
        self.SetBraking(0.0)
        self.SetSteering(DRIVE_STEERING)


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_LOC, VEH_INIT_ROT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # grippy tire for a paved road
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
    veh_obj = hmmwv.GetVehicle()               # cache: vehicle handle, reused every step
    # spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links

    # === Terrain (rigid Highway-mesh patch; friction/restitution/pose per prompt) ===
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(PATCH_FRICTION)
    patch_mat.SetRestitution(PATCH_RESTITUTION)

    col_path = chrono.GetChronoDataFile(HIGHWAY_COL)
    vis_path = chrono.GetChronoDataFile(HIGHWAY_VIS)
    used_mesh = os.path.exists(col_path)
    if used_mesh:
        patch = terrain.AddPatch(patch_mat, PATCH_CSYS, col_path, True, 0.0, True)
        if os.path.exists(vis_path):
            vshape = chrono.ChVisualShapeModelFile()
            vshape.SetFilename(vis_path)
            patch.GetGroundBody().AddVisualShape(
                vshape, chrono.ChFramed(PATCH_POS, PATCH_ROT))
    else:
        # Fallback: flat rigid patch at the same pose so the script is standalone.
        print("NOTE: Highway mesh not found; using flat fallback patch at same pose.")
        patch = terrain.AddPatch(patch_mat, PATCH_CSYS, FLAT_PATCH_LEN, FLAT_PATCH_WID)
        patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
    terrain.Initialize()

    # === Footprint check (wheels rest on, not through, the road) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    road_top_z = PATCH_POS.z
    assert wheel_bottom_z >= road_top_z - ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z={road_top_z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{road_top_z - wheel_bottom_z:.3f} m"
    )

    # === Driver ===
    driver = HighwayDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on rigid highway")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 60, 60,
                    chrono.ChCoordsysd(PATCH_POS, chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop (Synchronize/Advance; render-cadence; CSV logging) ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)      # guard against missing output dir

    times, xs, ys, speeds, rolls, pitches = [], [], [], [], [], []
    data_f = motion_f = None
    try:
        data_f = open("simulation_data.csv", "w", newline="")          # main physics log
        motion_f = open("cam/motion_log.csv", "w", newline="")         # per-body pose log
    except (OSError, IOError) as exc:   # disk full / permission denied
        print("ERROR opening CSV files:", exc)
        raise

    try:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "x", "y", "z", "speed", "roll", "pitch", "yaw", "throttle"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        step = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics this step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                rot = chassis.GetRot()
                rpy = rot.GetCardanAnglesXYZ()   # roll, pitch, yaw (rad)
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}", f"{rpy.x:.6f}",
                                 f"{rpy.y:.6f}", f"{rpy.z:.6f}",
                                 f"{driver_inputs.m_throttle:.4f}"])
                motion_w.writerow([f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}",
                                   f"{pos.y:.5f}", f"{pos.z:.5f}", f"{vel.x:.5f}",
                                   f"{vel.y:.5f}", f"{vel.z:.5f}"])
                if step % RENDER_EVERY == 0:
                    times.append(sim_time)
                    xs.append(pos.x)
                    ys.append(pos.y)
                    speeds.append(speed)
                    rolls.append(rpy.x)
                    pitches.append(rpy.y)

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
                step += 1
                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing (timeseries plot from the logged arrays) ===
    if times:
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax[0].plot(times, xs, label="x")
        ax[0].plot(times, ys, label="y")
        ax[0].set_ylabel("position (m)")
        ax[0].legend(); ax[0].grid(True)
        ax[1].plot(times, speeds, color="tab:green")
        ax[1].set_ylabel("speed (m/s)"); ax[1].grid(True)
        ax[2].plot(times, np.degrees(rolls), label="roll")
        ax[2].plot(times, np.degrees(pitches), label="pitch")
        ax[2].set_ylabel("attitude (deg)"); ax[2].set_xlabel("time (s)")
        ax[2].legend(); ax[2].grid(True)
        fig.suptitle("HMMWV on rigid highway")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print(f"Done. steps logged, final time={system.GetChTime():.3f}s, "
          f"mesh_terrain={used_mesh}")


if __name__ == "__main__":
    main()
