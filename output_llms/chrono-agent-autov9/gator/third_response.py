"""Gator UTV driving on flat rigid terrain (PyChrono 9.0.x, Irrlicht).

Model
-----
A wrapper-managed ``veh.Gator`` wheeled utility vehicle (NSC contact system,
TMEASY tires) driving on a flat ``veh.RigidTerrain`` patch and steered by a
scripted driver.

This script demonstrates three things explicitly:

1. Primitive (not mesh) visualization for every vehicle subsystem — the chassis,
   suspension, steering, wheels, and tires are all drawn with
   ``chrono.VisualizationType_PRIMITIVES``.
2. A chassis collision volume built from SIMPLE PRIMITIVE shapes (a single
   collision box welded to the chassis body) rather than a collision mesh — the
   wrapper chassis collision type is kept NONE and a primitive box is added inline.
3. A LESS-RESPONSIVE driver: a custom ``veh.ChDriver`` subclass holds a
   commanded ("key pressed") target for steering/throttle/braking and ramps the
   applied control toward that target at a deliberately slow rate, so a control
   command takes noticeably longer to fully apply (a sluggish, low-bandwidth
   driver). The slow-response rates are named constants.

Expected behavior: the Gator launches from rest, the throttle ramps in slowly,
the vehicle accelerates forward and then steers through a gentle turn while the
applied controls visibly lag the commanded targets.
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

# === Named constants: physics / geometry / control ===
TIME_STEP = 2.0e-3                      # integration step (s)
TIRE_STEP = 1.0e-3                      # tire substep (s)
SIM_END = 12.0                          # total simulated time (s)
RENDER_FPS = 30.0                       # review-video frame rate

TERRAIN_LENGTH = 200.0                  # rigid patch X size (m) — enlarged for turning
TERRAIN_WIDTH = 200.0                   # rigid patch Y size (m)
TERRAIN_FRICTION = 0.9                  # tire/ground friction
TERRAIN_RESTITUTION = 0.01             # ground bounciness

TIRE_RADIUS = 0.28575                   # Gator tire radius (m), from tire geometry
SUSPENSION_REF_HEIGHT = 0.306           # chassis-origin height above wheel-bottom at rest
INIT_X, INIT_Y = 0.0, 0.0               # spawn XY on the (open) terrain
TERRAIN_TOP_Z = 0.0                     # flat patch top plane
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis spawn height
ZTOL = 0.06                             # allowed wheel-bottom clearance vs support

# Chassis primitive collision box (full extents, m) — a SIMPLE primitive proxy.
CHASSIS_BOX = (2.6, 1.3, 0.6)
CHASSIS_BOX_OFFSET_Z = 0.3              # box center above chassis origin
CHASSIS_FRICTION = 0.7                  # chassis-vs-prop contact friction

# Less-responsive driver: seconds to traverse the full 0..1 (or -1..1) range.
# Larger => slower / more sluggish response to a held command.
THROTTLE_RESPONSE_TIME = 4.0            # s to go 0 -> 1 throttle
STEERING_RESPONSE_TIME = 5.0            # s to go 0 -> 1 steering (very sluggish)
BRAKING_RESPONSE_TIME = 2.0            # s to go 0 -> 1 braking

# Commanded ("keys pressed") schedule the sluggish driver chases.
THROTTLE_CMD = 0.7                      # held throttle command after launch
STEER_CMD = 0.5                         # held steering command during the turn
LAUNCH_DELAY = 1.0                      # s before throttle is commanded
STEER_START = 4.0                       # s at which the steer command is applied

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run


# === Driver: sluggish (rate-limited) scripted controller ===
class SluggishDriver(veh.ChDriver):
    """A deliberately LESS-RESPONSIVE driver.

    Holds a commanded target (what the operator is "pressing") and slews the
    applied control toward it at a capped rate, so any command takes longer to
    take full effect than an instantaneous driver would.
    """

    def __init__(self, vehicle, step):
        super().__init__(vehicle)
        # cache: per-step max increments derived once from the response times
        self._d_thr = step / THROTTLE_RESPONSE_TIME    # precomputed once
        self._d_str = step / STEERING_RESPONSE_TIME    # precomputed once
        self._d_brk = step / BRAKING_RESPONSE_TIME     # precomputed once
        self._applied_thr = 0.0
        self._applied_str = 0.0
        self._applied_brk = 0.0

    @staticmethod
    def _slew(current, target, max_delta):
        # move `current` toward `target` by at most `max_delta` (rate limit)
        if target > current:
            return min(current + max_delta, target)
        return max(current - max_delta, target)

    def _commanded(self, time):
        # the operator's intent (held "key" targets) as a function of time
        thr_cmd = THROTTLE_CMD if time >= LAUNCH_DELAY else 0.0
        str_cmd = STEER_CMD if time >= STEER_START else 0.0
        return thr_cmd, str_cmd, 0.0

    def Synchronize(self, time):
        thr_cmd, str_cmd, brk_cmd = self._commanded(time)
        # sluggish first-order rate limit toward the commanded targets
        self._applied_thr = self._slew(self._applied_thr, thr_cmd, self._d_thr)
        self._applied_str = self._slew(self._applied_str, str_cmd, self._d_str)
        self._applied_brk = self._slew(self._applied_brk, brk_cmd, self._d_brk)
        self.SetThrottle(self._applied_thr)
        self.SetSteering(self._applied_str)
        self.SetBraking(self._applied_brk)


def main():
    # === System & bodies (created by the veh.Gator wrapper) ===
    # The wrapper instantiates the ChSystemNSC, the chassis rigid body, the four
    # spindle/wheel bodies, and the suspension + steering joints internally.
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)   # add a primitive box below instead
    gator.SetChassisFixed(False)
    gator.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
    )
    gator.SetTireType(veh.TireModelType_TMEASY)             # slip/grip model for rigid road
    gator.SetTireStepSize(TIRE_STEP)
    gator.Initialize()

    # Primitive (NOT mesh) visualization for every vehicle subsystem.
    gator.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    sys = gator.GetSystem()                 # ChSystemNSC owned by the wrapper
    chassis = gator.GetChassisBody()        # cache: chassis body fetched once, reused every step
    veh_obj = gator.GetVehicle()            # cache: vehicle handle fetched once
    # spindles/wheels: veh_obj.GetAxles()[i].GetWheels()[j].GetSpindle()
    # joints: suspension + Pitman-arm steering links live inside the wrapper

    # === Chassis collision: simple PRIMITIVE box (not a mesh) ===
    # Keep the wrapper chassis collision NONE (set above) and weld one primitive
    # collision box to the chassis body so it collides with the world via simple
    # geometry rather than a triangle mesh.
    cmat = chrono.ChContactMaterialNSC()
    cmat.SetFriction(CHASSIS_FRICTION)
    cmat.SetRestitution(0.0)
    chassis.AddCollisionShape(
        chrono.ChCollisionShapeBox(cmat, CHASSIS_BOX[0], CHASSIS_BOX[1], CHASSIS_BOX[2]),
        chrono.ChFramed(chrono.ChVector3d(0, 0, CHASSIS_BOX_OFFSET_Z), chrono.QUNIT),
    )
    chassis.EnableCollision(True)
    sys.GetCollisionSystem().BindAll()      # rebuild collision models after the post-init edit

    # === Footprint assertion (wheels rest on the support, not through it) ===
    spindle_world = []
    for ax in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(ax, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs support top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain (flat rigid patch, enlarged so the turn stays on it) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver: the sluggish (less-responsive) scripted controller ===
    driver = SluggishDriver(veh_obj, TIME_STEP)
    driver.Initialize()

    # === Precomputed loop constants ===
    render_steps = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short check when validating
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)       # motion log + review video live here

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator on rigid terrain — primitives + sluggish driver")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.4), 7.0, 0.6)
        vis.Initialize()                                            # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                             # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(-8, -8, 4), chrono.ChVector3d(0, 0, 0.5))
        vis.AddTypicalLights()                                     # standard lighting
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                  # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)                                    # HUD input bars

    # === Main loop (render-cadence outer loop; log physics every step) ===
    data_f = None
    motion_f = None
    times, speeds, xs, ys = [], [], [], []
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open("cam/motion_log.csv", "w", newline="")
        data_w = csv.writer(data_f)
        motion_w = csv.writer(motion_f)
        data_w.writerow(["time", "x", "y", "z", "speed",
                         "applied_throttle", "applied_steering", "applied_braking"])
        motion_w.writerow(["time", "x", "y", "z", "vx", "vy", "vz", "yaw"])

        step = 0
        frame = 0
        while (HEADLESS or vis.Run()):
            time = sys.GetChTime()
            if time >= run_end:
                break

            if not HEADLESS and step % render_steps == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            driver_inputs = driver.GetInputs()

            # log physics this step
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            rot = chassis.GetRot()
            yaw = rot.GetCardanAnglesZYX().z
            data_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                             f"{speed:.5f}", f"{driver_inputs.m_throttle:.5f}",
                             f"{driver_inputs.m_steering:.5f}", f"{driver_inputs.m_braking:.5f}"])
            motion_w.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                               f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}", f"{yaw:.5f}"])
            times.append(time); speeds.append(speed); xs.append(pos.x); ys.append(pos.y)

            # synchronize the full subsystem stack, then advance it
            driver.Synchronize(time)
            terrain.Synchronize(time)
            gator.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            gator.Advance(TIME_STEP)        # advances the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges — close writers here
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing: timeseries plot from the logged arrays ===
    if times:
        t = np.array(times)
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        a1.plot(t, speeds, label="speed (m/s)")
        a1.set_ylabel("speed (m/s)"); a1.grid(True); a1.legend()
        a2.plot(xs, ys, label="path (x,y)")
        a2.set_xlabel("x (m)"); a2.set_ylabel("y (m)")
        a2.axis("equal"); a2.grid(True); a2.legend()
        fig.suptitle("Gator — speed and ground-plane path")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
