"""Gator wheeled-vehicle drive on flat rigid terrain (PyChrono 9.0.1, Irrlicht).

Models the catalog Gator UTV (`veh.Gator` wrapper, SMC contact system) driving
on a flat `veh.RigidTerrain` patch under a scripted, rate-limited driver. The
moving bodies are the wrapper-created chassis, four spindles/wheels, and the
suspension + steering links; the support is the rigid terrain patch.

Three things distinguish this scene:
  * every vehicle subsystem (chassis, suspension, steering, wheels, tires) is
    drawn with PRIMITIVE shapes rather than detailed meshes;
  * the chassis carries a simple PRIMITIVE box collision shape (the wrapper
    chassis collision stays CollisionType_NONE and a ChCollisionShapeBox is
    welded to the chassis body instead of a collision mesh);
  * the driver is deliberately *unresponsive* — the commanded throttle/steering
    ramp toward their targets at a small per-step delta, so the applied controls
    lag the request and take time to build up.

Expected behavior: the Gator starts at rest, the throttle slowly ramps in, and
the vehicle accelerates forward in a gentle steer; because the driver is
rate-limited the speed builds up gradually rather than snapping to full input.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 2e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # tire force model sub-step (s)
SIM_END = 12.0                   # simulated duration (s)
RENDER_FPS = 50.0                # review-video frame rate
INIT_X, INIT_Y = 0.0, 0.0        # spawn XY on the terrain centre
SUSPENSION_REF_HEIGHT = 0.5      # Gator chassis-origin height above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0              # flat rigid patch top plane
TERRAIN_LEN, TERRAIN_WID = 100.0, 100.0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT          # derived chassis-origin height

# Chassis collision box (full extents, m) — a coarse primitive proxy for the body.
CH_BOX_L, CH_BOX_W, CH_BOX_H = 3.0, 1.5, 1.0
CH_BOX_Z = 0.4                   # box centre above chassis origin

# Rate-limited ("sluggish") driver: how fast a command may change, per second.
# Small deltas => controls take noticeable time to reach the requested value.
THROTTLE_RATE = 0.25             # max throttle change per second (slow ramp-in)
STEER_RATE = 0.20                # max steering change per second (slow turn-in)
TARGET_THROTTLE = 0.7            # requested throttle once the run gets going
TARGET_STEER = 0.3               # requested (gentle) right steer
THROTTLE_HOLD_START = 1.0        # seconds before the throttle request is issued

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver === scripted, rate-limited ChDriver subclass (the "less responsive" input)
class RateLimitedDriver(veh.ChDriver):
    """Scripted driver whose applied inputs chase the requested targets slowly.

    Each Synchronize() nudges the current throttle/steering toward the requested
    target by at most rate*dt, so a step request in the command produces a slow
    ramp in the applied control instead of an instantaneous jump — the "controls
    take more time to apply" behavior asked for.
    """

    def __init__(self, vehicle, dt):
        super().__init__(vehicle)
        self._dt = dt                       # cache: per-call ramp interval
        self._throttle = 0.0
        self._steering = 0.0

    @staticmethod
    def _approach(current, target, max_step):
        # Move `current` toward `target` by at most `max_step`.
        if target > current:
            return min(current + max_step, target)
        return max(current - max_step, target)

    def Synchronize(self, time):
        req_throttle = TARGET_THROTTLE if time >= THROTTLE_HOLD_START else 0.0
        req_steer = TARGET_STEER if time >= THROTTLE_HOLD_START else 0.0
        self._throttle = self._approach(self._throttle, req_throttle,
                                        THROTTLE_RATE * self._dt)
        self._steering = self._approach(self._steering, req_steer,
                                        STEER_RATE * self._dt)
        self.SetThrottle(self._throttle)
        self.SetSteering(self._steering)
        self.SetBraking(0.0)


# === Vehicle === catalog Gator wrapper (owns its ChSystem + all rigid bodies)
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)   # add a primitive box to the body instead
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
                                         chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)             # rolling/grip tire on rigid road
gator.SetTireStepSize(TIRE_STEP)
gator.Initialize()

# Visualization type = PRIMITIVES on every subsystem (simplify from mesh).
gator.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()                  # ChSystemSMC owned by the wrapper
chassis_body = gator.GetChassisBody()       # cache: main chassis rigid body, reused below
veh_obj = gator.GetVehicle()                # cache: vehicle subsystem handle
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links are created inside the wrapper; terrain patch body is added below.

# Collision system MUST be Bullet for the terrain + chassis-box contact.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Chassis collision === simple PRIMITIVE box welded to the chassis body
chassis_mat = chrono.ChContactMaterialSMC()
chassis_mat.SetFriction(0.7)
chassis_mat.SetRestitution(0.0)
chassis_mat.SetYoungModulus(2e7)
chassis_body.AddCollisionShape(
    chrono.ChCollisionShapeBox(chassis_mat, CH_BOX_L, CH_BOX_W, CH_BOX_H),
    chrono.ChFramed(chrono.ChVector3d(0, 0, CH_BOX_Z), chrono.QUNIT),
)
chassis_body.EnableCollision(True)
system.GetCollisionSystem().BindAll()       # rebuild collision models after the post-init edit

# === Terrain === flat rigid patch supporting the vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LEN, TERRAIN_WID)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Footprint check: wheels must rest on (not through) the rigid patch.
TIRE_RADIUS = 0.285              # Gator tire radius (m), from the wheel geometry
ZTOL = 0.10
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === instantiate the rate-limited scripted driver
driver = RateLimitedDriver(veh_obj, TIME_STEP)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — primitive visualization, sluggish driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.0), 8.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            gator.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            gator.Advance(TIME_STEP)          # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:     # solver divergence / bad state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
