"""UAZBUS wheeled-vehicle real-time simulation on rigid terrain.

Model: a veh.UAZBUS wrapper (an NSC ChSystem owned by the wrapper) driving on a
flat veh.RigidTerrain patch with specified friction and restitution. A scripted
veh.ChDriver subclass supplies open-loop throttle/steering so the bus accelerates
forward from rest; net-zero steering keeps it on the terrain patch. Visualization
is a real-time Irrlicht chase-camera window (sky + lights + logo + chase camera).

System type: NSC (rigid-contact). Main bodies: UAZBUS chassis + 4 spindles/wheels
(created by the wrapper) and the rigid terrain patch.
Expected behavior: the bus settles on its wheels, then drives forward in a
straight line, translating several metres over the simulated interval.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / driver schedule (no bare literals)
TIME_STEP = 1e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # tire force-model step (s)
SIM_END = 10.0                   # simulated duration (s)
RENDER_FPS = 50.0                # render-cadence frame rate (frames per second)

TERRAIN_LENGTH = 200.0           # rigid patch X extent (m) — large to stay on-terrain
TERRAIN_WIDTH = 200.0            # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9           # specified terrain friction coefficient
TERRAIN_RESTITUTION = 0.01       # specified terrain restitution

TIRE_RADIUS = 0.372              # UAZBUS tire radius (m), from wheel geometry
SUSPENSION_REF_HEIGHT = 0.45     # chassis-origin height above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0              # rigid patch top plane (centered at origin)
ZTOL = 0.10                      # allowed wheel-bottom clearance/overlap vs support

INIT_X = 0.0                     # spawn X on the patch (m)
INIT_Y = 0.0                     # spawn Y on the patch (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis-origin height (m)

THROTTLE_START = 1.0             # begin accelerating after this time (s)
CRUISE_THROTTLE = 0.7            # forward throttle once moving

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver === scripted, time-based open-loop control (no human-in-the-loop)
class ScriptedDriver(veh.ChDriver):
    """Hold the brake briefly so the bus settles, then drive straight ahead."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_START:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(CRUISE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(0.0)        # net-zero steering -> straight line, stays on patch


# === Vehicle (UAZBUS wrapper owns its NSC ChSystem) ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT)
)
vehicle.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the bus drives
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()           # cache: main chassis rigid body, reused
veh_obj = vehicle.GetVehicle()               # cache: ChWheeledVehicle handle, reused
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain: RigidTerrain patch body added below.

# Collision system on the wrapper-owned system: enable Bullet narrow-phase contact
# for the tire/terrain interaction (required whenever the scene has collision).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch with the specified friction / restitution
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint assertion === wheels must rest on the patch, not sink through it
spindle_world = [
    veh_obj.GetSpindlePos(axle, side)
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === scripted open-loop driver bound to the vehicle
driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === render-cadence outer loop; vehicle subsystem advance inner batch
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
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)        # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
