"""ARTcar (small RC-scale wheeled vehicle) driving on flat rigid terrain.

Model
-----
- Vehicle: veh.ARTcar wrapper (chassis + double-wishbone suspensions + Pitman-arm
  steering + 4WD driveline + TMEASY tires). The wrapper creates and owns a
  ChSystemNSC (NSC contact) internally.
- Terrain: a single flat veh.RigidTerrain patch (Bullet rigid contacts) with a
  custom texture, sized wide enough for the car to drive in a straight line.
- Driver: a scripted veh.ChDriver subclass that ramps throttle up after a short
  settle phase and applies a gentle sinusoidal steering — no human-in-the-loop.

System type: NSC (non-smooth contact), Z-up world, gravity -9.81 m/s^2 on Z.

Expected behavior
------------------
The car settles onto the terrain, then accelerates forward (chassis +X position
grows monotonically once throttle engages) while remaining upright (chassis Z
stays near its rest height and the up-axis stays close to world +Z). The
interactive view is a chase camera following the chassis, rendered through the
vehicle-aware Irrlicht visual system at 50 frames per second.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
TIME_STEP = 1.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # TMEASY tire substep (s)
SIM_END = 12.0                     # total simulated time (s)
RENDER_FPS = 50.0                  # review video / display frame rate

TERRAIN_LENGTH = 200.0             # X extent (m) — wide enough for a long straight run
TERRAIN_WIDTH = 60.0               # Y extent (m)
TERRAIN_FRICTION = 0.9             # rubber-on-road friction
TERRAIN_RESTITUTION = 0.01         # nearly inelastic ground

INIT_X = -80.0                     # spawn near the -X end so the car drives across the patch
INIT_Y = 0.0
INIT_Z = 0.20                      # chassis-origin height so wheels rest on the patch (z=0)

TIRE_RADIUS = 0.085                # ARTcar TMEASY tire radius (m), from wheel geometry
ZTOL = 0.05                        # allowed wheel-bottom clearance/overlap vs terrain top
TERRAIN_TOP_Z = 0.0                # flat patch top plane

SETTLE_TIME = 1.0                  # s of braking before driving, lets suspension settle
THROTTLE = 0.7                     # cruise throttle once moving
STEER_AMP = 0.10                   # gentle steering amplitude (-1..+1)
STEER_FREQ = 0.25                  # steering oscillation frequency (Hz)

# Derived once before the loop (never recomputed inside it).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))            # precomputed once
INIT_POS = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT
STEER_OMEGA = 2.0 * math.pi * STEER_FREQ                               # precomputed once


# === Scripted driver === time-based control law (no keyboard / human-in-the-loop)
class ScriptedDriver(veh.ChDriver):
    """Brake during the settle phase, then cruise with gentle sinusoidal steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            self.SetThrottle(THROTTLE)
            self.SetBraking(0.0)
            self.SetSteering(STEER_AMP * math.sin(STEER_OMEGA * time))


# === Vehicle === ARTcar wrapper owns the ChSystemNSC; build + initialize first
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)        # slip/grip tire model for road driving
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.ARTcar wrapper) ===
system = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = vehicle.GetVehicle()               # cache: ChWheeledVehicle handle, reused every step
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links
# (double wishbones + Pitman arm) are created inside the wrapper.

# Bullet is the collision system used for vehicle + rigid-terrain contact. The
# wrapper builds its system internally, so set the type explicitly afterwards.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch with a custom texture, the support the car drives on
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Footprint check: every spindle's wheel-bottom must rest on (not through) the patch.
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_Z by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === scripted autonomous control, attached to the vehicle
driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.3), 2.5, 0.4)   # follow the small RC chassis
vis.Initialize()                                                  # Initialize FIRST (Irrlicht)
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()                                                   # outdoor sky backdrop
vis.AddTypicalLights()                                            # standard lighting
vis.AttachVehicle(veh_obj)                                        # bind chassis/wheel/tire assets
vis.AttachDriver(driver)                                          # steering/throttle/brake HUD

# === Main loop === advance the full subsystem stack; render at RENDER_FPS cadence


frame = 0
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
            vehicle.Synchronize(sim_time, driver_inputs, terrain)   # terrain arg: TMEASY ground contact
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)        # steps the wrapper-owned system (no DoStepDynamics)
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:     # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot, drop raw frames
