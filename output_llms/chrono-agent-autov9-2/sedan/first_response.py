"""BMW E90 Sedan driving on flat rigid terrain (PyChrono wheeled-vehicle demo).

Model
-----
A catalog BMW E90 Sedan (`veh.Sedan` wrapper) is placed on a flat rigid-terrain
patch and driven forward by a scripted time-based driver that ramps throttle and
applies a gentle sinusoidal steering input, exercising real-time control of
steering, throttle, and braking. The wrapper owns an NSC `ChSystem`; contact
between the TMEASY tires and the rigid road uses the Bullet collision system.

System type
-----------
NSC (non-smooth contact), owned by the `veh.Sedan` wrapper.

Main bodies
-----------
- Sedan chassis (rigid body, geometric-center origin) + four wheels/spindles,
  suspension and steering links (all created inside the wrapper).
- A single flat rigid-terrain patch acting as the road/ground reference.

Expected behavior
------------------
After a brief settle the throttle ramps in, the car accelerates and translates
forward along +X while steering oscillates mildly, so the chassis X position and
forward speed both grow over the run.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 2e-3                     # integration step (s)
SIM_END = 12.0                       # total simulated time (s)
RENDER_FPS = 50.0                    # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: physics steps per frame

TERRAIN_LENGTH = 200.0               # road extent along X (m); sized to driven distance
TERRAIN_WIDTH = 40.0                 # road extent along Y (m)
TERRAIN_TOP_Z = 0.0                  # top surface of the rigid road (m)

SUSPENSION_REF_HEIGHT = 0.45         # chassis-origin height above wheel-bottom at rest (m)
INIT_X = -TERRAIN_LENGTH / 2 + 10.0  # spawn near the road's -X end, room to drive forward
INIT_Y = 0.0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT     # derived rest height
TIRE_RADIUS = 0.33                   # BMW E90 tire radius (m) for the footprint assert
ZTOL = 0.10                          # allowed wheel-bottom clearance/overlap vs road top (m)

THROTTLE_RAMP_START = 1.0            # begin accelerating after a short settle (s)
THROTTLE_TARGET = 0.7               # cruise throttle
STEER_AMPLITUDE = 0.15               # mild sinusoidal steering (-1..+1)
STEER_FREQ = 0.25                    # steering oscillation frequency (Hz)

INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT


# === Driver === scripted time-based control of steering / throttle / braking
class SedanDriver(veh.ChDriver):
    """Real-time control law: settle, then ramp throttle with mild steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_START:
            # Hold still briefly so the suspension settles onto the road.
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            # Ramp throttle to target over 1 s for a smooth launch.
            ramp = min(1.0, time - THROTTLE_RAMP_START)
            self.SetThrottle(THROTTLE_TARGET * ramp)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(2.0 * math.pi * STEER_FREQ * time))


# === Vehicle === BMW E90 Sedan wrapper (owns its NSC system + bodies/joints)
vehicle = veh.Sedan()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
vehicle.SetTireStepSize(TIME_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.Sedan wrapper) ===
system = vehicle.GetSystem()              # ChSystemNSC owned by the wrapper
# Use the Bullet narrow-phase for all tire/terrain contact in this scene.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused every step
veh_obj = vehicle.GetVehicle()            # cache: vehicle subsystem handle, reused for spindle queries
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links inside wrapper

# === Terrain === flat rigid road patch sized to the driven distance
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint check === verify wheels rest on (not through) the road after Initialize
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs road top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === scripted autonomous controller (no human-in-the-loop)
driver = SedanDriver(veh_obj)
driver.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene: window + sky + chase camera + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)   # follow point on chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Output / logging setup ===

frame = 0

# === Main loop === render once per frame; advance the full vehicle stack per step
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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + timeseries plot, then clean frames
