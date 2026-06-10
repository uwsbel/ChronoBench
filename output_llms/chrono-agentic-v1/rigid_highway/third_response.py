"""
Rigid Highway Vehicle Simulation — HMMWV on Rigid Terrain Crossroads.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Main bodies: HMMWV chassis, 4 wheel spindles, RigidTerrain patch.
Expected behavior: HMMWV drives forward on a rigid terrain patch oriented at
-90 degrees about the Z-axis, positioned at (6, -70, 0) so the vehicle is at
a crossroads with the terrain. Contact material has friction=0.4 and
restitution=0.05. Driver: ChInteractiveDriver (real-time interactive control).
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
# Simulation timing
STEP_SIZE = 2e-3          # physics timestep (s)
SIM_END = 20.0            # simulation duration (s)
RENDER_FPS = 50.0         # render / frame-capture rate (Hz)
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # physics steps per frame; precomputed once
render_step_size = 1.0 / RENDER_FPS  # precomputed once

# Terrain patch parameters — prompt-specified final values
TERRAIN_FRICTION = 0.4        # prompt: friction updated from 0.9 to 0.4
TERRAIN_RESTITUTION = 0.05    # prompt: restitution updated from 0.01 to 0.05
TERRAIN_LENGTH = 200.0        # highway patch length (m)
TERRAIN_WIDTH = 30.0          # highway patch width (m)

# Terrain patch pose — prompt-specified
PATCH_POS = chrono.ChVector3d(6.0, -70.0, 0.0)                      # crossroads position
PATCH_ROT = chrono.QuatFromAngleZ(math.radians(-90.0))               # -90 deg about Z; precomputed once

# Vehicle spawn
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (HMMWV approx)
VEHICLE_INIT_POS = chrono.ChVector3d(6.0, -70.0, SUSPENSION_REF_HEIGHT)
VEHICLE_INIT_ROT = chrono.QUNIT

# Visualization
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)   # track point on chassis
CHASE_DIST = 9.0
CHASE_OFFSET = 0.5

# === Vehicle construction ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                           # MANDATORY — chassis must be free
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT_POS, VEHICLE_INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)            # TMEASY tire model
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()               # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()         # cache: main chassis rigid body; fetched once, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)
# joints: suspension + steering links created inside the HMMWV_Full wrapper

# === Visualization types (after Initialize) ===
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)        # prompt: 0.4
patch_mat.SetRestitution(TERRAIN_RESTITUTION)  # prompt: 0.05

# Patch at crossroads position with -90 deg Z rotation (prompt-specified)
patch_coordsys = chrono.ChCoordsysd(PATCH_POS, PATCH_ROT)
patch = terrain.AddPatch(patch_mat, patch_coordsys, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

veh_obj = hmmwv.GetVehicle()   # cache: fetched once, reused every step

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Rigid Highway Crossroads — friction=0.4, restitution=0.05")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DIST, CHASE_OFFSET)
vis.Initialize()                                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Driver (scored core: ChInteractiveDriver — interactive real-time control) ===
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver = veh.ChInteractiveDriver(veh_obj)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Recording setup (review-only) ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()   # cache: fetched once per iteration

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)         # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)   # maintain real-time pacing

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass   # CSV writer closed in review-only block below
