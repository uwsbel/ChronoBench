"""
FEDA Vehicle Simulation on Rigid Terrain with Irrlicht Visualization.

System type: ChSystemSMC (FEDA wrapper uses SMC contact method)
Vehicle: FED-Alpha (veh.FEDA) on a flat RigidTerrain patch
Driver: ChInteractiveDriverIRR — real-time keyboard steering/throttle/braking
Camera: Chase camera following the vehicle chassis (SetChaseCamera + AttachVehicle before Initialize)
Visualization: Mesh for all vehicle parts; Irrlicht window at 50 FPS
Expected behavior: Vehicle rests on terrain, user steers/drives interactively;
                   chassis, suspension, wheels, and tires rendered as meshes.
                   Chase camera follows vehicle as it drives forward.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Vehicle data path ===
# Required so TIR and JSON files are resolved by absolute path (not relative)
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
STEP_SIZE = 1.0 / 1000.0            # physics time step (1 kHz)
SIM_END = 30.0                       # simulation duration (s)
RENDER_FPS = 50.0                    # frames per second for rendering
RENDER_STEP_SIZE = 1.0 / RENDER_FPS  # s per render frame
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

# Vehicle initial spawn position
INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5          # FEDA chassis origin above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0                  # terrain surface at z=0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT

# Terrain dimensions (large enough for full 30s run at ~25 m/s)
TERRAIN_LENGTH = 1000.0              # m (X)
TERRAIN_WIDTH = 200.0                # m (Y)

# Chase camera parameters
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)    # track point on chassis
CHASE_DISTANCE = 9.0                 # camera distance behind vehicle
CHASE_HEIGHT = 0.5                   # camera height offset

# Driver response times (s to reach full deflection)
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3

# === Vehicle setup (FEDA wrapper — owns its ChSystem internally) ===
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)

feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_SMC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)          # MANDATORY — chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)    # PAC02: matches FEDA TIR data
feda.SetTireStepSize(STEP_SIZE)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()            # ChSystemSMC owned by the wrapper
chassis = feda.GetChassisBody()      # cache: main chassis rigid body, fetched once
# wheels/spindles: feda.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

# REQUIRED: enable collision system after Initialize() for contact/terrain scenes
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Assert wheel-bottom height is on (not through) the terrain surface
veh_obj = feda.GetVehicle()          # cache: inner vehicle object for spindle queries
TIRE_RADIUS = 0.33                   # approximate tire radius for FEDA
ZTOL = 0.10                          # allowed gap/overlap tolerance vs terrain top
spindle_positions = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)
wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Visualization types (mesh for all vehicle parts) ===
# In this 9.0.0 build, VisualizationType_* are in the veh namespace
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (RigidTerrain with custom texture) ===
terrain = veh.RigidTerrain(system)

# SMC contact material matching FEDA's contact method
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,        # flat patch centered at origin
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Custom texture applied to the terrain patch (tile pattern)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization (vehicle-aware, chase camera follows vehicle) ===
# AttachVehicle BEFORE Initialize so the chase camera tracks the vehicle correctly
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA Vehicle on Rigid Terrain — Irrlicht")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
vis.AttachVehicle(feda.GetVehicle())  # attach before Initialize for chase cam to track
vis.Initialize()                      # Initialize FIRST after window config (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
# Explicitly set chase camera state so it follows the vehicle
vis.SetChaseCameraState(veh.ChChaseCamera.Chase)

# === Interactive driver (keyboard steering / throttle / braking) ===
driver = veh.ChInteractiveDriverIRR(vis)    # takes the visual system (not the vehicle)
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)    # precomputed once
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)    # precomputed once
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)      # precomputed once
driver.Initialize()

# === Main loop (real-time, 50 FPS render cadence) ===
realtime_timer = chrono.ChRealtimeStepTimer()    # keeps wall-clock in sync with sim time
step_number = 0
frame = 0

try:
    while vis.Run():
        time = system.GetChTime()
        if time >= SIM_END:
            break

        # Throttled rendering at RENDER_FPS (50 Hz)
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Collect driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in fixed order: driver → terrain → vehicle → vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance all subsystems (feda.Advance steps the wrapper-owned ChSystem)
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        feda.Advance(STEP_SIZE)          # advances the ChSystem — do NOT also call system.DoStepDynamics
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)   # keep wall-clock in sync with sim time

except (RuntimeError, ValueError) as exc:    # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
