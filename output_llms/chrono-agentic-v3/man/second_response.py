"""
MAN 5t truck simulation on a rigid heightmap hill terrain.

System: NSC (wheeled vehicle wrapper, MAN_5t)
Vehicle: MAN 5t medium-duty truck, spawned at (-20, 0, 1.5)
Terrain: RigidTerrain with bump64.bmp heightmap patch (hills), grass texture
Driver: ChInteractiveDriverIRR (real-time interactive, review-only scripted maneuver)
Expected behavior: MAN 5t truck drives over hilly rigid terrain with grass texture.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


# === Constants ===
step_size = 1e-3                              # physics time step (s)
tire_step_size = step_size
sim_end = 20.0                                # simulation duration (s)
render_step_size = 1.0 / 50.0                 # render at 50 fps
render_steps = math.ceil(render_step_size / step_size)  # precomputed once

TERRAIN_LENGTH = 64.0                         # heightmap terrain length (m)
TERRAIN_WIDTH  = 64.0                         # heightmap terrain width (m)
TERRAIN_HMIN   = 0.0                          # heightmap minimum height (m)
TERRAIN_HMAX   = 3.0                          # heightmap maximum height (m)

INIT_LOC = chrono.ChVector3d(-20, 0, 1.5)     # initial vehicle location per prompt
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # no rotation (facing +X)
track_point = chrono.ChVector3d(-3.0, 0.0, 1.1)  # chase camera target point

vis_type = veh.VisualizationType_MESH         # vehicle visual fidelity
chassis_collision_type = veh.CollisionType_NONE

# === Data paths (required for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
vehicle = veh.MAN_5t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                         # MANDATORY
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# === System & bodies (created by the veh.MAN_5t wrapper) ===
sys = vehicle.GetSystem()                              # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()                    # cache: fetched once, reused each step
# wheels/spindles: vehicle.GetVehicle().GetAxle(i)...; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain — rigid hills via bump64 heightmap, grass texture ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HMIN,
    TERRAIN_HMAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht — vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver (interactive — scored-core default) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # seconds to reach max steering
throttle_time = 1.0   # seconds to reach max throttle
braking_time  = 0.3   # seconds to reach max braking

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
