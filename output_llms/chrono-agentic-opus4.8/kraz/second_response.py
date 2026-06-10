"""Kraz tractor-trailer truck performing a double lane change on flat rigid terrain.

Models the catalog Kraz heavy truck (tractor + semi-trailer) on a 100x100 m flat
RigidTerrain patch, using an NSC contact system and TMEASY tires. The vehicle spawns
at (-15, 0, 0.5) facing +X and executes a time-scheduled double-lane-change maneuver
(steer left, recover, steer right, recover, then brake) under constant throttle. A
chase camera follows the tractor chassis. Expected behavior: the truck accelerates
forward along +X, weaves left then right through the lane-change, and slows to a stop.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths (locate bundled Chrono + vehicle assets) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Named constants: spawn, terrain, timing, camera ===
init_loc = chrono.ChVector3d(-15, 0, 0.5)        # spawn shifted to -X so the maneuver fits the patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # identity: facing +X

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

terrain_height = 0.0
terrain_length = 100.0                           # X size of the flat patch
terrain_width = 100.0                            # Y size of the flat patch

track_point = chrono.ChVector3d(3, 0, 2.1)       # chase-camera aim point on the tractor
chase_distance = 25.0                            # camera distance behind the truck
chase_height = 10.5                              # camera height offset

contact_method = chrono.ChContactMethod_NSC      # rigid-terrain catalog truck uses NSC
step_size = 1e-3
tire_step_size = step_size
sim_end = 12.0                                   # bounded run covers the full maneuver + braking
render_step_size = 1.0 / 50                      # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)   # precomputed once: steps per frame

# === Vehicle (Kraz tractor-trailer) ===
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                   # MANDATORY — a fixed chassis never moves
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireStepSize(tire_step_size)          # Kraz uses its built-in TMEASY tire model
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

# === System & bodies (created and owned by the veh.Kraz wrapper) ===
system = vehicle.GetSystem()                      # ChSystemNSC owned by the wrapper
tractor = vehicle.GetTractor()                    # cache: tractor sub-vehicle, reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for terrain contact
# wheels/spindles + suspension/steering joints are created inside the wrapper; trailer is GetTrailer()

print("VEHICLE MASS: ", tractor.GetMass())        # report total tractor mass

# === Terrain (flat rigid patch) ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length, terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (vehicle-aware Irrlicht: window + chase camera + sky + light) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Double Lane Change')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, chase_distance, chase_height)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()                          # vehicle truths use a directional light
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

# === Driver (interactive driver bound to the vis; maneuver scripted in the loop) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # seconds to go 0 -> +-1 steering
throttle_time = 1.0   # seconds to go 0 -> +1 throttle
braking_time = 0.3    # seconds to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop ===

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Time-scheduled double-lane-change maneuver under constant throttle.
        if time < 2.0:
            driver.SetSteering(0.0)
            driver.SetThrottle(0.6)
        elif time < 4.0:
            driver.SetSteering(0.3)
            driver.SetThrottle(0.6)
        elif time < 7.0:
            driver.SetSteering(0.0)
            driver.SetThrottle(0.6)
        elif time < 8.0:
            driver.SetSteering(-0.3)
            driver.SetThrottle(0.6)
        elif time < 10.0:
            driver.SetSteering(0.0)
            driver.SetThrottle(0.6)
        else:
            driver.SetBraking(1.0)

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

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
        realtime_timer.Spin(step_size)            # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:         # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
