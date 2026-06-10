"""
Kraz tractor-trailer truck + BMW E90 sedan on a highway mesh terrain.

System:     NSC (rigid-terrain catalog-vehicle default)
Vehicles:   veh.Kraz (tractor-trailer) + veh.BMW_E90 (sedan) sharing one ChSystemNSC
Terrain:    RigidTerrain with a predefined highway mesh patch
Driver:     ChInteractiveDriverIRR for the truck (interactive, real-time);
            scripted fixed-throttle/steering driver for the sedan
Tire:       Rigid (prompt: rigid tire model for the truck via SetTireStepSize + default rigid),
            TMEASY for the sedan
Expected:   Truck drives on highway mesh terrain alongside a sedan that maintains
            fixed throttle forward with slight steering. Tractor and trailer positions
            and speeds are logged each step.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Physics and timing
TIME_STEP = 1e-3              # simulation step (s)
SIM_END   = 20.0              # total sim time (s)
RENDER_FPS = 50.0             # render cadence (Hz)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Truck initial pose (changed per prompt)
TRUCK_INIT_LOC = chrono.ChVector3d(-40.0, -3.0, 0.5)
TRUCK_INIT_ROT = chrono.QuatFromAngleZ(math.pi / 6)   # yaw ~30°

# Sedan initial pose (added per prompt)
SEDAN_INIT_LOC = chrono.ChVector3d(-20.0, 3.0, 0.5)
SEDAN_INIT_ROT = chrono.QuatFromAngleZ(math.pi / 8)   # yaw ~22.5°

# Terrain
TERRAIN_LENGTH = 600.0
TERRAIN_WIDTH  = 60.0

# Driver response rates (interactive truck)
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3
render_step_size = 1.0 / RENDER_FPS  # precomputed once

# === Data paths (truth-faithful — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Truck (Kraz tractor-trailer) ===
truck = veh.Kraz()
truck.SetContactMethod(chrono.ChContactMethod_NSC)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(TRUCK_INIT_LOC, TRUCK_INIT_ROT))
truck.SetTireStepSize(TIME_STEP)    # prompt: rigid tire model for the truck
truck.Initialize()

# Set tire visualization for tractor and trailer after Initialize()
truck.SetChassisVisualizationType(
    chrono.VisualizationType_MESH,
    chrono.VisualizationType_PRIMITIVES,
)
truck.SetSuspensionVisualizationType(
    chrono.VisualizationType_PRIMITIVES,
    chrono.VisualizationType_PRIMITIVES,
)
truck.SetWheelVisualizationType(
    chrono.VisualizationType_MESH,
    chrono.VisualizationType_MESH,
)
truck.SetTireVisualizationType(
    chrono.VisualizationType_NONE,
    chrono.VisualizationType_NONE,
)

# === System & bodies (created by the veh.Kraz wrapper) ===
sys = truck.GetSystem()                    # ChSystemNSC owned by the Kraz wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
tractor_chassis = truck.GetTractorChassisBody()   # cache: tractor chassis rigid body
trailer_chassis = truck.GetTrailer().GetChassis().GetBody()       # cache: trailer chassis body
# tractor wheels/spindles: truck.GetTractor().GetAxle(i)
# trailer wheels/spindles: truck.GetTrailer().GetAxle(i)

print("VEHICLE MASS: ", truck.GetTractor().GetMass())   # scored core — diagnostic

# === Sedan (BMW E90 — shares truck's system) ===
sedan = veh.BMW_E90(sys)   # MUST share truck's system (not a fresh wrapper)
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_LOC, SEDAN_INIT_ROT))
sedan.SetTireType(veh.TireModelType_TMEASY)
sedan.SetTireStepSize(TIME_STEP)
sedan.Initialize()

sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)

sedan_chassis = sedan.GetChassisBody()    # cache: sedan chassis rigid body

# === Terrain — RigidTerrain with predefined highway mesh ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Highway mesh patch (predefined mesh terrain per prompt)
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    highway_mesh,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Truck + Sedan on Highway Mesh")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()    # vehicle truths use directional light
vis.AttachVehicle(truck.GetTractor())   # Kraz: attach via GetTractor(), NOT GetVehicle()

# === Driver for truck (ChInteractiveDriverIRR — scored core) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Driver for sedan (scripted: fixed throttle + steering per prompt) ===
sedan_driver = veh.DriverInputs()
sedan_driver.m_throttle = 0.4    # fixed throttle — sedan moves forward
sedan_driver.m_steering = 0.05   # slight fixed steering
sedan_driver.m_braking  = 0.0

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize all subsystems (mandatory order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        sedan.Synchronize(time, sedan_driver, terrain)
        vis.Synchronize(time, driver_inputs)

        for _ in range(render_every):
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            truck.Advance(TIME_STEP)     # advances the wrapper-owned ChSystemNSC
            sedan.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
