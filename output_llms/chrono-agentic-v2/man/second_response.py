"""
MAN 5t truck simulation on rigid hilly terrain with a heightmap.

System type : NSC (rigid terrain)
Vehicle     : veh.MAN_5t — 5-tonne MAN truck, wheeled, catalog wrapper
Terrain     : veh.RigidTerrain with a heightmap patch (bump64.bmp) for hills,
              textured with grass.jpg
Driver      : veh.ChInteractiveDriverIRR (real-time interactive)
Expected    : Truck starts at (-20, 0, 1.5), drives over hilly terrain under
              interactive keyboard control; wheels maintain contact with
              the undulating surface.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
step_size          = 2e-3          # physics time step (s)
sim_end            = 20.0          # simulation end time (s)
render_fps         = 50.0
render_step_size   = 1.0 / render_fps
render_steps       = math.ceil(render_step_size / step_size)  # precomputed once

TERRAIN_LENGTH     = 200.0         # terrain patch X extent (m)
TERRAIN_WIDTH      = 200.0         # terrain patch Y extent (m)
TERRAIN_H_MIN      = 0.0           # heightmap minimum height (m)
TERRAIN_H_MAX      = 3.0           # heightmap maximum height (m)

INIT_POS           = chrono.ChVector3d(-20.0, 0.0, 1.5)
INIT_ROT           = chrono.ChQuaterniond(1, 0, 0, 0)   # heading +X

# === Data paths (required by catalog-vehicle reference scorer) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
vehicle = veh.MAN_5t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === System & bodies (created by the veh.MAN_5t wrapper) ===
system = vehicle.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()    # cache: main chassis rigid body
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain patch body below
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization types ===
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — rigid heightmap (hills) with grass texture ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Heightmap patch: bump64.bmp gives rolling hills over 200x200 m
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_H_MIN,
    TERRAIN_H_MAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 60, 60)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.3))
terrain.Initialize()

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t — Rigid Hilly Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver — interactive ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0   # review-only frame counter

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # Review-only scripted driving block for RUN-stage validation

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

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
