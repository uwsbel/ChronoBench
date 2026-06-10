"""
CityBus simulation with data-driven driver (ChDataDriver).

System type : NSC (rigid terrain, wheeled vehicle)
Vehicle     : veh.CityBus on RigidTerrain with a flat road patch
Driver      : veh.ChDataDriver — open-loop schedule:
              t=0.0  throttle=0.0, steering=0.0, braking=0.0
              t=0.1  throttle=1.0, steering=0.0, braking=0.0
              t=0.5  throttle=1.0, steering=0.7, braking=0.0
Expected    : Bus accelerates from rest at t=0.1 then steers to the left
              from t=0.5 onward, producing a curved trajectory.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths — required truth components for catalog vehicles ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
step_size        = 1e-3          # physics time step (s)
sim_end          = 20.0          # simulation end time (s)
render_fps       = 50.0
render_step_size = 1.0 / render_fps
render_steps     = math.ceil(render_step_size / step_size)  # precomputed once

terrainLength = 300.0
terrainWidth  = 300.0

init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QuatFromAngleZ(0.0)

# === Vehicle setup ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(step_size)
bus.Initialize()

bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.CityBus wrapper) ===
system  = bus.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = bus.GetChassisBody()           # cache: fetched once, reused every step
# wheels/spindles: bus.GetVehicle().GetAxle(i) ; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Data-driven driver ===
# Open-loop schedule defined per the task prompt (no time-response settings needed).
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),
])

driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus — Data Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        bus.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
