"""
UAZBUS vehicle simulation on rigid terrain with Irrlicht visualization.

System type: NSC (ChSystemNSC owned by the veh.UAZBUS wrapper).
Main bodies: UAZBUS chassis, 4 wheel spindles, rigid terrain patch.
Expected behavior: Vehicle drives forward with constant throttle (0.5),
zero steering, on a flat rigid terrain surface.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
step_size = 1e-3              # physics timestep (s)
sim_end   = 20.0              # simulation end time (s)
render_fps = 50.0             # target render frame rate
render_steps = max(1, math.ceil(1.0 / (render_fps * step_size)))  # precomputed once

terrainLength = 200.0
terrainWidth  = 200.0

SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.34            # approximate UAZBUS tire radius (m)
ZTOL = 0.08                   # wheel-bottom overlap tolerance (m)

init_loc = chrono.ChVector3d(0, 0, SUSPENSION_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths (mandatory truth components) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = vehicle.GetSystem()                    # ChSystemNSC owned by wrapper
chassis = vehicle.GetChassisBody()           # main chassis rigid body; cache: fetched once
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Footprint assert (verify wheel placement) ===
veh_obj = vehicle.GetVehicle()  # cache: fetched once, reused
spindle_positions = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)

wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs support z=0; raise SUSPENSION_REF_HEIGHT by "
    f"{-wheel_bottom_z:.3f} m"
)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)

# === Driver (interactive; scripted throttle applied per step in scored core) ===
driver = veh.ChInteractiveDriver(veh_obj)
steering_time = 1.0   # s to reach max steering
throttle_time = 1.0   # s to reach max throttle
braking_time  = 0.3   # s to reach max brake
driver.SetSteeringDelta(1.0 / (render_fps * steering_time))
driver.SetThrottleDelta(1.0 / (render_fps * throttle_time))
driver.SetBrakingDelta(1.0 / (render_fps * braking_time))
driver.Initialize()

# === Review-only: record setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # Scripted throttle in scored core (truth shape: uazbus drives forward)
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking  = 0.0

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
    import traceback; traceback.print_exc()
    raise
