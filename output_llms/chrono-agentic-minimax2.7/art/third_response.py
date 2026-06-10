"""
ARTcar vehicle on rigid terrain with adjusted vehicle parameters.
Simulation featuring an ARTcar with modified motor and tire properties
to achieve increased speed: higher max motor voltage ratio, increased
stall torque, and reduced tire rolling resistance.
"""
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle parameters ===
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc = chrono.ChVector3d(0, 0, 0.5)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# === Terrain parameters ===
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# === Camera tracking ===
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

# === Contact setup ===
contact_method = chrono.ChContactMethod_NSC

# === Time stepping ===
step_size = 1e-3
tire_step_size = step_size
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_every = max(1, round(render_step_size / step_size))  # precomputed once

# === Vehicle creation ===
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
# Parameter adjustments from input3.txt (turn 3):
# MaxMotorVoltageRatio: 0.16 -> 0.26
# StallTorque: 0.3 -> 0.4
# TireRollingResistance: 0.06 -> 0.03
vehicle.SetMaxMotorVoltageRatio(0.26)
vehicle.SetStallTorque(0.4)
vehicle.SetTireRollingResistance(0.03)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Post-init diagnostic ===
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === Simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
sim_end = 20.0  # bounded run for review

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(vehicle.GetSystem().GetChTime())
    terrain.Synchronize(vehicle.GetSystem().GetChTime())
    vehicle.Synchronize(vehicle.GetSystem().GetChTime(), driver_inputs, terrain)
    vis.Synchronize(vehicle.GetSystem().GetChTime(), driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

# === Review-only post-processing ===
