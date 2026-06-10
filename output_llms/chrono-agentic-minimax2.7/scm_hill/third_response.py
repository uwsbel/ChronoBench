"""
HMMWV driving on rigid terrain with heightmap bump surface.
System type: NSC (Non-Smooth Contact).
Main bodies: HMMWV_Full rigid vehicle, rigid terrain patch with bump64 heightmap.
Expected behavior: vehicle drives over bumpy terrain surface, suspension reacts to height variations.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 20.0          # 20 FPS render, ~50 physics steps per frame
sim_end = 30.0                          # review-run duration

initLoc = chrono.ChVector3d(-15.0, 0.0, 1.2)
initRot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID
contact_method = chrono.ChContactMethod_NSC  # changed from SMC per input3

render_every = max(1, round(render_step_size / step_size))  # precomputed once

# === System & gravity ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle (HMMWV_Full — wrapper owns its system) ===
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

system = vehicle.GetSystem()  # cache: ChSystemNSC owned by wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain — rigid terrain with single heightmap patch (replaces SCM per input3) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40.0, 40.0, -1.0, 1.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
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

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Review-only recording ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))  # controls frame capture


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    if REC:  # review-only frame capture
        vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
        frame += 1

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

        if system.GetChTime() >= sim_end:
            break

# === Review-only post-loop ===
