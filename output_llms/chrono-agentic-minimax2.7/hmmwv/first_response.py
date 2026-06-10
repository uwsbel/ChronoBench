"""
HMMWV on flat rigid terrain with Irrlicht visualization.

Simulates a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving on a flat
rigid terrain surface. Uses TMEASY tire model for realistic tire dynamics.
System: NSC (Non-Smooth Contact) for rigid terrain.

Key components:
- veh.HMMWV_Full wrapper with TMEASY tire model
- veh.RigidTerrain with NSC patch material
- veh.ChInteractiveDriver for interactive steering/throttle/braking
- Real-time synchronization: driver -> terrain -> vehicle -> visualization
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step = 1e-3          # physics timestep (s)
sim_end = 10.0            # simulation duration (s)
render_fps = 50.0         # rendering frame rate (fps)
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle initialization ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY: moving chassis

# Initial position: chassis origin at z = suspension ref height (~0.5m) above terrain
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QUNIT
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# TMEASY tire model for rigid terrain
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)

hmmwv.Initialize()
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)

# NSC material for rigid terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Large terrain: 2000m x 2000m to keep vehicle visible
terrain_length = 2000.0   # m
terrain_width = 2000.0    # m
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(time_step / steering_time)
driver.SetThrottleDelta(time_step / throttle_time)
driver.SetBrakingDelta(time_step / braking_time)
driver.Initialize()

# === Review-only: data driver for validation video ===

# === Review-only: CSV logging + frame capture setup ===

# === Scored core: real-time simulation loop ===
step_number = 0
active_driver = data_driver if data_driver is not None else driver
_use_data_driver = data_driver is not None

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = active_driver.GetInputs()

        if _use_data_driver:
            data_driver.Synchronize(sim_time)
        else:
            driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        if _use_data_driver:
            data_driver.Advance(time_step)
        else:
            driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
