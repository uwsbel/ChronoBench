"""
Sedan turn 2: Two BMW E90 sedans on flat rigid terrain with sinusoidal steering.
- Second vehicle added with initial position/orientation.
- Terrain texture changed to concrete.jpg.
- Driver system created for the second vehicle.
- Sinusoidal steering input for both vehicles in the simulation loop.
- Updated synchronization and advancement for both vehicles and drivers.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Time ===
time_step = 1e-3
sim_end = 10.0

# === Vehicle 1 (main sedan) ===
sedan1 = veh.BMW_E90()
sedan1.SetContactMethod(chrono.ChContactMethod_NSC)
sedan1.SetChassisCollisionType(veh.CollisionType_NONE)
sedan1.SetChassisFixed(False)
init_rot_1 = chrono.QuatFromAngleZ(0)   # heading in +x
sedan1.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-6, 0, 0.5), init_rot_1))
sedan1.SetTireType(veh.TireModelType_RIGID)
sedan1.SetTireStepSize(time_step)
sedan1.Initialize()
print("VEHICLE MASS: ", sedan1.GetVehicle().GetMass())

# === Vehicle 2 (second sedan, shared system) ===
sedan2 = veh.BMW_E90(sedan1.GetSystem())
sedan2.SetContactMethod(chrono.ChContactMethod_NSC)
sedan2.SetChassisCollisionType(veh.CollisionType_NONE)
sedan2.SetChassisFixed(False)
init_rot_2 = chrono.QuatFromAngleZ(math.pi)   # heading in -x (facing opposite)
sedan2.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(6, 0, 0.5), init_rot_2))
sedan2.SetTireType(veh.TireModelType_RIGID)
sedan2.SetTireStepSize(time_step)
sedan2.Initialize()

system = sedan1.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain (RigidTerrain flat, concrete texture) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan Turn 2 - Two Vehicles")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(sedan1.GetVehicle())

# === Drivers ===
driver1 = veh.ChInteractiveDriverIRR(vis)
driver1.SetSteeringDelta(1.0 / 1.0)
driver1.SetThrottleDelta(1.0 / 1.0)
driver1.SetBrakingDelta(1.0 / 0.3)
driver1.Initialize()

driver2 = veh.ChInteractiveDriverIRR(vis)
driver2.SetSteeringDelta(1.0 / 1.0)
driver2.SetThrottleDelta(1.0 / 1.0)
driver2.SetBrakingDelta(1.0 / 0.3)
driver2.Initialize()

# === Cached handles ===
chassis1 = sedan1.GetChassisBody()   # cache
chassis2 = sedan2.GetChassisBody()   # cache

# === Simulation loop ===
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


frame = 0
while vis.Run() and sedan1.GetSystem().GetChTime() < sim_end:
    time = sedan1.GetSystem().GetChTime()

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs1 = driver1.GetInputs()
    driver_inputs2 = driver2.GetInputs()

    # Sinusoidal steering for both vehicles (scored core — scripted driver)
    steer1 = 0.25 * math.sin(2.0 * math.pi * 0.2 * time)
    steer2 = 0.25 * math.sin(2.0 * math.pi * 0.2 * time + math.pi)
    driver_inputs1.m_steering = steer1
    driver_inputs2.m_steering = steer2
    driver_inputs1.m_throttle = 0.3
    driver_inputs2.m_throttle = 0.3
    driver_inputs1.m_braking = 0.0
    driver_inputs2.m_braking = 0.0

    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    sedan1.Synchronize(time, driver_inputs1, terrain)
    sedan2.Synchronize(time, driver_inputs2, terrain)
    vis.Synchronize(time, driver_inputs1)

    driver1.Advance(time_step)
    driver2.Advance(time_step)
    terrain.Advance(time_step)
    sedan1.Advance(time_step)
    sedan2.Advance(time_step)
    vis.Advance(time_step)


    step_number += 1
    realtime_timer.Spin(time_step)
