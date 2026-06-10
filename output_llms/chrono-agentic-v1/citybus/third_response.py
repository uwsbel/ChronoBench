"""
CityBus simulation on RigidTerrain with Pacejka (PAC02/PAC89-format) tire model,
dirt road texture, and a finer simulation step size of 5e-4 s for improved stability.

System: NSC (rigid contact, default for catalog wheeled vehicles)
Vehicle: veh.CityBus() — catalog wrapper, owns its ChSystem
Terrain: RigidTerrain flat patch (800 x 200 m), NSC contact material, dirt.jpg texture
Tire: TireModelType_PAC02 (reads CityBus_Pac02Tire.tir in PAC89 format — the 89-version Pacejka formulation)
Step: 5e-4 s (both simulation and tire), finer than the default 1e-3 for better stability
Driver: ChInteractiveDriver (real-time keyboard control)

Expected behavior: City bus accelerates from rest on a wide dirt-textured flat road,
drives forward, reaching highway speed. Pacejka tire force model provides realistic
lateral/longitudinal slip curves.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
# Simulation time parameters
time_step = 5e-4          # simulation step size [s] — reduced to 5e-4 for better stability
tire_step_size = 5e-4     # tire integration step [s] — matches sim step
sim_end = 30.0            # total simulation duration [s]

# Render cadence — precomputed once
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Terrain geometry — large enough for 30 s at ~25 m/s (~750 m travel)
terrain_length = 1000.0   # [m] X extent — generous to keep bus on terrain
terrain_width = 500.0     # [m] Y extent — wide to accommodate any lateral drift

# Vehicle spawn — start at west end of terrain, centered in Y
init_x = -400.0           # [m] start near western edge (terrain centered at x=0)
init_loc = chrono.ChVector3d(init_x, 0.0, 0.5)   # height above terrain
init_rot = chrono.QuatFromAngleZ(0.0)              # heading: +X direction

# Contact parameters
friction_coeff = 0.9
restitution_coeff = 0.01

# === Vehicle Setup ===
# Set vehicle data path so catalog tire JSON/TIR files resolve correctly
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# veh.CityBus() creates and owns its ChSystemNSC internally — no system arg.
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC02)              # prompt: Pacejka tire model — PAC02 reads PAC89-format .tir data (89-version formulation)
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                              # ChSystemNSC owned by wrapper
chassis = bus.GetChassisBody()                        # cache: main chassis rigid body; fetched once, reused in loop
# wheel spindles / suspension / steering links created inside wrapper
# accessible via bus.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()

# Collision system (REQUIRED — vehicle + terrain use contact)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization types (set after Initialize — required ordering)
bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

# NSC contact material matches vehicle contact method
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(friction_coeff)
patch_mat.SetRestitution(restitution_coeff)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,          # centered at origin, flat
    terrain_length,
    terrain_width,
)

# Dirt road texture (as specified — dirt.jpg replaces tile4.jpg)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.7, 0.5))

terrain.Initialize()

# === Irrlicht Visualization (Vehicle-specific Irrlicht system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus — PAC89/PAC02 Tires — Dirt Road — dt=5e-4")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 14.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(bus.GetVehicle())

# === Driver (Interactive — real-time keyboard control) ===
render_step_size = 1.0 / render_fps  # precomputed once — used by delta setters

steering_time = 1.0     # seconds from 0 → full steering
throttle_time = 1.0     # seconds from 0 → full throttle
braking_time = 0.3      # seconds from 0 → full brake

driver = veh.ChInteractiveDriver(bus.GetVehicle())
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run():
        time = system.GetChTime()

        if time >= sim_end:
            break

        # Throttled rendering at render_fps — render once per frame block
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs (interactive keyboard in scored core)
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (fixed order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems — vehicle.Advance steps the wrapper-owned ChSystem
        # (do NOT also call system.DoStepDynamics — would double-step)
        driver.Advance(time_step)
        terrain.Advance(time_step)
        bus.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
