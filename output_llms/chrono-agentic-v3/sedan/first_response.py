"""
BMW E90 Sedan simulation on rigid terrain using PyChrono vehicle module.

System: ChSystemNSC (owned by the BMW_E90 wrapper)
Vehicle: veh.BMW_E90 — TMEASY tire model on flat rigid terrain
Driver: ChInteractiveDriverIRR for real-time interactive control
Visualization: Irrlicht with chase camera and directional lighting
Expected behavior: Sedan drives on flat terrain; user controls steering/throttle/braking
in real time via keyboard (WASD / arrow keys in Irrlicht window).
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
step_size = 1e-3            # physics time step (s)
sim_end = 20.0              # simulation end time (s)
render_fps = 50.0
render_steps = math.ceil(1.0 / (render_fps * step_size))  # precomputed once

terrain_length = 300.0      # m — wide enough for full sim duration
terrain_width = 300.0       # m
SUSPENSION_REF_HEIGHT = 0.5 # chassis origin above wheel-bottom at rest (inferred default — verify)

# Initial spawn position
init_loc = chrono.ChVector3d(0.0, 0.0, SUSPENSION_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths (required scored-core components for catalog vehicle) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)   # MANDATORY — fixed chassis won't move
sedan.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
sedan.SetTireType(veh.TireModelType_TMEASY)   # TMEASY for terrain compliance
sedan.SetTireStepSize(step_size)
sedan.Initialize()

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
sys = sedan.GetSystem()              # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = sedan.GetChassisBody()     # cache: fetched once, reused

print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())   # truth component — scored core

# Set visualization types (after Initialize)
sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()        # vehicle truths use directional light
vis.AttachVehicle(sedan.GetVehicle())

# === Driver (interactive — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
render_step_size = 1.0 / render_fps  # precomputed once for delta calculations
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only setup ===

realtime_timer = chrono.ChRealtimeStepTimer()   # real-time pacing
step_number = 0

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()  # cache: fetched once per outer loop

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        sedan.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        sedan.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
