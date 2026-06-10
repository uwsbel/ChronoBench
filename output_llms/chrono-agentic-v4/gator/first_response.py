"""
Gator Vehicle on Flat Rigid Terrain — PyChrono 9.0 simulation.

Plan: mbs_in_scene (wrapper-managed vehicle scene).
System: ChSystemNSC with RigidTerrain contact.
Bodies: Gator vehicle (wheeled), rigid terrain patch.
Expected behavior: vehicle drives on flat terrain under driver control.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only: sim_recording for frame capture + CSV ===

# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

terrain_length = 200.0
terrain_width = 200.0

# === Paths — vehicle data files ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create Gator vehicle (NSC for rigid terrain) ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(time_step)
gator.Initialize()
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

sys = gator.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Essential components visible (system + bodies created by the wrapper) ===
chassis = gator.GetChassisBody()  # cache: main chassis body

# === Terrain (RigidTerrain, NSC material) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization — Irrlicht window for the vehicle ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Driver — ChInteractiveDriverIRR (interactive keyboard control) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Review-only: frame directory + CSV setup >>>

# === Main loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        # --- review-only CSV log (every physics step) ---

        sim_time = sys.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        gator.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        gator.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

        if sys.GetChTime() >= sim_end:
            break

# --- review-only post-loop: close CSV + assemble video + plot ---
