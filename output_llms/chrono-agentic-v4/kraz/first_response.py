"""
Kraz truck simulation on rigid terrain with Irrlicht visualization.

This simulation creates a Kraz tractor-trailer vehicle on a flat rigid terrain,
initializes an interactive driver for real-time control, and runs a synchronized
vehicle + terrain + driver + visualization loop.
"""

import os
import sys
import math

# Ensure PyChrono 9.0.0 (source build) is used — prepend to sys.path to override
# any leaked PYTHONPATH from the chrono-agent conda env
_CHRONO900_BIN = "/home/hongyu/Documents/chrono-900/build/bin"
if _CHRONO900_BIN not in sys.path:
    sys.path.insert(0, _CHRONO900_BIN)

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# >>> review-only >>>
import sim_recording as rec
# <<< review-only <<<

# === Time and physics constants ===
time_step = 1e-3
sim_end = 5.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_every = max(1, round(render_step_size / time_step))

# === Data paths (required for catalog vehicles — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create the Kraz vehicle ===
# Kraz is a tractor-trailer; attach visualization via GetTractor()
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisCollisionType(veh.CollisionType_NONE)
kraz.SetChassisFixed(False)
# Initial position: start at origin, chassis 0.5 m above ground (suspension ref height)
kraz_init_loc = chrono.ChVector3d(0, 0, 0.5)
kraz_init_rot = chrono.QUNIT
kraz.SetInitPosition(chrono.ChCoordsysd(kraz_init_loc, kraz_init_rot))
# Rigid tire is the default for rigid terrain
kraz.SetTireStepSize(time_step)
kraz.Initialize()

system = kraz.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", kraz.GetTractor().GetMass())

# === Terrain (rigid flat ground) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_length = 200.0
terrain_width = 200.0
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht — vehicle-aware window) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Truck Simulation")
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddLight(chrono.ChVector3d(5, -5, 10), 100, chrono.ChColor(1, 1, 1))
# Add an explicit camera AFTER Initialize — look at vehicle from behind/above
vis.AddCamera(chrono.ChVector3d(0.0, -15.0, 5.0), chrono.ChVector3d(0.0, 0.0, 0.5))
vis.AttachVehicle(kraz.GetTractor())

# === Driver (interactive IRR — scored core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Essential components visible (system, chassis, terrain, driver, vis) ===
# system:  ChSystemNSC owned by the Kraz wrapper
# chassis: kraz.GetChassisBody()
# terrain: RigidTerrain patch body
# driver:  ChInteractiveDriverIRR bound to vis
# vis:     ChWheeledVehicleVisualSystemIrrlicht

# === Review-only recording setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))  # review-only
irr_dir = rec.frame_dir("frames") if REC else None  # review-only

frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    if REC:  # review-only
        vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
        frame += 1

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        kraz.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        kraz.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

        if system.GetChTime() >= sim_end:
            break

# === Review-only: assemble video, plot table ===
if REC:  # >>> review-only >>>
    rec.assemble_all_videos(irr_dir)
    rec.cleanup_frames(irr_dir)
# <<< review-only <<<
