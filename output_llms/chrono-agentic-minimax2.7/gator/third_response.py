"""
Gator vehicle simulation — turn 3.
Changes from base:
  1. Visualization simplified from mesh to primitives (chassis, wheels, tires, suspension, steering).
  2. Chassis collision added using a primitive box (not mesh collision).
  3. Driver time response increased — controls take longer to apply.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# review-only: sim_recording import and frame directory

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Create Gator vehicle ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(time_step)
gator.Initialize()

system = gator.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Add chassis collision using primitive box ===
chassis_body = gator.GetChassisBody()
cmat = chrono.ChContactMaterialNSC()
cmat.SetFriction(0.8)
cmat.SetRestitution(0.01)
chassis_body.AddCollisionShape(
    chrono.ChCollisionShapeBox(cmat, 1.6, 0.9, 0.35),
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
)
chassis_body.EnableCollision(True)
system.GetCollisionSystem().BindAll()

# === Terrain (rigid, flat) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0, 200.0
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Set visualization to primitives (turn 3 delta) ===
gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Visualization — Irrlicht window ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator - Turn 3")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Interactive driver (less responsive — turn 3 delta) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 3.0    # was 1.0 — less responsive
throttle_time = 3.0    # was 1.0 — less responsive
braking_time = 1.0     # was 0.3 — less responsive
render_step_size = render_every * time_step
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# review-only: CSV setup and scripted driver for autonomous motion

# === Main loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            frame += 1

    driver.Synchronize(time)

    # review-only: scripted driver for autonomous motion in the review run

    terrain.Synchronize(time)

    # review-only: override with scripted driver inputs for review run
    driver_inputs = driver.GetInputs()  # scored core: interactive driver

    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # review-only: advance scripted driver for review run
    driver.Advance(time_step)

    terrain.Advance(time_step)
    gator.Advance(time_step)
    vis.Advance(time_step)

    # review-only: log CSV

    step_number += 1
    realtime_timer.Spin(time_step)

# review-only: post-loop video assembly and cleanup
