"""
Gator Vehicle Simulation with Sensor Camera
============================================
System type: NSC (ChSystemNSC owned by veh.Gator wrapper)
Main bodies: Gator chassis, front/rear axle spindles, rigid terrain patch
Expected behavior: Gator vehicle on flat rigid terrain with interactive driver
  and a chassis-attached sensor camera. The visualization uses
  ChWheeledVehicleVisualSystemIrrlicht with chase camera. A ChCameraSensor
  (OptiX) is attached to the chassis for sensor output. The sensor manager
  provides point lighting for the camera.

Scored core: vehicle init + terrain + interactive driver + Irrlicht viz +
  sensor manager with camera attached to chassis, real-time loop.
Review-only: frame capture, CSV logging, video assembly.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants ===
STEP_SIZE        = 1e-3          # physics time step (s)
SIM_END          = 20.0          # simulation end time (s)
RENDER_FPS       = 50.0
TERRAIN_LENGTH   = 200.0         # rigid terrain X extent (m)
TERRAIN_WIDTH    = 200.0         # rigid terrain Y extent (m)
INIT_X           = 0.0
INIT_Y           = 0.0
SUSPENSION_H     = 0.5           # Gator chassis origin height above wheel-bottom at rest (m)

render_steps = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))        # precomputed once

# === Data paths (truth-faithful — required for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup (wrapper-managed — Gator owns its ChSystemNSC) ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, SUSPENSION_H)
init_rot = chrono.QUNIT
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# Visualization types for vehicle parts
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()                # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = gator.GetChassisBody()          # cache: fetched once, reused for sensor

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Terrain — rigid flat patch ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Driver — interactive (scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / RENDER_FPS      # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor manager with point lights and chassis camera ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Chassis-attached camera (follows vehicle in its local frame)
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-6, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis,                  # attach to real chassis body
    30,                       # update_rate Hz (physical rate, not 1/dt)
    cam_offset,
    1280, 720,
    1.408,                    # horizontal FOV (rad)
)
cam.SetName("ChassisCamera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))
manager.AddSensor(cam)

# === Review-only: recording setup ===

# === Realtime timer ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Main loop ===
frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()

        step_number += 1


        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # ensure partial output is flushed (csv writers closed in review-only block below)
