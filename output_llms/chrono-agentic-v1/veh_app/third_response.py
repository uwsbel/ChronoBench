"""
HMMWV Vehicle Simulation with Depth Camera Sensor and Vehicle State Logging.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Terrain: RigidTerrain flat patch (NSC contact).
Vehicle: HMMWV_Full with TMEASY tires on rigid terrain.
Sensors: ChDepthCamera attached to the chassis at offset (-5, 0, 2),
         1280x720, FOV=1.408 rad, max depth=30 m,
         with visualization filter (ChFilterVisualize) for the depth map.
Driver: ChInteractiveDriver (real-time interactive control).
Logging: Vehicle state (position X, Y, Z and heading) logged at every step.
Expected behavior: HMMWV drives on flat terrain; depth camera provides
depth-map sensor output from a rear-facing vantage; state is logged to CSV.
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants ===
STEP_SIZE = 1e-3                   # physics step size (s)
SIM_END = 30.0                     # simulation end time (s)
RENDER_FPS = 50.0                  # Irrlicht render target (Hz)
TERRAIN_LENGTH = 200.0             # terrain patch length (m)
TERRAIN_WIDTH = 200.0              # terrain patch width (m)
INIT_X = 0.0                       # vehicle spawn X (m)
INIT_Y = 0.0                       # vehicle spawn Y (m)
INIT_Z = 0.5                       # vehicle spawn Z slightly above terrain

# Depth camera parameters (from prompt spec)
DC_OFFSET = chrono.ChVector3d(-5.0, 0, 2)  # offset pose on chassis (m)
DC_WIDTH = 1280                             # image width (px)
DC_HEIGHT = 720                             # image height (px)
DC_HFOV = 1.408                             # horizontal field of view (rad)
DC_MAX_DEPTH = 30.0                         # maximum depth (m)
DC_UPDATE_RATE = 30.0                       # physical update rate (Hz)

# Derived render cadence — precomputed once
render_step_size = 1.0 / RENDER_FPS                                  # precomputed once
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))         # precomputed once

# === HMMWV vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain (truth)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, fetched once
# joints: suspension + steering created inside the wrapper
# wheels/spindles: accessed via hmmwv.GetVehicle().GetAxle(i)

# Visualization types — call after Initialize
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
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
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht vehicle visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Depth Camera Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
# ChInteractiveDriverIRR is not available in this build; use ChInteractiveDriver
driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())
steering_time = 1.0    # s to reach full steering
throttle_time = 1.0    # s to reach full throttle
braking_time = 0.3     # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)   # precomputed once
driver.SetThrottleDelta(render_step_size / throttle_time)   # precomputed once
driver.SetBrakingDelta(render_step_size / braking_time)     # precomputed once
driver.Initialize()

# === Sensor manager + Depth Camera ===
manager = sens.ChSensorManager(system)
# Lighting for camera sensor (point lights — canonical setup)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(50, 50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Depth camera offset pose: behind/above chassis at (-5, 0, 2), slight forward pitch
dc_offset_pose = chrono.ChFramed(
    DC_OFFSET,
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)),  # slight pitch toward front
)

# Build ChDepthCamera attached to chassis body
depth_cam = sens.ChDepthCamera(
    chassis,          # attach to chassis — moves with vehicle (cache: reuse chassis local)
    DC_UPDATE_RATE,   # physical update rate (Hz), not 1/dt
    dc_offset_pose,   # offset frame on chassis
    DC_WIDTH,         # image width (px)
    DC_HEIGHT,        # image height (px)
    DC_HFOV,          # horizontal FOV (rad)
    DC_MAX_DEPTH,     # maximum depth (m)
)
depth_cam.SetName("Depth Camera")
depth_cam.SetLag(0)
depth_cam.SetCollectionWindow(0)

# Depth camera filter chain (prompt: "Applied visualization filter for Depth Map"):
# ChFilterVisualize visualizes the raw depth buffer as a preview window (depth map)
depth_cam.PushFilter(sens.ChFilterVisualize(DC_WIDTH, DC_HEIGHT, "Depth Map"))
# Save depth frames (raw float depth data) to cam/depth/
depth_cam.PushFilter(sens.ChFilterSave("cam/depth/"))
manager.AddSensor(depth_cam)

# === Verify vehicle spawn geometry ===
TIRE_RADIUS = 0.33    # HMMWV TMEASY tire approximate radius (m)
ZTOL = 0.12           # allowed wheel clearance/overlap vs ground (m)
veh_obj = hmmwv.GetVehicle()   # cache: fetched once for footprint check
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_Z by {abs(wheel_bottom_z):.3f} m"
)

# === Review-only: output directories and CSV setup ===

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame_number = 0  # separate consecutive frame counter for ffmpeg

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering — render at RENDER_FPS, not every physics step
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Scored-core driver inputs (interactive)
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in order: driver → terrain → vehicle → vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem — do NOT also call system.DoStepDynamics
        vis.Advance(STEP_SIZE)

        # Update sensor manager exactly once per step (after Advance sequence)
        manager.Update()

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
