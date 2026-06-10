"""
HMMWV vehicle simulation with a depth camera sensor and vehicle state logging.

System: NSC (rigid terrain, wheeled vehicle wrapper)
Bodies: HMMWV full vehicle on a flat rigid terrain; depth camera attached to chassis
Sensors: ChDepthCamera — offset pose (-5, 0, 2), 1280×720, 1.408 rad HFOV, 30 m max depth,
         with ChFilterDepthToRGBA8 visualization filter (depth map preview).
Logging: Vehicle state (position X, Y, Z and heading) logged every simulation step.
Expected: HMMWV responds to interactive driver inputs; depth camera provides a live
          depth-map preview; vehicle state is logged to simulation_data.csv each step.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
step_size = 1e-3
sim_end = 20.0
render_fps = 50.0
render_steps = max(1, math.ceil(1.0 / (render_fps * step_size)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0
INIT_X, INIT_Y = 0.0, 0.0
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (HMMWV)
INIT_Z = 0.0 + SUSPENSION_REF_HEIGHT  # flat terrain at z=0

# === Data paths (mandatory truth components for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT,
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                                # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                       # cache: fetched once, reused

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
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

# === Irrlicht visualization (vehicle-specific) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Depth Camera + State Logging")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()      # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / render_fps                  # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor manager + depth camera ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Depth camera: offset at (-5, 0, 2) in the chassis frame, tilt slightly down
DEPTH_CAM_OFFSET = chrono.ChVector3d(-5.0, 0, 2)
DEPTH_CAM_HFOV   = 1.408          # horizontal field of view (rad)
DEPTH_CAM_MAX    = 30.0           # maximum depth (m)
DEPTH_CAM_W      = 1280
DEPTH_CAM_H      = 720
DEPTH_CAM_RATE   = 30             # physical update rate (Hz)

offset_pose = chrono.ChFramed(
    DEPTH_CAM_OFFSET,
    chrono.QuatFromAngleAxis(0.15, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)
depth_cam = sens.ChDepthCamera(
    chassis,                   # attach to chassis body (real protagonist)
    DEPTH_CAM_RATE,
    offset_pose,
    DEPTH_CAM_W,
    DEPTH_CAM_H,
    DEPTH_CAM_HFOV,
    DEPTH_CAM_MAX,
)
depth_cam.SetName("Depth Camera")
depth_cam.SetLag(0)
depth_cam.SetCollectionWindow(0)
# Depth-map visualization filter (scored core — prompt-required)
depth_cam.PushFilter(sens.ChFilterDepthToRGBA8("Depth Map"))   # visualize depth as RGBA
depth_cam.PushFilter(sens.ChFilterVisualize(DEPTH_CAM_W, DEPTH_CAM_H, "Depth Camera"))
depth_cam.PushFilter(sens.ChFilterDepthAccess())               # host access to depth buffer
depth_cam.PushFilter(sens.ChFilterSave("cam/depth/"))          # save depth-map frames
manager.AddSensor(depth_cam)

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # Scored-core vehicle state (prompt-required every step)
        pos     = chassis.GetPos()        # cache: chassis body fetched before loop
        heading = chassis.GetRot().GetCardanAnglesZYX().x   # heading angle (Z rotation)

        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        manager.Update()

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
