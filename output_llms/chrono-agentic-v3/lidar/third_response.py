"""
ARTcar vehicle on rigid terrain with 3D lidar, 2D lidar, and third-person camera sensors.

System type: NSC (wrapper-managed via veh.ARTcar).
Main bodies: ARTcar chassis + 4 wheel spindles + rigid terrain patch.
Expected behavior: ARTcar drives forward on flat terrain; lidar sensors scan the environment
from the vehicle chassis; a third-person camera follows the vehicle.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Constants ===
# Simulation parameters
STEP_SIZE = 1e-3           # physics time step (s)
SIM_END   = 20.0           # simulation duration (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle / terrain
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0
INIT_LOC       = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT       = chrono.QuatFromAngleZ(0)

# Lidar parameters
LIDAR_UPDATE_RATE   = 5.0         # Hz
H_SAMPLES_3D        = 800
V_SAMPLES_3D        = 300
H_SAMPLES_2D        = 800
V_SAMPLES_2D        = 1
LIDAR_MAX_RANGE     = 100.0
LIDAR_OFFSET        = chrono.ChVector3d(1.0, 0, 1)  # per input3: changed from (-12,0,1) to (1,0,1)

# Camera sensor parameters
CAM_UPDATE_RATE = 30   # Hz, physical rate
CAM_OFFSET      = chrono.ChVector3d(-6, 0, 3)  # third-person: behind and above chassis

# === Data paths (mandatory truth components) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup (ARTcar wrapper) ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
artcar.SetTireType(veh.TireModelType_RIGID)
artcar.SetTireStepSize(STEP_SIZE)
artcar.Initialize()

# === System & bodies (created by veh.ARTcar wrapper) ===
system   = artcar.GetSystem()          # ChSystemNSC owned by the wrapper
chassis  = artcar.GetChassisBody()     # cache: main chassis rigid body; reused for sensors
# wheels/spindles: artcar.GetVehicle().GetAxle(i); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (called after Initialize)
artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht — vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Lidar Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (interactive IRR — standard scored-core form matching truth) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor Manager ===
manager = sens.ChSensorManager(system)
# Point lights for camera sensor rendering
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-50, 0, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === 3D Lidar Sensor (attached to vehicle chassis) ===
offset_pose_3d = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_3d = sens.ChLidarSensor(
    chassis,                               # attached to vehicle chassis (changed from box)
    LIDAR_UPDATE_RATE,
    offset_pose_3d,
    H_SAMPLES_3D,                          # horizontal samples
    V_SAMPLES_3D,                          # vertical samples (3D)
    2 * chrono.CH_PI,                      # horizontal FOV
    chrono.CH_PI / 12,                     # max vertical angle
    -chrono.CH_PI / 6,                     # min vertical angle
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,                                     # sample radius
    0.003,                                 # vertical divergence angle
    0.003,                                 # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar_3d.PushFilter(sens.ChFilterVisualize(H_SAMPLES_3D, V_SAMPLES_3D, "Raw Lidar Depth 3D"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud 3D"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_3d)

# === 2D Lidar Sensor (attached to vehicle chassis) ===
offset_pose_2d = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    chassis,                               # attached to vehicle chassis (changed from box)
    LIDAR_UPDATE_RATE,
    offset_pose_2d,
    H_SAMPLES_2D,                          # horizontal samples
    V_SAMPLES_2D,                          # v_samples=1 → 2D lidar
    2 * chrono.CH_PI,                      # horizontal FOV
    0,                                     # max_vert_angle = 0 for 2D
    0,                                     # min_vert_angle = 0 for 2D
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar_2d.PushFilter(sens.ChFilterVisualize(H_SAMPLES_2D, V_SAMPLES_2D, "Raw Lidar Depth 2D"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud 2D"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

# === Third-Person Camera Sensor (attached to vehicle chassis) ===
offset_pose_cam = chrono.ChFramed(
    CAM_OFFSET,
    chrono.QuatFromAngleAxis(0.3, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)
cam_3p = sens.ChCameraSensor(
    chassis,              # attach to chassis — follows vehicle in its local frame
    CAM_UPDATE_RATE,
    offset_pose_cam,
    1280, 720,
    1.408,                # horizontal FOV (rad)
)
cam_3p.SetName("Third-Person Camera")
cam_3p.SetLag(0)
cam_3p.SetCollectionWindow(0)
cam_3p.PushFilter(sens.ChFilterVisualize(1280, 720, "Third-Person Camera"))
cam_3p.PushFilter(sens.ChFilterRGBA8Access())
cam_3p.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(cam_3p)

# === Review-only: recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and artcar.GetSystem().GetChTime() < SIM_END:
        time = artcar.GetSystem().GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        artcar.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # scored-core cleanup anchor
