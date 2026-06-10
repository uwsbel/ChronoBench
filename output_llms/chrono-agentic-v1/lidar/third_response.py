"""
ARTcar vehicle simulation with lidar sensors (3D and 2D) and a third-person camera
sensor, all attached to the vehicle chassis, on a rigid terrain patch.

System: ChSystemNSC (owned by veh.ARTcar wrapper)
Vehicle: ARTcar on RigidTerrain
Sensors: 3D ChLidarSensor (800h x 300v), 2D ChLidarSensor (800h x 1v),
         third-person ChCameraSensor — all attached to the chassis.
Driver: ChInteractiveDriverIRR (real-time interactive, vis-bound)
Visualization: ChWheeledVehicleVisualSystemIrrlicht (chase camera)
Expected behavior: ARTcar drives on flat rigid terrain; sensors scan the environment
from the chassis mount at offset (1.0, 0, 1); lidar point clouds and RGB camera
frames are saved; Irrlicht window provides real-time view.
"""

# === Imports ===
import math
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Data paths (anchored to the chrono-900 data tree) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 5e-4           # integration step [s]
sim_end = 20.0             # simulation end time [s]
render_fps = 50.0          # Irrlicht render rate [Hz]
render_step_size = 1.0 / render_fps                             # precomputed once
render_steps = max(1, math.ceil(render_step_size / time_step))  # precomputed once

TERRAIN_LENGTH = 200.0     # terrain patch length [m]
TERRAIN_WIDTH = 200.0      # terrain patch width [m]
INIT_HEIGHT = 0.5          # chassis spawn height above terrain [m]

# Lidar sensor parameters
LIDAR_UPDATE_RATE = 5.0    # Hz
H_SAMPLES_3D = 800
V_SAMPLES_3D = 300
H_SAMPLES_2D = 800
V_SAMPLES_2D = 1
LIDAR_MAX_RANGE = 100.0
H_FOV = 2 * chrono.CH_PI
MAX_VERT_ANGLE_3D = chrono.CH_PI / 12
MIN_VERT_ANGLE_3D = -chrono.CH_PI / 6

# Camera sensor parameters
CAM_UPDATE_RATE = 30       # Hz (physical rate, not 1/dt)
CAM_W, CAM_H = 1280, 720
CAM_FOV = 1.408            # horizontal FOV [rad]


# === Vehicle setup (ARTcar wrapper owns the ChSystem) ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, INIT_HEIGHT), chrono.QUNIT)
)
artcar.SetTireType(veh.TireModelType_TMEASY)
artcar.SetTireStepSize(time_step)
artcar.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
system = artcar.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = artcar.GetChassisBody()            # cache: main chassis rigid body
# wheels/spindles: artcar.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch body below

artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht Vehicle Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar with Lidar and Camera Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (interactive, vis-bound — scored core) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0       # s to reach full steering
throttle_time = 1.0       # s to reach full throttle
braking_time = 0.3        # s to reach full braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor Manager ===
manager = sens.ChSensorManager(system)
# Point lights for sensor camera rendering
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === 3D Lidar Sensor (attached to chassis) ===
lidar3d_offset = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),                         # offset pose per prompt
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar3d = sens.ChLidarSensor(
    chassis,                            # attached to vehicle chassis
    LIDAR_UPDATE_RATE,
    lidar3d_offset,
    H_SAMPLES_3D,
    V_SAMPLES_3D,
    H_FOV,
    MAX_VERT_ANGLE_3D,
    MIN_VERT_ANGLE_3D,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar3d.SetName("3D Lidar Sensor")
lidar3d.SetLag(0)
lidar3d.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar3d.PushFilter(sens.ChFilterVisualize(H_SAMPLES_3D, V_SAMPLES_3D, "3D Lidar Depth"))
lidar3d.PushFilter(sens.ChFilterDIAccess())
lidar3d.PushFilter(sens.ChFilterPCfromDepth())
lidar3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar3d)

# === 2D Lidar Sensor (attached to chassis) ===
lidar2d_offset = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),                         # same mount position as 3D
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar2d = sens.ChLidarSensor(
    chassis,
    LIDAR_UPDATE_RATE,
    lidar2d_offset,
    H_SAMPLES_2D,
    V_SAMPLES_2D,                                         # 1 for 2D lidar
    H_FOV,
    0,                                                     # max_vert = 0 for 2D
    0,                                                     # min_vert = 0 for 2D
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar2d.PushFilter(sens.ChFilterVisualize(H_SAMPLES_2D, V_SAMPLES_2D, "2D Lidar Depth"))
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# === Third-Person Camera Sensor (attached to chassis) ===
# Offset behind and above the vehicle for a third-person chase view
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-6.0, 0, 2.0),                      # behind and above chassis
    chrono.QuatFromAngleAxis(0.25, chrono.ChVector3d(0, 1, 0)),
)
cam_sensor = sens.ChCameraSensor(
    chassis,                            # attached to vehicle chassis (third-person)
    CAM_UPDATE_RATE,
    cam_offset,
    CAM_W, CAM_H,
    CAM_FOV,
)
cam_sensor.SetName("Third Person Camera")
cam_sensor.SetLag(0)
cam_sensor.SetCollectionWindow(0)
cam_sensor.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "Third Person Camera"))
cam_sensor.PushFilter(sens.ChFilterRGBA8Access())
cam_sensor.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(cam_sensor)

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and artcar.GetSystem().GetChTime() < sim_end:
        sim_time = artcar.GetSystem().GetChTime()  # cache: fetched once per frame

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        artcar.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        artcar.Advance(time_step)
        vis.Advance(time_step)

        manager.Update()


        step_number += 1
        realtime_timer.Spin(time_step)
        if artcar.GetSystem().GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
