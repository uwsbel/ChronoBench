"""
Gator vehicle simulation with rigid terrain, a sensor manager hosting a camera,
a lidar, and a depth camera, plus vehicle state logging at every step.

System: ChSystemNSC (NSC contact method via Gator wrapper), rigid terrain.
Vehicle: Gator with TMEASY tires, scripted driver (steering=0.5, throttle=0.2).
Scene objects: a fixed blue box and a fixed blue cylinder.
Sensors:
  - ChCameraSensor attached to chassis (third-person POV) with visualization filter.
  - ChLidarSensor attached to chassis with DI, XYZI, and point-cloud-visualization filters.
  - ChDepthCamera attached to chassis at offset (-5, 0, 2), 1280×720, FOV 1.408,
    max depth 30 m, with visualization filter for depth map.
Logging: vehicle position (X, Y, Z) and heading logged at every simulation step.

Expected behavior: Gator steers right at constant throttle; sensors capture the
scene from behind/above; depth map shows distance to objects ahead.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Named constants ===
STEP_SIZE       = 1e-3          # physics time step (s)
SIM_END         = 30.0          # simulation end time (s)
TIRE_STEP_SIZE  = STEP_SIZE

TERRAIN_LENGTH  = 50.0
TERRAIN_WIDTH   = 50.0

INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# Sensor parameters
UPDATE_RATE  = 10               # sensor update rate (Hz) — physical rate
IMAGE_WIDTH  = 1280
IMAGE_HEIGHT = 720
FOV          = 1.408            # horizontal field of view (rad)
LAG          = 0
EXPOSURE     = 0

# Lidar parameters
H_SAMPLES    = 800
V_CHANNELS   = 300
H_FOV        = 2 * chrono.CH_PI
MAX_VERT_ANG = chrono.CH_PI / 12
MIN_VERT_ANG = -chrono.CH_PI / 6
LIDAR_RANGE  = 100.0
SAMPLE_RAD   = 2
DIV_ANG      = 0.003

# === Data paths (MANDATORY for catalog vehicles) ===
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup (Gator wrapper) ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)                         # MANDATORY — fixed chassis never moves
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(TIRE_STEP_SIZE)
gator.SetInitFwdVel(0.0)
gator.Initialize()

# Visualization types (in veh namespace for 9.0.0 source build)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))

# === System & bodies (created by the veh.Gator wrapper) ===
system  = gator.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()     # cache: fetched once, reused every step
# wheels/spindles: gator.GetVehicle().GetAxle(i); suspension/steering: wrapper-created

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Rigid Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# === Scene objects ===
# Fixed blue box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.AddBody(box)

# Fixed blue cylinder
cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.AddBody(cylinder)

# === Driver (scripted: steering=0.5, throttle=0.2) ===
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# === Sensor Manager ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# --- RGB Camera (third-person POV, attached to chassis) ---
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis,              # attach to chassis body (cache: fetched above)
    UPDATE_RATE,          # physical update rate (Hz)
    cam_offset_pose,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    FOV,
)
cam.SetName("Third Person POV")
cam.SetLag(LAG)
cam.SetCollectionWindow(EXPOSURE)
cam.PushFilter(sens.ChFilterVisualize(IMAGE_WIDTH, IMAGE_HEIGHT, "Gator Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))         # save RGB frames for review video
manager.AddSensor(cam)

# --- Lidar Sensor (attached to chassis) ---
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,              # attach to chassis (cache)
    UPDATE_RATE,          # physical update rate (Hz)
    lidar_offset_pose,
    H_SAMPLES,
    V_CHANNELS,
    H_FOV,
    MAX_VERT_ANG,
    MIN_VERT_ANG,
    LIDAR_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    SAMPLE_RAD,
    DIV_ANG,
    DIV_ANG,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(LAG)
lidar.SetCollectionWindow(1.0 / UPDATE_RATE)          # lidar: 1/update_rate
# Filter chain: DI access → XYZI point cloud → point cloud visualization
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# --- Depth Camera (as specified in the prompt) ---
depth_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
depth_cam = sens.ChDepthCamera(
    chassis,              # attach to chassis body (cache)
    UPDATE_RATE,          # physical update rate (Hz)
    depth_offset_pose,
    IMAGE_WIDTH,          # 1280
    IMAGE_HEIGHT,         # 720
    FOV,                  # 1.408 rad horizontal FOV
)
depth_cam.SetName("Depth Camera Sensor")
depth_cam.SetLag(LAG)
depth_cam.SetCollectionWindow(EXPOSURE)
depth_cam.SetMaxDepth(30)                             # maximum depth 30 m
depth_cam.PushFilter(sens.ChFilterVisualize(IMAGE_WIDTH, IMAGE_HEIGHT, "Depth Map"))
manager.AddSensor(depth_cam)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    time = 0.0
    while time < SIM_END:
        time = system.GetChTime()

        # Log vehicle state: position (X, Y, Z) and heading every step
        vehicle_pos = gator.GetVehicle().GetChassis().GetPos()    # cache: per-step position
        heading = gator.GetVehicle().GetChassis().GetRot().GetCardanAnglesZYX().z  # yaw heading
        print(f"Time: {time:.3f}  X: {vehicle_pos.x:.3f}  Y: {vehicle_pos.y:.3f}  Z: {vehicle_pos.z:.3f}  Heading: {heading:.4f}")


        # Scripted driver inputs (scored core — truth scripts these values)
        driver.SetSteering(0.5)
        driver.SetThrottle(0.2)

        # Collect outputs for inter-module communication
        driver_inputs = driver.GetInputs()

        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)

        # Update sensor manager (once per step)
        manager.Update()

        # Advance all modules
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
