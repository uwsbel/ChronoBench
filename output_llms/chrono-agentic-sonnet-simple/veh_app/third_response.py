import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle initial pose
initLoc = chrono.ChVector3d(0, -5, 0.4)                              # starting position (Y=-5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity rotation

# Simulation parameters
step_size = 1e-3                                                      # physics time step (s)
tire_step_size = step_size
end_time = 30                                                         # simulation end (s)
render_step_size = 1.0 / 50                                           # render cadence = 50 FPS

# Sensor parameters
update_rate = 10                                                      # sensor update rate (Hz)
image_width = 1280                                                    # camera image width (px)
image_height = 720                                                    # camera image height (px)
fov = 1.408                                                           # horizontal FOV (rad)
lag = 0                                                               # sensor lag (s)
exposure_time = 0                                                     # exposure/collection window (s)

# Create Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
gator.SetChassisFixed(False)                                          # MANDATORY: allow chassis to move
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY for rigid terrain
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

# Visualization types for vehicle parts (after Initialize)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# Print vehicle diagnostics
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Set collision system after Initialize (required for contact scenes)
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Rigid terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()                            # NSC matches vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)        # flat 50x50 m patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Box scene object
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))                             # placed at origin, z=0.5
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(box)

# Cylinder scene object
cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))                        # stacked above box
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(cylinder)

# Driver: scripted (matching truth — SetSteering/SetThrottle in loop)
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Sensor manager with point light
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(                                         # single overhead point light
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# RGB camera attached to chassis (third-person view)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),       # slight downward tilt
)
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),                                          # attached to chassis
    update_rate,                                                     # 10 Hz physical update rate
    offset_pose,
    image_width,
    image_height,
    fov,
)
cam.SetName("Third Person POV")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                           # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                        # SCORED CORE: save color frames
manager.AddSensor(cam)

# Lidar sensor attached to chassis
offset_pose_lidar = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,                                                     # 10 Hz update rate
    offset_pose_lidar,
    800,                                                             # 800 horizontal samples
    300,                                                             # 300 vertical channels
    2 * chrono.CH_PI,                                               # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                              # max vertical angle
    -chrono.CH_PI / 6,                                              # min vertical angle
    100.0,                                                          # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                                              # sample radius
    0.003,                                                          # divergence angle
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1.0 / update_rate)                         # lidar: collection = 1/update_rate
lidar.PushFilter(sens.ChFilterDIAccess())                            # depth+intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())                         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # XYZI host access
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# Depth camera — offset pose as specified
offset_pose_depthcam = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),                                   # (-5, 0, 2) offset on chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
depth_cam = sens.ChDepthCamera(
    gator.GetChassisBody(),                                          # attached to chassis
    update_rate,                                                     # 10 Hz physical update rate
    offset_pose_depthcam,
    image_width,                                                     # 1280 px width
    image_height,                                                    # 720 px height
    fov,                                                             # 1.408 rad HFOV
)
depth_cam.SetName("Depth Camera Sensor")
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(exposure_time)
depth_cam.SetMaxDepth(30)                                            # max depth = 30 m (as specified)
depth_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Depth Map"))  # visualization
manager.AddSensor(depth_cam)


# Simulation loop — per step: log state, set scripted inputs, synchronize, advance
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
while time < end_time:
    time = gator.GetSystem().GetChTime()

    # Log vehicle state at every step (position + heading)
    vehicle_pos = gator.GetVehicle().GetChassis().GetPos()
    vehicle_heading = gator.GetVehicle().GetChassis().GetRot().GetCardanAnglesZYX().z
    print("Time: ", time, " X: ", vehicle_pos.x, " Y: ", vehicle_pos.y, " Z: ", vehicle_pos.z, " Heading: ", vehicle_heading)


    # Scripted driver inputs (truth: steer=0.5, throttle=0.2)
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    # Update sensors
    manager.Update()

    # Advance modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    realtime_timer.Spin(step_size)                                   # real-time pacing
