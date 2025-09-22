import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

# ------------------------------------------------------------------
# 1.  Configure Chrono data paths
# ------------------------------------------------------------------
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
print("Vehicle assets path:", chrono.GetChronoDataPath() + "vehicle/")

# ------------------------------------------------------------------
# 2.  Initial vehicle configuration
# ------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization types
chassis_vis_type    = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_NONE
tire_vis_type       = veh.VisualizationType_MESH

# Simulation parameters
step_size        = 1e-3
tire_step_size   = step_size
render_fps       = 50
render_step_size = 1.0 / render_fps

# Sensor parameters
noise_model   = "NONE"
update_rate   = 10      # Hz
image_width   = 1280
image_height  = 720
fov           = 1.408   # horizontal field of view
lag           = 0.0
exposure_time = 0.0

# ------------------------------------------------------------------
# 3.  Create and initialize the Gator vehicle
# ------------------------------------------------------------------
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

print("Vehicle mass:   ", gator.GetVehicle().GetMass())
print("Driveline type: ", gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     ", gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      ", gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print()

# Use the Bullet collision engine
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------------------------------------------------------
# 4.  Create a flat rigid terrain
# ------------------------------------------------------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# ------------------------------------------------------------------
# 5.  Add a box and a cylinder to the world
# ------------------------------------------------------------------
# Box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(box)

# Cylinder  (note: signature is radius, height, density)
cyl = chrono.ChBodyEasyCylinder(0.5, 1.0, 1000, True)
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))
cyl.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(cyl)

# ------------------------------------------------------------------
# 6.  Interactive driver (no GUI in this example)
# ------------------------------------------------------------------
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# ------------------------------------------------------------------
# 7.  Sensor manager & lighting
# ------------------------------------------------------------------
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity),
                            500.0)

# ------------------------------------------------------------------
# 8a.  Third‐person RGB camera
# ------------------------------------------------------------------
rgb_offset = chrono.ChFrameD(chrono.ChVector3d(-8.0, 0.0, 1.45),
                             chrono.Q_from_AngAxis(0.2, chrono.ChVector3d(0,1,0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    rgb_offset,
    image_width, image_height,
    fov
)
cam.SetName("Third Person RGB")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# ------------------------------------------------------------------
# 8b.  LIDAR sensor
# ------------------------------------------------------------------
lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.0, 0.0, 2.0),
                               chrono.Q_from_AngAxis(0.0, chrono.ChVector3d(0,1,0)))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset,
    800,      # horiz samples
    300,      # vert channels
    2*chrono.CH_PI, chrono.CH_PI/12, -chrono.CH_PI/6,
    100.0,    # max range
    sens.LidarBeamShape_RECTANGULAR,
    2,        # sample radius
    0.003,    # divergence horiz
    0.003,    # divergence vert
    sens.LidarReturnMode_STRONGEST_RETURN
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1.0/update_rate)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# ------------------------------------------------------------------
# 8c.  --- NEW --- Depth camera
# ------------------------------------------------------------------
depth_offset = chrono.ChFrameD(chrono.ChVector3d(-5.0, 0.0, 2.0),
                               chrono.Q_from_AngAxis(0.0, chrono.ChVector3d(0,1,0)))
depth_cam = sens.ChDepthCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    depth_offset,
    image_width, image_height,
    fov,
    30.0                # maximum depth = 30 m
)
depth_cam.SetName("Depth Camera")
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(1.0/update_rate)
# visualize depth map
depth_cam.PushFilter(sens.ChFilterVisualizeDepth(image_width, image_height, "Depth Map"))
manager.AddSensor(depth_cam)

# ------------------------------------------------------------------
# 9.  Main simulation loop
# ------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0.0
end_time = 30.0

print("\nStarting simulation loop...")
print("  time[s]   x[m]    y[m]    z[m]    yaw[rad]")

while time < end_time:
    time = gator.GetSystem().GetChTime()

    # simple steering & throttle
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    inputs = driver.GetInputs()

    # synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, inputs, terrain)
    manager.Update()

    # log vehicle pos & heading
    pos = gator.GetVehicle().GetChassisBody().GetPos()
    rot = gator.GetVehicle().GetChassisBody().GetRot()
    # compute yaw from quaternion (z‐axis heading)
    w, x, y, z = rot.e0, rot.e1, rot.e2, rot.e3
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    print(f"{time:8.3f}   {pos.x:7.3f} {pos.y:7.3f} {pos.z:7.3f}   {yaw:7.3f}")

    # advance all
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    realtime_timer.Spin(step_size)

print("Done.")