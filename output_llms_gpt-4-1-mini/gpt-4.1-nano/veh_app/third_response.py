import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the data path for Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization types
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation parameters
step_size = 1e-3
tire_step_size = step_size
tend = 1000
render_step_size = 1.0 / 50  # FPS = 50
noise_model = "NONE"
update_rate = 10
image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0
vis = True

# Create the vehicle
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

# Print vehicle info
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Collision system
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem_Type_BULLET)

# Terrain setup
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT), 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Create a box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(box)

# Create cylinder
cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxisY, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().AddBody(cylinder)

# Driver system
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Sensor manager
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# -----------------------
# Create existing sensors
# -----------------------

# Camera sensor
offset_pose_cam = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose_cam,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# Lidar sensor
offset_pose_lidar = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),     # Body lidar is attached to
    update_rate,               # Scanning rate
    offset_pose_lidar,         # Offset pose
    800,                       # Horizontal samples
    300,                       # Vertical channels
    2*chrono.CH_C_PI,           # Horizontal FOV
    chrono.CH_C_PI / 12,        # Max vertical FOV
    -chrono.CH_C_PI / 6,        # Min vertical FOV
    100.0,                      # Max distance
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1.0 / update_rate)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# -----------------------
# Add Depth Camera (New)
# -----------------------
depth_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
depth_cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    depth_offset_pose,
    1280,
    720,
    fov
)
depth_cam.SetName("Depth Camera")
// Apply visualization filter for depth maps
depth_cam.PushFilter(sens.ChFilterVisualizeDepthMap(1280, 720, "Depth Map"))
manager.AddSensor(depth_cam)

# Main simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30

while time < end_time:
    time = gator.GetSystem().GetChTime()

    # Set driver inputs (example)
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    # Update sensors
    manager.Update()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # Log vehicle state: position and heading
    chassis_pos = gator.GetChassisBody().GetPos()
    chassis_rot = gator.GetChassisBody().GetRot()
    heading = chassis_rot.GetY().GetZ()  # or compute heading from rot as needed
    print(f"Time: {time:.3f} | Position: ({chassis_pos.x:.2f}, {chassis_pos.y:.2f}, {chassis_pos.z:.2f}) | Heading: {math.degrees(heading):.2f} deg")

    # Optional: Sleep for real-time if necessary
    # realtime_timer.Spin(step_size)