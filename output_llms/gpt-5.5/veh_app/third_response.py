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

# Visualization type for vehicle parts
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50

# Noise model for the sensors
noise_model = "NONE"

# Update rate in Hz for the sensors
update_rate = 10

# Image width and height for cameras
image_width = 1280
image_height = 720

# Camera horizontal field of view
fov = 1.408

# Depth camera maximum depth
max_depth = 30.0

# Sensor lag and exposure time
lag = 0
exposure_time = 0

# View camera images
vis = True


# ---------------------------------------------------------------------
# Helper functions for logging vehicle state
# ---------------------------------------------------------------------
def vec_component(v, name):
    """Return a vector component, compatible with both property and method APIs."""
    value = getattr(v, name)
    return float(value() if callable(value) else value)


def get_heading(chassis_body):
    """
    Compute chassis heading from the chassis forward direction projected
    onto the global XY plane.
    """
    try:
        fwd = chassis_body.TransformDirectionLocalToParent(chrono.ChVector3d(1, 0, 0))
    except AttributeError:
        fwd = chassis_body.GetRot().Rotate(chrono.ChVector3d(1, 0, 0))

    return math.atan2(vec_component(fwd, "y"), vec_component(fwd, "x"))


# ---------------------------------------------------------------------
# Create the vehicle
# ---------------------------------------------------------------------
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

# Print vehicle information
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Set collision system type
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


# ---------------------------------------------------------------------
# Create the terrain
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(gator.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)

terrain.Initialize()


# ---------------------------------------------------------------------
# Create fixed obstacle bodies
# ---------------------------------------------------------------------

# Create a box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(box)

# Create a cylinder
cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(cylinder)


# ---------------------------------------------------------------------
# Create the driver system
# ---------------------------------------------------------------------
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()


# ---------------------------------------------------------------------
# Create a sensor manager
# ---------------------------------------------------------------------
manager = sens.ChSensorManager(gator.GetSystem())

intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0
)


# ---------------------------------------------------------------------
# RGB camera
# ---------------------------------------------------------------------
camera_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    camera_offset_pose,
    image_width,
    image_height,
    fov
)

cam.SetName("Third Person POV")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)

# Visualize RGB camera image
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))

manager.AddSensor(cam)


# ---------------------------------------------------------------------
# Added Depth Camera
# ---------------------------------------------------------------------
depth_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),
    chrono.ChQuaterniond(1, 0, 0, 0)
)

depth_cam = sens.ChDepthCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    depth_offset_pose,
    image_width,
    image_height,
    fov,
    max_depth
)

depth_cam.SetName("Depth Camera")
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(exposure_time)

# Visualization filter for depth map
depth_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Depth Camera - Depth Map"))

manager.AddSensor(depth_cam)


# ---------------------------------------------------------------------
# Lidar sensor
# ---------------------------------------------------------------------
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),              # Body lidar is attached to
    update_rate,                         # Scanning rate in Hz
    lidar_offset_pose,                   # Offset pose
    800,                                 # Number of horizontal samples
    300,                                 # Number of vertical channels
    2 * chrono.CH_PI,                    # Horizontal field of view
    chrono.CH_PI / 12,                   # Maximum vertical angle
    -chrono.CH_PI / 6,                   # Minimum vertical angle
    100.0,                               # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,     # Shape of the lidar beam
    2,                                   # Sample radius
    0.003,                               # Horizontal divergence angle
    0.003,                               # Vertical divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN
)

lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1 / update_rate)

# Lidar filters
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

manager.AddSensor(lidar)


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()

time = 0
end_time = 30

while time < end_time:
    time = gator.GetSystem().GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()
    driver_inputs.m_steering = 0.5
    driver_inputs.m_throttle = 0.2
    driver_inputs.m_braking = 0.0

    # Log vehicle state at every simulation step
    chassis_body = gator.GetChassisBody()
    pos = chassis_body.GetPos()
    heading = get_heading(chassis_body)

    print(
        f"Time: {time:.4f} s | "
        f"Position: X={vec_component(pos, 'x'):.4f}, "
        f"Y={vec_component(pos, 'y'):.4f}, "
        f"Z={vec_component(pos, 'z'):.4f} | "
        f"Heading: {heading:.6f} rad"
    )

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    # Update sensors
    manager.Update()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # Maintain real-time execution
    realtime_timer.Spin(step_size)