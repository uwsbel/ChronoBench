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
initLoc = chrono.ChVectorD(0, -5, 0.4)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Noise model for the sensors
noise_model = "NONE"  # No noise model

# Update rate in Hz for the sensors
update_rate = 10

# Image width and height for cameras
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure time (in seconds) for each image
exposure_time = 0

# View camera images
vis = True

# Create the vehicle, set parameters, and initialize
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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

# ------------------
# Create the terrain
# ------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# ------------------
# Add Box Object
# ------------------
box_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
box_body.SetPos(chrono.ChVectorD(0, 0, 0.5))
box_body.SetBodyFixed(True)
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
gator.GetSystem().Add(box_body)

# ------------------
# Add Cylinder Object
# ------------------
cylinder_body = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
cylinder_body.SetPos(chrono.ChVectorD(0, 0, 1.5))
cylinder_body.SetBodyFixed(True)
cylinder_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
gator.GetSystem().Add(cylinder_body)

# Create the interactive driver system
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# -----------------------
# Create a sensor manager
# -----------------------
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Create two cameras and add them to the sensor manager
offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8.0, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")
# Renders the image at current point in the filter graph
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# -----------------------
# Add Lidar Sensor
# -----------------------
lidar_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.0, 0, 2))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset_pose,
    800,  # horizontal samples
    300,  # vertical channels
    2 * chrono.CH_C_PI,  # horizontal field of view
    chrono.CH_C_PI / 12,  # maximum vertical field of view
    -chrono.CH_C_PI / 6,  # minimum vertical field of view
    100.0  # maximum range
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(exposure_time)
lidar.PushFilter(sens.ChFilterIntensity())
lidar.PushFilter(sens.ChFilterXYZI())
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Visualization"))
manager.AddSensor(lidar)

# ---------------
# Simulation loop
# ---------------

realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    # set driver inputs
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)