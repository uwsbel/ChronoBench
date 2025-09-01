import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the data path for Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (modified as per instructions)
initLoc = chrono.ChVector3d(0, -5, 0.4)  # Changed Y-coordinate to -5
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

# Simulation end time (corrected to use tend instead of hardcoded 30)
tend = 1000  # Original simulation end time

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Noise model for the sensors
noise_model = "NONE"

# Update rate in Hz for the sensors
update_rate = 10

# Image width and height for cameras
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408

# Sensor parameters
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

# Print vehicle information
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Set collision system type
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Add box object (1x1x1 at (0,0,0.5) with blue color)
box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_body.SetFixed(True)
box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().SetLengths(chrono.ChVector3d(1, 1, 1))
box_body.AddAsset(box_shape)
box_color = chrono.ChColorAsset()
box_color.SetColor(chrono.ChColor(0, 0, 1))  # Blue
box_body.AddAsset(box_color)
gator.GetSystem().Add(box_body)

# Add cylinder object (radius 0.5, height 1 at (0,0,1.5) with blue color)
cyl_body = chrono.ChBody()
cyl_body.SetPos(chrono.ChVector3d(0, 0, 1.5))
cyl_body.SetFixed(True)
cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().rad = 0.5
cyl_shape.GetCylinderGeometry().height = 1.0
cyl_body.AddAsset(cyl_shape)
cyl_color = chrono.ChColorAsset()
cyl_color.SetColor(chrono.ChColor(0, 0, 1))  # Blue
cyl_body.AddAsset(cyl_color)
gator.GetSystem().Add(cyl_body)

# Create the interactive driver system
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Create a sensor manager
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Create camera sensor
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# Create lidar sensor
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.ChQuaterniond(1, 0, 0, 0)
)

lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset,
    800,
    300,
    2 * chrono.CH_PI,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    100.0
)
lidar.SetName("3D Lidar")
lidar.SetBeamShape(sens.ChLidarSensor.BeamShape_RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.ChLidarSensor.ReturnMode_STRONGEST_RETURN)
lidar.SetLag(lag)
lidar.SetCollectionWindow(exposure_time)

# Add lidar filters
lidar.PushFilter(sens.ChFilterDIA())  # Depth, Intensity, Azimuth
lidar.PushFilter(sens.ChFilterXYZI())  # XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Viewer"))  # Visualization
manager.AddSensor(lidar)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = tend  # Corrected to use tend instead of hardcoded 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    # Set driver inputs (as per instructions)
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    # Collect output data from modules
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # Spin in place for real time
    realtime_timer.Spin(step_size)