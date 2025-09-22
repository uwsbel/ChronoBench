```python
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

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
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

# ------------------
# Create the terrain
# ------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Create the interactive driver system
driver = veh.ChDriver(gator.GetVehicle())

# -----------------------
# Create a sensor manager
# -----------------------
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Create a box object
box_size = chrono.ChVector3d(1, 1, 1)
box_pos = chrono.ChVector3d(0, 0, 0.5)
box_color = chrono.ChColor(0, 0, 1)
box_texture = veh.GetDataFile("terrain/textures/tile4.jpg")
box = veh.ChBox(box_size, box_pos, box_color)
box.SetTexture(box_texture, 50, 50)
manager.AddSensor(box)

# Create a cylinder object
cylinder_radius = 0.5
cylinder_height = 1
cylinder_pos = chrono.ChVector3d(0, 0, 1.5)
cylinder_color = chrono.ChColor(0, 0, 1)
cylinder_texture = veh.GetDataFile("terrain/textures/tile4.jpg")
cylinder = veh.ChCylinder(cylinder_radius, cylinder_height, cylinder_pos, cylinder_color)
cylinder.SetTexture(cylinder_texture, 50, 50)
manager.AddSensor(cylinder)

# Create and add a Lidar sensor to the system
lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0.0, 0, 2))
lidar_samples_h = 800
lidar_samples_v = 300
lidar_fov_h = 2 * chrono.CH_PI
lidar_fov_v_min = -chrono.CH_PI / 12
lidar_fov_v_max = chrono.CH_PI / 12
lidar_range_max = 100.0
lidar_rectangular_beam_shape = True
lidar_sample_radius = 2
lidar_divergence_angle = 0.003
lidar_strongest_return_mode = True
lidar_filters = [sens.ChFilterDepth(image_width, image_height), sens.ChFilterIntensity(), sens.ChFilterXYZI(), sens.ChFilterVisualize(image_width, image_height, "Gator Lidar")]
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset_pose,
    lid