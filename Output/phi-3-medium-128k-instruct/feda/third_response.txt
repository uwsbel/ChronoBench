import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Change terrain texture to grass
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Create the sensor manager
sensor_manager = chrono.ChSensorManager()

# Add point lights to the scene
light1 = chrono.ChLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 1000)
light2 = chrono.ChLight(chrono.ChVector3d(0, 10, 0), chrono.ChColor(1, 1, 1), 1000)
sensor_manager.AddLight(light1)
sensor_manager.AddLight(light2)

# Add camera sensor to the vehicle’s chassis body
camera_sensor = chrono.ChCameraSensor(vehicle.GetSystem(), 1280, 720, 60)
camera_sensor.SetPosition(chrono.ChVector3d(0, 0, 1.5))
camera_sensor.SetFieldOfView(90)
camera_sensor.SetResolution(1920, 1080)

# Add visualization filter to render the image and include it in the sensor manager
camera_filter = chrono.ChVisualizationFilter(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the sensor manager
sensor_manager = chrono.ChSensorManager()

# Add point lights to the scene
light1 = chrono.ChLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 1000)
light2 = chrono.ChLight(chrono.ChVector3d(0, 10, 0), chrono.ChColor(1, 1, 1), 1000)
sensor_manager.AddLight(light1)
sensor_manager.AddLight(light2)

# Add camera sensor to the vehicle’s chassis body
camera_sensor = chrono.ChCameraSensor(vehicle.GetSystem(), 1280, 720, 60)
camera_sensor.SetPosition(chrono.ChVector3d(0, 0, 1.5))
camera_sensor.SetFieldOfView(90)
camera_sensor.SetResolution(1920, 1080)

# Add visualization filter to render the image and include it in the sensor manager
camera_filter = chrono.ChVisualizationFilter(camera_sensor)
camera_filter.SetTexture(veh.GetDataFile("camera/textures/grass.jpg"), 200, 200)
sensor_manager.AddSensor(camera_filter)

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.AddSensorManager.AddSensor.AddLight(veh.AddLight(veh.AddLight(veh.AddLight.AddLight(veh.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight.AddLight