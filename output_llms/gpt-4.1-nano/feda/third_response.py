import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "/vehicle/")

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Track Point for chase camera
trackPoint = chrono.ChVector3D(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size (FPS 50)
render_step_size = 1.0 / 50

# Create the vehicle
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization options
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with grass texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.QUNIT),
    terrainLength, terrainWidth)

# Change terrain texture to grass
texture_path = "terrain/textures/grass.jpg"  # Assume this file exists in data folder
patch.SetTexture(veh.GetDataFile(texture_path), 200, 200)

# Set terrain color
patch.SetColor(chrono.ChColor(0.2, 0.8, 0.2))  # Greenish to resemble grass

terrain.Initialize()

# -------- Sensor System Setup --------
# Initialize the Irrlicht driver to create a sensor manager
# We will create a sensor manager to attach sensors later
sensor_manager = irr.ChSensorManager()

# Add point lights for scene illumination
light1 = irr.ChLightingPoint()
light1.SetPosition(0, 20, 0)
light1.SetIntensity(8.0)  # High intensity
sensor_manager.Add(light1)

light2 = irr.ChLightingPoint()
light2.SetPosition(20, 20, 20)
light2.SetIntensity(6.0)
sensor_manager.Add(light2)

# -------- Vehicle Irrlicht and Camera Sensor --------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()

# Attach the sensor manager to the visualization
vis.AttachSensorManager(sensor_manager)

# Add scene elements
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

# Attach vehicle to rendering
vis.AttachVehicle(vehicle.GetVehicle())

# -------- Camera Sensor (First-Person) --------
# Create camera sensor attached to the vehicle's chassis body
chassis_body = vehicle.GetVehicle().GetChassis()
camera_position = chrono.ChVectorD(0.3, 0, 1.0)  # Slightly in front of the chassis, at driver viewpoint

# Add camera sensor to sensor manager
camera_sensor = sensor_manager.AddSensor(chrono.ChSensor(
    chassis_body,  # attached to chassis body
    chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 1.0)),  # relative position
    30.0,  # Field of View in degrees
    1920,  # resolution X
    1080   # resolution Y
))
# Set camera type to perspective
camera_sensor.SetTypes(chrono.ChSensorType_CAMERA)

# -------- Visualization filter - for example, rendering the camera view
visual_filter = camera_sensor.GetDisplayCamera()

# -------- Driver Input Setup --------
driver = veh.ChInteractiveDriverIRR(vis)
# Set input deltas
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Calculate how many steps per render frame
render_steps = math.ceil(render_step_size / step_size)

# Timer for real-time synchronization
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation Loop
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified FPS
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        # Optional: visualize camera image if needed
        vis.EndScene()
        render_frame += 1

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update camera sensor (to get latest image frame)
    camera_sensor.Update()

    # Optionally, retrieve the image or display it here
    # image = camera_sensor.GetImage()
    # (Processing can be added here if needed)

    # Increment step
    step_number += 1

    # Spin to catch up for real-time display
    realtime_timer.Spin(step_size)