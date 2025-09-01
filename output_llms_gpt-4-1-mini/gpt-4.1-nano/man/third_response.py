import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np

# Import sensor modules (assuming such exists; if not, would need to install/validate)
import pychrono.sensor as sensor

# Set data paths
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

# Rigid terrain parameters
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

# Time interval between render frames
render_step_size = 1.0 / 50  # 50 FPS

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with new texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Changed texture from "tile4.jpg" to "grass.jpg"
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Initialize sensor manager
sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())

# Create and add a lidar sensor
# Define sensor position relative to vehicle
lidar_position = chrono.ChVector(0, 0, 2)  # Adjust as needed
lidar_orientation = chrono.QUNIT  # No rotation

# Create lidar sensor with parameters
lidar = sensor.ChLidarSensor(
    vehicle.GetVehicle(), 
    25.0,  # max range in meters
    0.25,  # angular resolution in radians
    360.0, # horizontal scan angles in degrees
    lidar_position, 
    lidar_orientation)

sensor_manager.AddSensor(lidar)

# Create some random boxes in the environment
# Number of boxes
num_boxes = 10
box_size = 1.0  # size of each box

for i in range(num_boxes):
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 
                               density=10, 
                               isDynamic=True)
    # Random position within terrain bounds
    x = np.random.uniform(-terrainLength/2 + box_size, terrainLength/2 - box_size)
    y = np.random.uniform(-terrainWidth/2 + box_size, terrainWidth/2 - box_size)
    z = terrainHeight + box_size / 2  # On top of terrain
    box.SetPos(chrono.ChVectorD(x, y, z))
    # Random color for visualization (optional)
    color = chrono.ChColor(np.random.random(), np.random.random(), np.random.random())
    box.SetColor(color)

# Initialize driver system
driver = veh.ChInteractiveDriverIRR(vis)
# Set control response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Calculate the number of steps between renders
render_steps = math.ceil(render_step_size / step_size)

# Initialize real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render at specified frames
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update sensor manager and get latest sensor data
    sensor_manager.Update()
    # Optionally, retrieve point cloud data or lidar data here
    # lidar_points = lidar.GetPointCloud() # if such method exists

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step counter
    step_number += 1

    # Ensure real-time keeping
    realtime_timer.Spin(step_size)