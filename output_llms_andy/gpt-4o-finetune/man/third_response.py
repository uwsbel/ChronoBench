import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np


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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

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

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Create a sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Add a lidar sensor
lidar_offset = chrono.ChVector3d(0, 0, 2.3)
lidar_theta = chrono.Q_FROM_AZ_EL(0, -0.5)
lidar_pose = chrono.ChFramed(lidar_offset, lidar_theta)
min_distance = 0.1
max_distance = 100
update_rate = 5
lag = 0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * np.pi
vertical_fov = np.pi / 6
max_grab_distance = 100
noise_model = sens.ChLidarNoiseModelNone()
return_mode = sens.ChLidarReturnMode_STRONGEST_RETURN

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),           # Body lidar is attached to
    update_rate,            # Scanning rate in Hz
    lag,                    # Time lag between observation and  data, s
    lidar_pose,             # Offset pose of the lidar
    horizontal_samples,     # Number of horizontal samples
    vertical_samples,       # Number of vertical channels
    max_grab_distance,      # Max distance that a ray can "grab" an object
    horizontal_fov,         # Horizontal field of view
    vertical_fov,           # Vertical field of view
    min_distance,           # Minimum distance for a ray to be considered valid
    max_distance,           # Maximum distance for a ray
    return_mode,            # Return mode for the lidar
    noise_model             # Noise model for the lidar
)
lidar.SetName("Lidar Sensor")
lidar.SetDescription("Lidar sensor mounted on the vehicle")
lidar.SetLag(lag)
lidar.SetUpdateRate(update_rate)
lidar.SetHorizontalSampleNum(horizontal_samples)
lidar.SetVerticalSampleNum(vertical_samples)
lidar.SetMaxHorizontalFOV(horizontal_fov)
lidar.SetMaxVerticalFOV(vertical_fov)
lidar.SetCollectionWindow(1.0/update_rate)
lidar.SetMinDistance(min_distance)
lidar.SetMaxDistance(max_distance)
lidar.SetBeamDivergence(0.0, 0.0)
lidar.SetPhaseFunction(sens.Ch_LAMBERT)
lidar.SetReturnMode(return_mode)
lidar.SetNoiseModel(noise_model)
lidar.SetVerticalRayArrangement(sens.ChVerticalSampleMode_CONSTANT, 0.0, 0)

assert manager.AddSensor(lidar)

        
# Create a list to store body pointers for the boxes
body_list = []
num_boxes = 10  # Number of boxes to create
box_half_dims = chrono.ChVector3d(3, 3, 3)  # Half dimensions of each box
box_mass = 100  # Mass of each box
box_inertia = chrono.ChVector3d(10, 10, 10)  # Inertia tensor of each box

# Create and add boxes to the simulation
for i in range(num_boxes):
    # Calculate the position of each box along the x-axis
    box_x = 3 * i

    # Create a box body
    box_body = chrono.ChBody()
    box_body.SetMass(box_mass)
    box_body.SetInertiaXX(box_inertia)
    box_body.SetFixed(False)  # Make the box dynamic (not fixed)
    box_body.SetPos(chrono.ChVector3d(box_x, 0, 0))  # Set the position of the box

    # Add visualization for the box
    box_shape = chrono.ChVisualShapeBox(2 * box_half_dims.x, 2 * box_half_dims.y, 2 * box_half_dims.z)
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.AddVisualShape(box_shape)

    # Add the box to the list
    body_list.append(box_body)

    # Add the box to the physical system
    vehicle.GetSystem().Add(box_body)
# Initialize sensor manager
manager.Initialize()
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)