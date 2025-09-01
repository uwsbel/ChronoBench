import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np
import pychrono.sensor as sensor
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

# Initialize vehicle visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Update collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and initialize the terrain
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

# Create sensor manager and lidar sensor
sensor_manager = sensor.ChSensorManager()
lidar = veh.LidarSensor(sensor_manager, veh.LidarSensor.Type_Disk, 10, 10, 10, 1000)
lidar.SetPosition(chrono.ChVector3d(0, 0, 5))
lidar.Initialize()
sensor_manager.AddSensor(lidar)

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering and throttle inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

def CreateRandomBoxes(terrain, sensor_manager):
    # Generate random boxes
    for _ in range(10):
        size = chrono.ChVector3d(
            np.random.uniform(0.5, 2.5),
            np.random.uniform(0.5, 2.5),
            np.random.uniform(0.5, 1.5)
        )
        position = chrono.ChVector3d(
            np.random.uniform(-10, 10),
            np.random.uniform(-10, 10),
            np.random.uniform(0, 2)
        )
        color = chrono.ChColor(
            np.random.uniform(0.5, 1),
            np.random.uniform(0.2, 0.8),
            np.random.uniform(0.2, 0.8)
        )
        material = chrono.ChContactMaterialNSC()
        material.SetFriction(0.6)
        material.SetRestitution(0.3)
        box = terrain.AddBox(
            size,
            position,
            color,
            material
        )
        box.SetName(f"box_{_}")

def UpdateSensors():
    sensor_manager.Update()

# Add random boxes to the simulation
CreateRandomBoxes(terrain, sensor_manager)

# Initialize simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation for one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    
    # Increment frame number
    step_number += 1
    
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)
    
    # Update sensors
    UpdateSensors()