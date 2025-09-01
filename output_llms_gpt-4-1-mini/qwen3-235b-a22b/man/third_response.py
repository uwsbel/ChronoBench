import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens  # Added sensor module
import math
import numpy as np  # Added numpy module


# Set data paths (corrected vehicle data path)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataFile('vehicle/'))  # Fixed data path

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50

# Create vehicle
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

# Changed texture to grass.jpg
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Add sensor manager and lidar sensor
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  # Attach to chassis
    10,                        # Update rate
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.QUNIT),  # Position
    360,                       # Horizontal samples
    1,                         # Vertical samples
    math.pi,                   # Horizontal FOV
    math.pi/12,                # Max vertical angle
    0                          # Min vertical angle
)
lidar.SetName("Lidar_Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.1)
lidar.PushFilter(sens.ChFilterDIAccess())  # Data access filter
sensor_manager.AddSensor(lidar)

# Add random boxes to the environment
num_boxes = 10
box_size = 1.0
for i in range(num_boxes):
    # Generate random positions
    x = np.random.uniform(-terrainLength/2, terrainLength/2)
    y = np.random.uniform(-terrainWidth/2, terrainWidth/2)
    z = box_size / 2  # Place on ground
    
    # Create box body
    box_body = chrono.ChBody()
    box_body.SetPos(chrono.ChVectorD(x, y, z))
    box_body.SetBodyFixed(True)
    box_body.SetCollide(True)
    
    # Collision shape
    box_body.GetCollisionModel().ClearModel()
    box_body.GetCollisionModel().AddBox(box_size/2, box_size/2, box_size/2)
    box_body.GetCollisionModel().BuildModel()
    
    # Visualization shape
    box_vis = chrono.ChBoxShape()
    box_vis.GetBoxGeometry().Size = chrono.ChVectorD(box_size/2, box_size/2, box_size/2)
    box_body.GetAssets().push_back(box_vis)
    
    # Add to system
    vehicle.GetSystem().Add(box_body)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update sensor manager
    sensor_manager.Update()  # Updated sensor manager within loop

    # Increment step counter
    step_number += 1
    
    # Maintain real-time
    realtime_timer.Spin(step_size)