import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random
import numpy as np

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
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

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                            0,     # Bekker Kc
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (Pa)
                            30,    # Mohr friction limit (degrees)
                            0.01,  # Janosi shear coefficient (m)
                            2e8,   # Elastic stiffness (Pa/m), before plastic yield
                            3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes to the scene
def add_random_boxes(system, num_boxes=10):
    """Add randomly positioned boxes ensuring they don't overlap with vehicle initial position"""
    boxes = []
    vehicle_pos = initLoc
    min_distance_from_vehicle = 5.0  # Minimum distance from vehicle
    
    for i in range(num_boxes):
        # Generate random position ensuring it's not too close to vehicle
        while True:
            x = random.uniform(-15, 15)
            y = random.uniform(-15, 15)
            z = random.uniform(0.5, 2.0)
            
            box_pos = chrono.ChVector3d(x, y, z)
            distance = (box_pos - vehicle_pos).Length()
            
            if distance > min_distance_from_vehicle:
                break
        
        # Create box with random dimensions
        box_size = chrono.ChVector3d(
            random.uniform(0.5, 2.0),
            random.uniform(0.5, 2.0),
            random.uniform(0.5, 2.0)
        )
        
        # Create the box body
        box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000)
        box.SetPos(box_pos)
        box.SetBodyFixed(False)
        
        # Set random color for visualization
        box_color = chrono.ChColor(random.random(), random.random(), random.random())
        box.GetVisualShape(0).SetColor(box_color)
        
        # Add to system
        system.Add(box)
        boxes.append(box)
    
    return boxes

# Add boxes to the scene
boxes = add_random_boxes(vehicle.GetSystem())

# Create sensor manager
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights at various positions
light_positions = [
    chrono.ChVector3d(0, 0, 10),
    chrono.ChVector3d(10, 10, 8),
    chrono.ChVector3d(-10, 10, 8),
    chrono.ChVector3d(10, -10, 8),
    chrono.ChVector3d(-10, -10, 8)
]

for i, pos in enumerate(light_positions):
    point_light = sens.ChPointLight(pos, chrono.ChColor(1, 1, 1), 500.0)
    sensor_manager.scene.AddPointLight(point_light)

# Create camera sensor attached to vehicle chassis
camera_offset = chrono.ChVector3d(0, 0, 2.0)  # Offset from chassis center
camera_sensor = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  # Attach to chassis
    30.0,  # Update rate in Hz
    camera_offset,  # Offset position
    chrono.ChQuaterniond(1, 0, 0, 0),  # Orientation
    1280,  # Image width
    720,   # Image height
    chrono.CH_C_PI / 3,  # Field of view (60 degrees)
    1  # Super sampling factor
)

# Configure camera settings
camera_sensor.SetName("VehicleCamera")
camera_sensor.SetLag(0.0)
camera_sensor.SetCollectionWindow(0.0)

# Add filters to the camera
# Add visualization filter to see camera feed
camera_sensor.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))

# Add RGBA8 buffer filter
camera_sensor.PushFilter(sens.ChFilterRGBA8Access())

# Add camera to sensor manager
sensor_manager.AddSensor(camera_sensor)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors and Objects')
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

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print(f"Added {len(boxes)} random boxes to the scene")

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Update sensor manager
    sensor_manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)