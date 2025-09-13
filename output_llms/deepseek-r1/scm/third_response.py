import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens  # Added for sensor system
import math
import random  # Added for random box placement

# Initialize Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model
tire_model = veh.TireModelType_RIGID

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point tracked by camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and configure HMMWV vehicle
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

# Create deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6,   # Bekker Kphi
    0,     # Bekker Kc
    1.1,   # Bekker n exponent
    0,     # Mohr cohesive limit (Pa)
    30,    # Mohr friction limit (degrees)
    0.01,  # Janosi shear coefficient (m)
    2e8,   # Elastic stiffness (Pa/m)
    3e4    # Damping (Pa s/m)
)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, 0.02)

# ========================================================================
# ADD RANDOM BOXES TO THE SCENE
# ========================================================================
num_boxes = 15
box_size = 1.0  # Box size (cubic)
safe_radius = 3.0  # Minimum distance from vehicle start position

for _ in range(num_boxes):
    while True:
        # Generate random position within terrain bounds
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        pos = chrono.ChVector3d(x, y, terrainHeight + box_size/2)
        
        # Ensure box isn't near vehicle start position
        if (pos - initLoc).Length() > safe_radius:
            break
    
    # Create box body
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000)  # Size, density
    box.SetPos(pos)
    box.SetRot(chrono.Q_from_AngZ(random.uniform(0, 2*chrono.CH_PI)))
    box.SetFixed(False)
    vehicle.GetSystem().Add(box)

# ========================================================================
# SET UP SENSOR SYSTEM
# ========================================================================
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights to scene
sensor_manager.scene.AddPointLight(chrono.ChVector3d(10, 10, 20), chrono.ChColor(1, 1, 1), 500.0)
sensor_manager.scene.AddPointLight(chrono.ChVector3d(-10, -10, 20), chrono.ChColor(0.8, 0.8, 1.0), 500.0)

# Configure camera sensor
camera_pos = chrono.ChVector3d(0.5, 0, 1.5)  # Position relative to chassis
camera_rot = chrono.Q_from_Euler123(chrono.ChVector3d(0, 0.2, 0))  # Slight downward tilt
camera_frame = chrono.ChFrameD(camera_pos, camera_rot)

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  # Parent body
    30,                        # Update rate (Hz)
    camera_frame,              # Offset pose
    1280,                      # Image width
    720,                       # Image height
    chrono.CH_PI / 3           # Horizontal FOV (60 degrees)
)
camera.SetName("Vehicle Camera")
camera.SetLag(0.05)  # 50ms lag to simulate processing delay
camera.SetCollectionWindow(0.02)  # Exposure time

# Add filters
camera.PushFilter(sens.ChFilterRGBA8Access())  # Allow RGBA8 format access
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))  # Add visualization

sensor_manager.AddSensor(camera)

# ========================================================================
# SET UP VISUALIZATION AND DRIVER
# ========================================================================
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Enhanced HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
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

# Simulation parameters
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# ========================================================================
# SIMULATION LOOP
# ========================================================================
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

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update sensor system (after advancing physics)
    sensor_manager.Update()

    # Increment counters
    step_number += 1
    realtime_timer.Spin(step_size)