import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
sens.SetDataPath(chrono.GetChronoDataPath())  # Set sensor data path

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

# Point tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize vehicle
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
terrain.Initialize(20, 20, 0.02)

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
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

# =======================================================================
# ADD RANDOM BOXES TO THE SCENE
# =======================================================================
system = vehicle.GetSystem()
random.seed(42)  # For reproducibility

# Generate 20 randomly positioned boxes avoiding vehicle start area
for _ in range(20):
    while True:
        # Generate random position within terrain bounds
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        # Ensure box isn't near vehicle start location
        if math.sqrt((x - initLoc.x)**2 + (y - initLoc.y)**2) > 3.0:
            break
    
    # Create box with size 1x1x1 and density 1000 kg/m³
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(chrono.ChVector3d(x, y, 0.5))  # Position box on ground
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)

# =======================================================================
# SENSOR SYSTEM INTEGRATION
# =======================================================================
# Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(10, 10, 10), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.AddPointLight(chrono.ChVector3d(-10, 10, 10), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))

# Camera sensor configuration
camera_update_rate = 30  # Hz
image_width = 1280
image_height = 720
fov = 70  # degrees
exposure_time = 0.02  # seconds

# Create camera offset position and orientation
offset_pos = chrono.ChVector3d(-1, 0, 1)  # Behind and above chassis
# Rotation matrix: camera looks forward (X) with Z up
R = chrono.ChMatrix33d()
R.SetRow(0, chrono.ChVector3d(0, -1, 0))   # Camera X (right) = chassis -Y
R.SetRow(1, chrono.ChVector3d(0, 0, -1))   # Camera Y (down) = chassis -Z
R.SetRow(2, chrono.ChVector3d(1, 0, 0))    # Camera Z (forward) = chassis X
offset_rot = chrono.ChQuaterniond(R)
offset_pose = chrono.ChFramed(offset_pos, offset_rot)

# Create and configure camera sensor
camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),     # Attach to chassis
    camera_update_rate,           # Update rate
    offset_pose,                  # Position and orientation
    image_width,                  # Image width
    image_height,                 # Image height
    fov,                          # Field of view
    1,                            # Super sampling factor
    sens.CameraLensModelType_PINHOLE,
    False                         # Not global exposure
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)                 # No lag
camera.SetCollectionWindow(exposure_time)  # Exposure time
camera.PushFilter(sens.ChFilterVisualize(640, 360, "Camera Feed"))  # Visualize feed
manager.AddSensor(camera)

# =======================================================================
# SIMULATION LOOP
# =======================================================================
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
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
    
    # Update sensor manager
    manager.Update()

    # Increment counters
    step_number += 1
    realtime_timer.Spin(step_size)