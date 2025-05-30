import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_RIGID

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point for chase camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # FPS = 50

# Create the vehicle
vehicle = veh.HMMWV_Full()  # or veh.HMMWV_Reduced()
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

# Create SCM terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6,   # Kphi
    0,     # Kc
    1.1,   # n exponent
    0,     # Mohr cohesive
    30,    # Mohr friction (degrees)
    0.01,  # Janosi shear coefficient
    2e8,   # Elastic stiffness
    3e4    # Damping
)

# Add moving patch (around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(),
                       chrono.ChVector3d(0, 0, 0),
                       chrono.ChVector3d(5, 3, 1))

# Set plotting type
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize terrain mesh
terrain.Initialize(20, 20, 0.02)

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver
driver = veh.ChInteractiveDriverIRR(vis)
# Driver input deltas
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Calculate steps per frame
render_steps = math.ceil(render_step_size / step_size)

# Create a random seed or fixed seed for reproducibility
random.seed(42)

# Create random boxes in the scene (excluding vicinity of vehicle)
num_boxes = 20  # number of boxes to add
boxes = []

for _ in range(num_boxes):
    # Random position within terrain bounds, avoiding vehicle area (~x<-8 to x=8)
    while True:
        x = random.uniform(-terrainLength/2 + 5, terrainLength/2 - 5)
        y = random.uniform(-terrainWidth/2 + 5, terrainWidth/2 - 5)
        # Ensure boxes are not too close to initial vehicle position
        if abs(x - initLoc.x) > 2 and abs(y - initLoc.y) > 2:
            break
    size_x = random.uniform(0.2, 1.0)
    size_y = random.uniform(0.2, 1.0)
    size_z = random.uniform(0.2, 1.0)
    box = chrono.ChBodyEasyBox(size_x, size_y, size_z,  # dimensions
                                100,  # density
                                True,  # visualization
                                True)  # collision
    box.SetPosition(chrono.ChVector3d(x, y, terrainHeight + size_z/2))
    # Optional: set color or other properties
    # box.GetVisualShape().SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    # Add to system
    vehicle.GetSystem().Add(box)
    boxes.append(box)

# Integrate sensor system
# Note: PyChrono's sensors are based on the Chrono Sensors module
# For this script, assume we have access to chrono.sensor module
import pychrono.sensor as sensitivity

# Create a sensor manager
sensor_manager = sensitivity.ChSensorManager()
sensor_manager_scene = sensor_manager.GetScene()
sensor_manager.SetScene(sensor_manager_scene)

# Add point lights at various positions
for i in range(3):
    light_pos = chrono.ChVector3d(
        random.uniform(-10, 10),
        random.uniform(-10, 10),
        10
    )
    # Add point light to the scene
    # Since pychrono.sensor may not have direct method, assume using Irrlicht interface
    # To add a light in Irrlicht, typically via vis.SceneManager()
    vis_scene_mgr = vis.GetSceneManager()
    light = vis_scene_mgr.AddLightSceneNode(
        None,  # parent node
        chrono.ChVector3d(light_pos.x, light_pos.y, light_pos.z),
        chrono.ChColor(1, 1, 1),  # color white
        10.0  # radius
    )

# Add a camera sensor attached to vehicle chassis
# Using the sensor module
camera = sensitivity.ChCameraSensor(
    vehicle.GetChassisBody(),  # parent body
    30,  # update rate (Hz)
    chrono.ChFrameFct(
        chrono.ChVectorF(0.0, 0.0, 1.5),  # position relative to chassis
        chrono.ChQuaternionF(1, 0, 0, 0)  # orientation
    )
)
camera.SetName("FrontCamera")
camera.SetResolution(640, 480)
camera.SetFocalLength(35)  # in mm, optional
camera.SetFieldOfView(45.0)  # in degrees
sensor_manager.AddSensor(camera)

# Set a filter to visualize the camera feed
# For visualization, we can opt to display the camera image on the console or save to file
# Here, assume we provide a visualization function (placeholder)
def visualize_camera_feed(sensor):
    # Placeholder: In actual code, retrieve image data from sensor
    # and visualize or save it accordingly.
    pass

# Register callback for camera sensor to visualize feed (if supported)
# For the sake of this example, assume we can set a callback
# camera.RegisterFilter(visualize_camera_feed)

# Main simulation loop
# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified intervals
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Fetch driver inputs
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
    
    # Advance sensors
    sensor_manager.Process()
    sensor_manager.Update()
    sensor_manager.Render()
    # Alternatively, if only the camera sensor is involved:
    # camera.Synchronize(time)

    vis.Advance(step_size)

    step_number += 1

    # Keep real time in sync
    realtime_timer.Spin(step_size)