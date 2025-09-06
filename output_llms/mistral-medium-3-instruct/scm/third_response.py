import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import math
import random

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

# Point on chassis tracked by the camera
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

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add randomly positioned boxes to the scene
def add_random_boxes(system, num_boxes=10, vehicle_pos=initLoc, min_distance=5.0):
    for i in range(num_boxes):
        # Generate random position ensuring it's not too close to the vehicle
        while True:
            x = random.uniform(-terrainLength/2, terrainLength/2)
            y = random.uniform(-terrainWidth/2, terrainWidth/2)
            z = random.uniform(0.5, 2.0)

            box_pos = chrono.ChVector3d(x, y, z)
            distance = (box_pos - vehicle_pos).Length()

            if distance > min_distance:
                break

        # Create box with random size
        size_x = random.uniform(0.5, 2.0)
        size_y = random.uniform(0.5, 2.0)
        size_z = random.uniform(0.5, 2.0)

        box = chrono.ChBodyEasyBox(size_x, size_y, size_z, 1000, True, True)
        box.SetPos(box_pos)
        box.SetBodyFixed(False)
        box.GetVisualModel().SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        system.Add(box)

add_random_boxes(vehicle.GetSystem())

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

# Create sensor manager
sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())

# Add point lights to the scene
def add_point_lights(system, num_lights=5):
    for i in range(num_lights):
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        z = random.uniform(3.0, 8.0)

        light = chrono.ChPointPointLight()
        light.SetPos(chrono.ChVector3d(x, y, z))
        light.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        light.SetIntensity(1.0)
        system.Add(light)

add_point_lights(vehicle.GetSystem())

# Create and add camera sensor
camera = sensor.ChCameraSensor(
    vehicle.GetChassisBody(),  # parent body
    10,                        # update rate in Hz
    True,                      # enable save to disk
    "camera_sensor"            # name
)

# Configure camera settings
camera.SetName("vehicle_camera")
camera.SetOffsetPose(chrono.ChFrameD(chrono.ChVector3d(0, 0, 2.0), chrono.ChQuaternionD(1, 0, 0, 0)))
camera.SetLensModel(sensor.ChLensModelType.PERSPECTIVE)
camera.SetImageWidth(640)
camera.SetImageHeight(480)
camera.SetFocalLength(0.035)  # 35mm focal length
camera.SetHorizontalFOV(chrono.ChMath.PI / 3)  # 60 degree FOV

# Add filter to visualize camera feed
filter = sensor.ChFilterVisualize(640, 480, "Camera Feed")
camera.AddFilter(filter)

# Add camera to sensor manager
sensor_manager.AddSensor(camera)

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

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
    sensor_manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)