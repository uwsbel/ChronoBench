import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

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
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
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

# Add visualization for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Add collision system
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

# Add random boxes to the scene
# Create a list to hold box objects
boxes = []
# Generate random positions for the boxes
for _ in range(20):
    # Generate random position for the box
    box_pos = chrono.ChVector3d(
        -50 + chrono.RandFloat(-30, 30),
        -50 + chrono.RandFloat(-30, 30),
        0.0 + chrono.RandFloat(-10, 10)
    )
    
    # Create a box with random size
    box_size = chrono.ChVector3d(
        1 + chrono.RandFloat(0, 1),
        1 + chrono.RandFloat(0, 1),
        1 + chrono.RandFloat(0, 1)
    )
    
    # Create a box with random material
    box_material = chrono.ChMaterial()
    box_material.SetDiffuseColor(chrono.ChColor(
        chrono.RandFloat(0, 1),
        chrono.RandFloat(0, 1),
        chrono.RandFloat(0, 1)
    ))
    
    # Create the box and add it to the system
    box = veh.ChBodyEasyBox()
    box.SetPosition(box_pos)
    box.SetSize(box_size)
    box.SetMaterial(box_material)
    box.Create()
    box.AttachTo(vehicle.GetSystem())
    boxes.append(box)

# Create the sensor system
sensor_manager = veh.ChSensorManager()
# Create a camera sensor
camera = veh.ChCameraSensor()
camera.SetPosition(trackPoint)
camera.SetFieldOfView(90.0)  # Set field of view
camera.SetResolution(256, 256)  # Set resolution
camera.AttachTo(sensor_manager)
sensor_manager.AddSensor(camera)

# Add point lights to the scene
light_positions = [
    chrono.ChVector3d(0, 0, 10),
    chrono.ChVector3d(10, 0, 10),
    chrono.ChVector3d(-10, 0, 10),
    chrono.ChVector3d(0, 10, 10),
    chrono.ChVector3d(0, -10, 10)
]

for pos in light_positions:
    light = veh.ChLightSource()
    light.SetPosition(pos)
    light.SetDiffuseColor(chrono.ChColor(1, 1, 1))
    light.Create()
    light.AttachTo(vehicle.GetSystem())

# Add the sensor manager to the vehicle system
sensor_manager.Initialize()
sensor_manager.AttachTo(vehicle.GetSystem())

# Set up the camera feed visualization
camera_render_target = veh.ChRenderTarget()
camera_render_target.AttachTo(vis.GetDevice())
camera.AttachTo(camera_render_target)
sensor_manager.AddRenderTarget(camera_render_target, "Camera Feed")

# Set up the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
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

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
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

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)