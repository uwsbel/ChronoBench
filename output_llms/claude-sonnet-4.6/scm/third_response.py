import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
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
terrain.SetSoilParameters(
    2e6,   # Bekker Kphi
    0,     # Bekker Kc
    1.1,   # Bekker n exponent
    0,     # Mohr cohesive limit (Pa)
    30,    # Mohr friction limit (degrees)
    0.01,  # Janosi shear coefficient (m)
    2e8,   # Elastic stiffness (Pa/m), before plastic yield
    3e4    # Damping (Pa s/m), proportional to negative vertical speed
)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution)
terrain.Initialize(20, 20, 0.02)

# ---------------------------------------------------------------------------
# Add randomly positioned boxes to the scene
# ---------------------------------------------------------------------------
random.seed(42)
num_boxes = 10

# Create a contact material for the boxes (SMC contact method)
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.5)
box_mat.SetRestitution(0.1)

for i in range(num_boxes):
    # Generate a random position, ensuring it is not within the vehicle's initial area
    while True:
        bx = random.uniform(-15, 15)
        by = random.uniform(-15, 15)
        # Ensure the box is at least 3 m away from the vehicle's initial position
        dist = math.sqrt((bx - initLoc.x) ** 2 + (by - initLoc.y) ** 2)
        if dist > 3.0:
            break

    box_size_x = random.uniform(0.3, 1.0)
    box_size_y = random.uniform(0.3, 1.0)
    box_size_z = random.uniform(0.3, 1.0)

    box_body = chrono.ChBodyEasyBox(
        box_size_x, box_size_y, box_size_z,
        1000,       # density (kg/m^3)
        True,       # create visual shape
        True,       # create collision shape
        box_mat     # contact material
    )
    box_body.SetPos(chrono.ChVector3d(bx, by, box_size_z / 2.0))
    box_body.SetFixed(False)
    vehicle.GetSystem().Add(box_body)

# ---------------------------------------------------------------------------
# Create the vehicle Irrlicht interface
# ---------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
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

# ---------------------------------------------------------------------------
# Sensor System Setup
# ---------------------------------------------------------------------------

# Create a sensor manager attached to the simulation system
manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights at various positions in the scene
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 10),
    chrono.ChColor(1, 1, 1),
    500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 10, 10),
    chrono.ChColor(1, 1, 1),
    500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, -10, 10),
    chrono.ChColor(1, 1, 1),
    500.0
)

# --- Camera Sensor ---
cam_update_rate = 30          # camera update rate [Hz]
cam_width = 1280              # image width [pixels]
cam_height = 720              # image height [pixels]
cam_fov = 1.408               # horizontal field of view [rad] (~80 deg)
cam_lag = 0.0                 # sensor lag [s]
cam_collection_time = 1.0 / cam_update_rate  # collection window [s]

# Offset pose: camera mounted slightly in front of and above the chassis center
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.5, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  # body the camera is attached to
    cam_update_rate,           # update rate [Hz]
    cam_offset_pose,           # offset pose relative to the body
    cam_width,                 # image width [pixels]
    cam_height,                # image height [pixels]
    cam_fov                    # horizontal field of view [rad]
)
camera.SetName("ChassisCamera")
camera.SetLag(cam_lag)
camera.SetCollectionWindow(cam_collection_time)

# Add a filter to visualize the camera feed in a window
camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Vehicle Camera Feed"))

# Add the camera sensor to the manager
manager.AddSensor(camera)

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
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

    # Update the sensor manager
    manager.Update()

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)