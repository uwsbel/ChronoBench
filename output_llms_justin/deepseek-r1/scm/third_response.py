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

# Enable moving patch feature
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes
num_boxes = 20
box_size = chrono.ChVector3d(0.5, 0.5, 0.5)  # Dimensions (x,y,z)
density = 1000  # kg/m³

# Safe distance from vehicle initial position
safe_radius = 3.0
safe_radius_sq = safe_radius**2

for i in range(num_boxes):
    while True:
        # Generate random position within 40m range
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        # Check distance from vehicle start position
        dx = x - initLoc.x
        dy = y - initLoc.y
        if dx*dx + dy*dy >= safe_radius_sq:
            break
    
    z = box_size.z / 2  # Center at half-height
    pos = chrono.ChVector3d(x, y, z)
    
    # Create box with collision and visualization
    box_body = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, density, True, True)
    box_body.SetPos(pos)
    box_body.SetFixed(False)
    box_body.GetVisualShape(0).SetColor(chrono.ChColor(
        random.random(), 
        random.random(), 
        random.random()
    ))
    vehicle.GetSystem().Add(box_body)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for inputs
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 100), chrono.ChColor(1, 1, 1), 1000.0)

# Create camera sensor attached to chassis
camera_body = vehicle.GetChassisBody()
camera_pos = chrono.ChVector3d(-2, 0, 1)  # Position relative to chassis
camera_rot = chrono.Q_from_AngZ(math.pi) * chrono.Q_from_AngY(-math.pi/2)  # Look forward
camera_pose = chrono.ChFramed(camera_pos, camera_rot)

camera = sens.ChCameraSensor(
    camera_body,            # Attached body
    30,                     # Update rate (Hz)
    camera_pose,            # Offset pose
    1280,                   # Image width
    720,                    # Image height
    math.radians(70)        # Horizontal FOV
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)

# Add filters
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))
manager.AddSensor(camera)

# ---------------
# Simulation loop
# ---------------
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize counters
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
    
    # Update sensor manager
    manager.Update()
    
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin for real-time synchronization
    realtime_timer.Spin(step_size)