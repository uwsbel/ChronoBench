import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random

# Set data paths
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

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                         0,      # Bekker Kc
                         1.1,    # Bekker n exponent
                         0,      # Mohr cohesive limit (Pa)
                         30,     # Mohr friction limit (degrees)
                         0.01,   # Janosi shear coefficient (m)
                         2e8,    # Elastic stiffness (Pa/m)
                         3e4)    # Damping (Pa s/m)

# Add moving patch around vehicle
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize terrain
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes
n_boxes = 20
box_size = chrono.ChVector3d(0.5, 0.5, 0.5)  # Dimensions
density = 1000  # kg/m³
safe_radius = 5.0  # Minimum distance from vehicle

for _ in range(n_boxes):
    while True:
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        dist = math.sqrt((x - initLoc.x)**2 + (y - initLoc.y)**2)
        if dist >= safe_radius:
            break
    z = terrainHeight + box_size.z / 2
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, density)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(True)
    box.SetCollide(True)
    vehicle.GetSystem().Add(box)

# Create vehicle visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights to the scene
manager.scene.AddPointLight(chrono.ChVector3d(0, 10, 5), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.AddPointLight(chrono.ChVector3d(0, -10, 5), chrono.ChColor(1, 1, 1), 1000.0)

# Create camera sensor attached to chassis
chassis = vehicle.GetChassisBody()
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 3), 
                              chrono.Q_from_AngAxis(chrono.CH_PI, chrono.ChVector3d(1, 0, 0)))
camera = sens.ChCameraSensor(
    chassis,                  # Attached to chassis
    30,                       # Update rate (Hz)
    offset_pose,              # Position and orientation
    1280, 720,                # Image resolution
    90,                       # Field of view (degrees)
    1,                        # Super-sampling factor
    sens.CameraLensModelType_PINHOLE,
    False                     # No global illumination
)
camera.SetName("Vehicle Camera")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)

# Add visualization filter
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))
manager.AddSensor(camera)

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set input response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Calculate render steps
render_steps = math.ceil(render_step_size / step_size)

# Initialize counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Simulation loop
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

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update sensors
    manager.Update()

    # Increment step count
    step_number += 1

    # Real-time synchronization
    realtime_timer.Spin(step_size)