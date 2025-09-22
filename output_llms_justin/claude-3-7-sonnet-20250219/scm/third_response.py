import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np
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
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced()  could be another choice here
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsys(initLoc, initRot))  # Corrected from ChCoordsysd to ChCoordsys
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
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(10, 5, 1))  # Expanded patch size

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

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

# ---------------------------------------------------------------------
# Add randomly positioned boxes to the simulation
# ---------------------------------------------------------------------
def is_close_to_vehicle(pos, vehicle_pos, min_distance=5.0):
    dx = pos.x - vehicle_pos.x
    dy = pos.y - vehicle_pos.y
    distance = math.sqrt(dx*dx + dy*dy)
    return distance < min_distance

# Get vehicle position
vehicle_pos = vehicle.GetVehicle().GetPos()

# Create and add boxes
system = vehicle.GetSystem()
num_boxes = 20

for i in range(num_boxes):
    # Generate random position (ensuring it's not too close to the vehicle)
    while True:
        x = random.uniform(-30, 30)
        y = random.uniform(-30, 30)
        pos = chrono.ChVector3d(x, y, 0.5)  # 0.5 is half the height to place on ground
        if not is_close_to_vehicle(pos, vehicle_pos):
            break
    
    # Create box with random size
    box_size_x = random.uniform(0.5, 1.5)
    box_size_y = random.uniform(0.5, 1.5)
    box_size_z = random.uniform(0.5, 2.0)
    
    box = chrono.ChBodyEasyBox(box_size_x, box_size_y, box_size_z, 1000, True, True)
    box.SetPos(pos)
    box.SetRot(chrono.Q_from_AngZ(random.uniform(0, 2*math.pi)))
    box.SetBodyFixed(False)
    
    # Add color for visualization
    col = chrono.ChColor(random.random(), random.random(), random.random())
    box.GetVisualShape(0).SetColor(col)
    
    system.Add(box)

# ---------------------------------------------------------------------
# Integrate a Sensor System
# ---------------------------------------------------------------------
# Create a sensor manager
manager = sens.ChSensorManager(system)

# Add point lights at various positions in the scene
light_positions = [
    chrono.ChVectorD(10, 10, 10),
    chrono.ChVectorD(-10, 10, 10),
    chrono.ChVectorD(10, -10, 10),
    chrono.ChVectorD(-10, -10, 10)
]

for pos in light_positions:
    # Add light with specific intensity
    manager.AddPointLight(pos, chrono.ChColor(1, 1, 1), 500.0)

# Create a camera sensor attached to the vehicle chassis
camera_offset = chrono.ChVectorD(0, 0, 1.5)  # Camera offset from vehicle position
camera_direction = chrono.ChVectorD(1, 0, 0)  # Looking forward
camera_up = chrono.ChVectorD(0, 0, 1)         # Up direction

# Create the camera
camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),                  # body camera is attached to
    30.0,                                      # update rate in Hz
    chrono.ChFrameD(camera_offset, chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))),  # offset pose
    1280,                                      # image width
    720,                                       # image height
    1.0                                        # fov
)

# Set camera settings
camera.SetName("Camera Sensor")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)

# Create a filter to visualize the camera feed
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View", 0.8))

# Add noise to make it more realistic
camera.PushFilter(sens.ChFilterRGBNoise(0.1, 0.1, 0.1))

# Save the camera data to disk
camera.PushFilter(sens.ChFilterSave("camera/"))

# Add the camera to the sensor manager
manager.AddSensor(camera)

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
    
    # Update sensor manager
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)