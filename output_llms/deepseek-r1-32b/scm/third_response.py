import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random
import numpy as np

# SetChronoDataPath
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

# Create randomly positioned boxes
def add_random_box(system, num_boxes=10):
    for _ in range(num_boxes):
        # Generate random position avoiding vehicle area
        while True:
            x = random.uniform(-50, 50)
            y = random.uniform(-50, 50)
            z = random.uniform(0, 5)
            # Check distance from vehicle
            vehicle_pos = vehicle.GetChassisBody().GetPos()
            dist = math.sqrt((x - vehicle_pos.x)**2 + (y - vehicle_pos.y)**2)
            if dist > 5:  # Keep boxes at least 5m away from vehicle
                break
        
        # Create box
        box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
        box.SetPos(chrono.ChVector3d(x, y, z))
        box.SetRot(chrono.ChQuaterniond(1, random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)).Normalize())
        box.SetMaterialSurface(chrono.ChMaterialSurface())
        box.GetMaterialSurface().SetYoungModulus(2e8)
        box.GetMaterialSurface().SetFriction(0.5)
        
        # Random color
        box.SetColor(chrono.ChColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)))
        
        # Add to system
        system.Add(box)

add_random_box(vehicle.GetSystem(), num_boxes=20)

# Create sensor manager
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights
light1 = irr.ChPointLight()
light1.SetPos(chrono.ChVector3d(10, 10, 10))
light1.SetIntensity(1)
sensor_manager.AddLight(light1)

light2 = irr.ChPointLight()
light2.SetPos(chrono.ChVector3d(-10, -10, 10))
light2.SetIntensity(1)
sensor_manager.AddLight(light2)

# Add camera sensor
camera = sens.ChCameraSensor(sensor_manager)
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(0, 0, 1.71))  # Position relative to chassis
camera.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  # Looking forward
camera.AttachTo(vehicle.GetChassisBody())  # Mount on vehicle chassis

# Add filter to visualize camera feed
camera_filter = sens.ChFilterVisualize()
camera_filter.SetWindowTitle("Camera Feed")
camera_filter.SetFrequency(30)  # Update rate
camera.AddFilter(camera_filter)

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

# Add sensor manager to visual system
vis.AttachSensorManager(sensor_manager)

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
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
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
    sensor_manager.Synchronize(time)

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