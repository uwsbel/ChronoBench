import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random
import numpy as np

# Set up random seed for reproducibility
random.seed(42)

# Create some randomly positioned boxes
def add_random_boxes(system, num_boxes=10, min_dist_from_vehicle=5.0):
    vehicle_pos = vehicle.GetChassisBody().GetPos()
    box_size = 1.0
    box_mass = 100.0
    
    for _ in range(num_boxes):
        # Generate random position within terrain bounds
        x = random.uniform(-terrainLength/2 + box_size, terrainLength/2 - box_size)
        y = random.uniform(-terrainWidth/2 + box_size, terrainWidth/2 - box_size)
        z = terrainHeight + box_size/2
        
        # Ensure boxes are not too close to vehicle
        dist = np.linalg.norm([x - vehicle_pos.x, y - vehicle_pos.y])
        if dist < min_dist_from_vehicle:
            continue
            
        # Create box body
        box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_mass, True, True)
        box.SetPos(chrono.ChVector3d(x, y, z))
        box.SetMaterialSurface(chrono.ChMaterialSurface())
        system.AddBody(box)

# Initialize Chrono and vehicle
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create sensor manager
sensor_manager = irr.ChSensorManager()
sensor_manager.SetRenderTarget(chrono.GetChronoRenderSystem().GetWindow())

# Add point lights
light1 = irr.ChLightPoint()
light1.SetIntensity(1.0)
light1.SetPosition(chrono.ChVector3d(10, 10, 10))
sensor_manager.AddSensor(light1)

light2 = irr.ChLightPoint()
light2.SetIntensity(1.0)
light2.SetPosition(chrono.ChVector3d(-10, -10, 10))
sensor_manager.AddSensor(light2)

# Create camera sensor
camera = irr.ChCameraSensor()
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(0, 0, 2))
camera.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
sensor_manager.AddSensor(camera)

# Attach camera to vehicle chassis
chassis = vehicle.GetChassisBody()
camera.AttachTo(chassis, chrono.ChVector3d(0, 0, 1.5), chrono.ChVector3d(0, 1, 0))

# Add filter for camera feed visualization
filter = irr.ChFilterCamera()
filter.SetCamera(camera)
sensor_manager.AddFilter(filter)

# Rest of the original code with modifications...

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# Initialize vehicle and add random boxes
vehicle.Initialize()
add_random_boxes(vehicle.GetSystem())

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Boxes and Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add sensor manager to visualization
vis.GetSensorManager().AddSensorManager(sensor_manager)

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Update sensor manager
    sensor_manager.Update()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        filter.Render()
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

    # Increment frame number
    step_number += 1

    # Spin for real time
    realtime_timer.Spin(step_size)