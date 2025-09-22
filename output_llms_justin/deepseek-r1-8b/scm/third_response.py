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
chassis_collision_type = veh.CollisionType_PRIMITIVES

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

# Add visualization components
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager and camera
sensor_manager = veh.ChSensorManager()
camera_sensor = veh.ChCameraSensor()
camera_sensor.SetName('Chassis Camera')
camera_sensor.SetResolution(1280, 720)
camera_sensor.SetFieldOfView(60.0)
camera_sensor.SetSensorPosition(chrono.ChCoordsysd(0.5, 1.5, 5.0))
camera_sensor.SetSensorRotation(chrono.ChQuaterniond(1, 0, 0, 0))
sensor_manager.AddSensor(camera_sensor)
sensor_manager.AttachToVehicle(vehicle.GetVehicle())

# Add point lights
lights = veh.ChLightSystem()
lights.AddLightDirectional()
lights.AddLightPoint(chrono.ChCoordsysd(0, 0, 5, 0, 0, 0), chrono.ChColor(1, 1, 1))
lights.AddLightPoint(chrono.ChCoordsysd(10, 0, 5, 0, 0, 0), chrono.ChColor(1, 1, 1))
lights.AddLightPoint(chrono.ChCoordsysd(0, 10, 5, 0, 0, 0), chrono.ChColor(1, 1, 1))
lights.AddLightPoint(chrono.ChCoordsysd(-10, 0, 5, 0, 0, 0), chrono.ChColor(1, 1, 1))
lights.AddLightPoint(chrono.ChCoordsysd(0, -10, 5, 0, 0, 0), chrono.ChColor(1, 1, 1))

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.Initialize()

# Set time response for steering and throttle
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Function to add random boxes around the vehicle
def AddRandomBoxes(vehicle, system):
    while True:
        x = vehicle.GetInitPosition().GetX() + (vehicle.GetSizeX() * 0.5 - 5, vehicle.GetSizeX() * 0.5 - 5)
        y = vehicle.GetInitPosition().GetY() + (vehicle.GetSizeY() * 0.5 - 5, vehicle.GetSizeY() * 0.5 - 5)
        z = vehicle.GetInitPosition().GetZ() + (vehicle.GetSizeZ() * 0.5 - 5, vehicle.GetSizeZ() * 0.5 - 5)
        
        # Check if box is inside the vehicle
        box = veh.ChBodyEasyBox(0.5, 0.5, 0.5, x, y, z)
        is_inside = False
        for i in range(4):
            if box.GetBody(i).GetCollisionBox().GetCenter().GetDistanceSquared(vehicle.GetCollisionBox().GetCenter()) < 2.0:
                is_inside = True
                break
        if not is_inside:
            system.AddBody(box)
            break

# Add random boxes
system = vehicle.GetSystem()
AddRandomBoxes(vehicle, system)

# Set camera filter
camera_filter = veh.ChCameraFilter()
camera_filter.SetCameraSensor(camera_sensor)
camera_filter.SetFilterType(veh.ChCameraFilterType_VISUALIZATION)
camera_filter.SetOverlayColor(chrono.ChColor(0, 1, 0))  # Green overlay
camera_filter.Initialize()
vis.AddFilter(camera_filter)

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Simulation loop
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)