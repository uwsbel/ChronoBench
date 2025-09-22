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

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_RIGID

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0  # Corrected from hardcoded 20 in original
terrainWidth = 100.0   # Corrected from hardcoded 20 in original

# Camera tracking point
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50

# Create vehicle
vehicle = veh.HMMWV_Full()
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
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, 0.02)  # Fixed initialization parameters

# Add random boxes
num_boxes = 10
box_size = 1.0
min_distance = 5.0

for _ in range(num_boxes):
    while True:
        x = random.uniform(-50, 50)
        y = random.uniform(-50, 50)
        dx = x + 8
        dy = y
        if math.sqrt(dx**2 + dy**2) > min_distance:
            break
    z = 0.5 * box_size
    
    box_body = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(x, y, z))
    box_body.SetBodyFixed(True)
    vehicle.GetSystem().Add(box_body)

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

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Add point lights
manager.AddLight(sens.ChLight(chrono.ChVector3f(10, 10, 10), chrono.ChVector3f(1, 1, 1)))
manager.AddLight(sens.ChLight(chrono.ChVector3f(-10, -10, 10), chrono.ChVector3f(0.8, 0.8, 0.8)))

# Create camera sensor
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.ChQuaterniond(1, 0, 0, 0)
)
camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    60,
    offset_pose,
    1280,
    720,
    chrono.CH_C_PI / 3
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))
manager.AddSensor(camera)

# Simulation loop
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    manager.Synchronize(time)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    manager.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)