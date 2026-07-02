import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import math
import random

# Set Chrono data path
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
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS

# Create HMMWV vehicle
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

# Create SCM deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# Add randomly positioned boxes
num_boxes = 5
box_size = 0.5
for _ in range(num_boxes):
    while True:
        x = random.uniform(-20, 20)
        y = random.uniform(-20, 20)
        z = random.uniform(0.1, 2.0)
        # Ensure boxes are not inside the vehicle's initial position
        if (x - initLoc.x)**2 + (y - initLoc.y)**2 > 25:
            break
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(True)
    box.GetVisualShape(0).SetMaterial(0)
    vehicle.GetSystem().AddBody(box)

# Create sensor manager
sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())
sensor_manager.scene.AddPointLight(chrono.ChVector3d(10, 10, 10), chrono.ChColor(1, 1, 1), 500)
sensor_manager.scene.AddPointLight(chrono.ChVector3d(-10, -10, 10), chrono.ChColor(1, 1, 1), 500)

# Create camera sensor
camera = sensor.ChCameraSensor(
    vehicle.GetChassisBody(),  # Attach to chassis
    30,                        # Update rate (Hz)
    chrono.ChFrame<>(chrono.ChVector3d(0, 0, 1.5), chrono.Q_from_AngAxis(0, chrono.ChVector3d(1, 0, 0))),
    640, 480,                  # Resolution
    60,                        # FOV (degrees)
    0.01, 100                  # Near/far clipping
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0.01)

# Add visualization filter
camera.PushFilter(sensor.ChFilterVisualize(640, 480, "Vehicle Camera Feed"))
sensor_manager.AddSensor(camera)

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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
    
    # Update sensor manager
    sensor_manager.Update()
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)