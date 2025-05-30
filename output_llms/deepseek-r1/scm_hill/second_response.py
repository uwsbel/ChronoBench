import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Simulation setup
contact_method = chrono.ChContactMethod_SMC
contact_vis = False
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 20

# Create and initialize vehicle
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

# Create deformable terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# Add random box obstacles
np.random.seed(42)
for _ in range(5):
    x = np.random.uniform(-20, 20)
    y = np.random.uniform(-10, 10)
    z = 0.5
    
    box = chrono.ChBody()
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(True)
    
    material = chrono.ChMaterialSurfaceSMC()
    box_shape = chrono.ChCollisionShapeBox(material, 1.0, 1.0, 1.0)
    box.AddCollisionShape(box_shape)
    box.EnableCollision(True)
    
    vis_shape = chrono.ChVisualShapeBox(1.0, 1.0, 1.0)
    vis_shape.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
    box.AddVisualShape(vis_shape)
    
    vehicle.GetSystem().Add(box)

# Create sensor system
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 1000)

# Configure lidar sensor
lidar_pos = chrono.ChVector3d(0, 0, 2)
lidar_rot = chrono.Q_from_AngZ(0)
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),
    10,
    chrono.ChFramed(lidar_pos, lidar_rot),
    360,
    1,
    chrono.CH_2PI,
    chrono.CH_PI/12,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    0.001,
    0.001
)
lidar.SetName("Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add lidar filters
lidar.PushFilter(sens.ChFilterVisualize(640, 360, "Lidar Output"))
lidar.PushFilter(sens.ChFilterPCfromDepth())
sensor_manager.AddSensor(lidar)

# Initialize visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Driver setup
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop setup
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
    
    # Synchronize systems
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
    sensor_manager.Update()
    
    step_number += 1
    realtime_timer.Spin(step_size)