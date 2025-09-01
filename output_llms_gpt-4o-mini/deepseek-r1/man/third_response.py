import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize the MAN vehicle
vehicle = veh.MAN_10t() 
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), 
    terrainLength, terrainWidth, 0.2)  # Added thickness parameter

patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create random boxes
num_boxes = 20
for _ in range(num_boxes):
    x = chrono.ChRandom.Get() * 40 - 20  # -20 to 20
    y = chrono.ChRandom.Get() * 40 - 20
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)  # Size 1x1x1, density 1000
    box.SetPos(chrono.ChVector3d(x, y, 0.5))
    box.SetFixed(True)
    vehicle.GetSystem().Add(box)

# Create sensor manager
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetKeyframeSizeFromTimeStep(step_size, 1)

# Create and add lidar sensor
lidar_offset = chrono.ChVector3d(2, 0, 1.5)  # Front of vehicle
lidar_transform = chrono.ChFramed(lidar_offset, chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),
    10,  # Update rate in Hz
    lidar_transform,
    360,  # Horizontal samples
    100,  # Vertical samples
    math.radians(360),  # Horizontal FOV
    math.radians(30),   # Vertical FOV
    math.radians(15),   # Max vertical angle
    math.radians(-15),  # Min vertical angle
    100.0  # Max distance
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterPCfromDepth())
sensor_manager.AddSensor(lidar)

# Create vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
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

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop setup
render_steps = math.ceil(render_step_size / step_size)
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
    vis.Advance(step_size)
    
    # Update sensor manager
    sensor_manager.Update()

    step_number += 1
    realtime_timer.Spin(step_size)