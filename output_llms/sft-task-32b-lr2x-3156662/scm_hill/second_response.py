import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)
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
render_step_size = 1.0 / 20  # FPS = 50

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
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# Create 5 box obstacles
obstacles = []
for _ in range(5):
    obstacle = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True)
    obstacle.SetPos(chrono.ChVector3d(np.random.uniform(-40, 40), np.random.uniform(-40, 40), 0.5))
    obstacle.SetBodyFixed(True)
    obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.1, 0.1))
    vehicle.GetSystem().AddBody(obstacle)
    obstacles.append(obstacle)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 100), chrono.ChColor(1, 1, 1), 500)

# Create lidar sensor
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(), 
    update_rate=5.0, 
    parent=vehicle.GetChassisBody(), 
    offset_pose=chrono.ChFramed(chrono.ChVector3d(0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    horizontal_samples=360, 
    vertical_samples=1, 
    horizontal_fov=2 * math.pi, 
    vertical_fov=0.05, 
    max_distance=100.0, 
    min_distance=0.05
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.02)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(128, 128, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Simulation loop parameters
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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
    
    # Update sensor manager
    manager.Update()
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)