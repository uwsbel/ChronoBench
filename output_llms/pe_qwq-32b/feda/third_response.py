import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensors as sensors  # Added for sensor manager
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)  # Use ChVectorD instead of ChVector3d
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # Use ChQuaternionD

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method setup
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Rendering settings
render_step_size = 1.0 / 50  # 50 FPS

# Create the FEDA vehicle
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system (Bullet for collision detection with NSC solver)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChMaterialSurfaceNSC()  # Use ChMaterialSurfaceNSC for NSC contact
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # Changed texture to grass
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create sensor manager
sensor_manager = sensors.ChSensorManager(vehicle.GetSystem())

# Add point lights for better illumination
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()  # Keep directional light
vis.AddPointLight(chrono.ChVectorD(5, 5, 10), 200, chrono.ChColor(1,1,1), 200)
vis.AddPointLight(chrono.ChVectorD(-5, -5, 10), 200, chrono.ChColor(1,1,1), 200)
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Create camera sensor
cam_pos = chrono.ChVectorD(0, 0, 1.5)  # Position relative to chassis center
cam_rot = chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0))  # Looking forward
cam_frame = chrono.ChFrameD(cam_pos, cam_rot)
cam = sensors.ChCameraSensor(
    sensor_manager,
    vehicle.GetChassisBody(),
    cam_frame,
    1920, 1080,  # High resolution
    90,  # Field of View
    0.1, 100  # Near/far clipping planes
)
vis_filter = sensors.ChVisualizationCamera(cam, True)
sensor_manager.AddVisualizationFilter(vis_filter)
sensor_manager.AddSensor(cam)

# Initialize driver with vehicle parameter
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize(vehicle)  # Fixed parameter

print("VEHICLE MASS: ", vehicle.GetMass())  # Fixed method call

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
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
    vis.Advance(step_size)
    
    sensor_manager.Update()  # Update sensor manager
    
    step_number += 1
    realtime_timer.Spin(step_size)