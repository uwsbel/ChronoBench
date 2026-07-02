import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path and vehicle data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method and visualization
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS

# Create FEDA vehicle and initialize
vehicle = veh.FEDA()
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

# Create terrain with grass texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.3, 0.6, 0.3))  # Grass-like color
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA Vehicle with Sensor Camera')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager and add point lights
manager = chrono.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3d(5, 5, 5), 100, 1.0)
manager.scene.AddPointLight(chrono.ChVector3d(-5, 5, 5), 100, 1.0)

# Add camera sensor for first-person view
camera = chrono.ChCameraSensor(
    vehicle.GetChassisBody(),  # Attach to chassis
    1280, 720,                # Resolution
    chrono.ChFrame3d(chrono.ChVector3d(0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),  # Position and orientation
    60,                        # FOV in degrees
    0.01,                      # Near clipping plane
    100                        # Far clipping plane
)
camera.SetName("First Person Camera")
camera.SetLag(0)
camera.SetUpdateRate(50)
manager.AddSensor(camera)

# Add visualization filter for camera output
vis_filter = chrono.ChFilterVisualize(1280, 720, "Camera Sensor Output")
manager.AddFilter(vis_filter)

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Print vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Simulation loop parameters
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene at specified intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize modules
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
    manager.Update()
    
    step_number += 1
    realtime_timer.Spin(step_size)