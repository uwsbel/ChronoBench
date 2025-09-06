import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens  # Added sensor module import
import math

# Set data paths
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

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50

# Create vehicle and configure
vehicle = veh.FEDA()
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

# Configure collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with grass texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Corrected texture path using chrono.GetChronoDataFile
grass_texture_path = chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg")
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(grass_texture_path, 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional()

# Add point lights for better illumination
vis.AddLightPoint(chrono.ChVector3f(10, 10, 10), chrono.ChColor(1, 1, 1), 1000)
vis.AddLightPoint(chrono.ChVector3f(-10, 10, 10), chrono.ChColor(1, 1, 1), 1000)
vis.Initialize()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager and add camera
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.SetVerbose(False)

# Configure camera sensor
chassis = vehicle.GetChassis().GetBody()
camera = sens.ChCameraSensor(
    chassis,                  # Attach to chassis
    60.0,                     # Update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 1.2), chrono.QUNIT),  # Position and orientation
    1280,                     # Image width
    720,                      # Image height
    chrono.CH_PI / 3          # Field of view (60 degrees)
)
camera.SetName("First Person View")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))  # Add visualization filter
manager.AddSensor(camera)

# Initialize driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop setup
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Main simulation loop
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
    manager.Update()  # Ensure camera updates with vehicle movement

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Maintain real-time execution
    realtime_timer.Spin(step_size)