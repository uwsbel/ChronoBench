import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

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
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize the FEDA vehicle
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
    terrainLength, terrainWidth, 0.2)  # Added thickness parameter

patch.SetTexture(veh.GetDataFile("terrain/textures/grass1.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add point lights for better illumination
vis.AddLight(chrono.ChVector3d(2, 2, 5), 10, chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVector3d(-2, -2, 5), 10, chrono.ChColor(1, 1, 1))

# Create sensor manager
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetKeyframeSizeFromTimeStep(step_size)

# Create and configure camera sensor
chassis_body = vehicle.GetVehicle().GetChassisBody()
camera_offset = chrono.ChVector3d(1.5, 0, 1.0)  # Front-mounted camera
camera_pose = chrono.ChFrameD(camera_offset, chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))

camera = sens.ChCameraSensor(
    chassis_body,
    30,        # Update rate in Hz
    camera_pose,
    1280,      # Image width
    720,       # Image height
    60,        # Horizontal FOV (degrees)
    sens.CameraLensModelType_PINHOLE,
    0.1,       # Near clipping distance
    100.0      # Far clipping distance
)

camera.SetName("First Person Camera")
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "First Person View"))
camera.PushFilter(sens.ChFilterRGBA8Access())
sensor_manager.AddSensor(camera)

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set control input parameters
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop parameters
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

    # Get and process driver inputs
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

    # Update sensor manager
    sensor_manager.Update()

    step_number += 1
    realtime_timer.Spin(step_size)