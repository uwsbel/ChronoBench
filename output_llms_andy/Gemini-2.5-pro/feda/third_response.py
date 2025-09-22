import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens # Added for sensor manager and camera
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
# For sensor assets, if any are directly loaded (e.g. custom meshes for sensors)
# sens.SetSensorDataPath(chrono.GetChronoDataPath() + 'sensor/')


# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5) # The Z value might need adjustment depending on FEDA model's origin
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE # Chassis has no collision geometry

# Type of tire model (RIGID, TMEASY, PAC02, etc.)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (reference Z for patch definition)
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the chase camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) # Relative to vehicle chassis

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Not used in this script directly, but good to define

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames for Irrlicht
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle components
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set the collision system type for the entire simulation
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Add a patch of terrain
# The ChCoordsysd defines the position and orientation of the patch center
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.1), chrono.QUNIT), # Lowered slightly to avoid initial co-planar issues
    terrainLength, terrainWidth)

# 1. Terrain Texture Change: Change the terrain texture to a grass texture.
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200) # Changed from tile4.jpg
patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5)) # A greenish color for grass
terrain.Initialize()

# ---------------------------------------------------------------------
# Create a sensor manager and add lights/camera
# ---------------------------------------------------------------------
# 2. Sensor Manager and Light Additions:
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.SetVerbose(False) # Optional: reduce console output from sensor manager

# Add point lights to the sensor scene for better illumination of the FPV camera view
# Intensity is part of the ChColor components. Range affects falloff.
light_intensity = 2.0 # Adjust this value to make lights brighter or dimmer
light_color = chrono.ChColor(light_intensity, light_intensity, light_intensity)
light_range = 50.0 # Max range of the light

# Add a few point lights from different positions for the sensor scene
manager.AddPointLight(chrono.ChVector3d(10, 10, 10), light_color, light_range)
manager.AddPointLight(chrono.ChVector3d(-10, 10, 10), light_color, light_range)
manager.AddPointLight(chrono.ChVector3d(0, -10, 10), chrono.ChColor(light_intensity*0.7, light_intensity*0.7, light_intensity*0.7), light_range)


# 3. Camera Sensor Addition:
# Add a camera sensor to the vehicle's chassis for FPV
cam_update_rate = 30  # FPS for the camera sensor
cam_width = 1280      # High resolution width
cam_height = 720      # High resolution height
cam_fov_degrees = 75  # Field Of View in degrees

# Define the camera's position and orientation relative to the chassis body
# For FEDA, chassis origin is typically at front axle, mid-track, on ground.
# So, place camera in a typical driver's head position.
cam_pos_relative = chrono.ChVector3d(0.5, 0.0, 1.2)  # X:forward, Y:left, Z:up from chassis origin
cam_rot_relative = chrono.QUNIT # Looking straight ahead relative to chassis
fpv_camera_offset_pose = chrono.ChFrameD(cam_pos_relative, cam_rot_relative)

camera_fpv = sens.ChCameraSensor(
    vehicle.GetChassisBody(),       # Parent body (vehicle chassis)
    cam_update_rate,                # Update rate in Hz
    fpv_camera_offset_pose,         # Offset pose relative to parent
    cam_width,                      # Image width (pixels)
    cam_height,                     # Image height (pixels)
    chrono.CH_PI * cam_fov_degrees / 180.0,  # Horizontal FOV (radians)
    # sens.CameraLensModelType_PINHOLE, # Default lens model
    # False                           # Use global illumination (default: False)
)
camera_fpv.SetName("FPVCameraSensor")

# Add a visualization filter to this camera to display its output
# This will open a new window for the camera feed.
camera_fpv.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "FPV Camera View"))

# Add the FPV camera to the sensor manager
manager.AddSensor(camera_fpv)


# Create the vehicle Irrlicht interface (for main simulation view)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle with FPV Camera')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) # Chase camera parameters
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional() # Main directional light for Irrlicht scene
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


# Create the driver system (interactive)
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between Irrlicht render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0 # Not strictly used beyond an example, but good for tracking render frames

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render Irrlicht scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # render_frame += 1 # Increment if used

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs) # Synchronize Irrlicht visualization

    # 4. Sensor Manager Update in Simulation Loop:
    # Update the sensor manager (this acquires data from all sensors)
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) # Advance Irrlicht visualization

    # Increment step number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Cleanup (though Python's GC handles most of it, explicit deletion can be good in C++)
# For PyChrono, this is less critical unless managing large non-Chrono resources.