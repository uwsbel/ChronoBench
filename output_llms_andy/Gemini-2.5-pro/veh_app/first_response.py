import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import os
import math

# ---------------------------------------------------------------------
# Set paths to Chrono data files
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
sens.SetSensorDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'sensor', ''))

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
# Simulation step size
step_size = 2e-3  # 2 ms

# Simulation end time
t_end = 100

# Output directories
out_dir = "GATOR_SENSOR_DEMO"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
if not os.path.exists(os.path.join(out_dir, "images")):
    os.makedirs(os.path.join(out_dir, "images"))


# ---------------------------------------------------------------------
# Create the Chrono system
# ---------------------------------------------------------------------
print("Creating Chrono system...")
# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set the solver type
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# ---------------------------------------------------------------------
# Create the Gator vehicle
# ---------------------------------------------------------------------
print("Creating Gator vehicle...")
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Create the Gator vehicle
gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC) # or SMC if system is ChSystemSMC
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(veh.TireModelType_TMEASY) # TMeasy tires
gator.SetTireStepSize(step_size)
gator.SetPowertrainType(veh.PowertrainModelType_SIMPLE) # Simple powertrain
gator.Initialize()

# Set visualization type for different parts
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH) # Wheels are often part of tire mesh
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle = gator.GetVehicle()
chassis_body = vehicle.GetChassisBody()

# ---------------------------------------------------------------------
# Create the rigid terrain
# ---------------------------------------------------------------------
print("Creating rigid terrain...")
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.1), chrono.QUNIT),  # slightly below vehicle
                         200.0, 200.0) # Length and width
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the Irrlicht visualization for the main simulation
# ---------------------------------------------------------------------
print("Creating Irrlicht visualization...")
# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Simulation with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5) # POV, dist, height
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AttachVehicle(vehicle)

# ---------------------------------------------------------------------
# Create the interactive driver system
# ---------------------------------------------------------------------
print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(vis)
# Set the time response for steering and throttle keyboard inputs.
# NOTE: this is not exact, since we do not render quite at the specified FPS.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)
driver.Initialize()
driver.SetVehicle(vehicle) # Link driver to vehicle

# ---------------------------------------------------------------------
# Create the Sensor Manager and Sensors
# ---------------------------------------------------------------------
print("Creating sensor manager and sensors...")
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetVerbose(False)

# Set sensor manager background
# (This background is for the sensor rendering, not the main Irrlicht visualization)
background = sens.Background()
background.mode = sens.BackgroundMode_ENVIRONMENT_MAP
background.env_tex = sens.GetDataFile("sensor/textures/sky_2_4k.hdr") # A common HDR for environment lighting
sensor_manager.SetBackground(background)

# --- Camera Sensor ---
# Define the offset of the camera from the chassis reference frame
# Camera looking forward, slightly up, from the front-center of the chassis
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.8, 0, 0.5), # x, y, z position relative to chassis
    chrono.Q_from_AngAxis(chrono.CH_PI / 20, chrono.ChVector3d(0, 1, 0)) # Small upward pitch
)
update_rate = 30 # Hz

camera = sens.ChCameraSensor(
    chassis_body,       # Body camera is attached to
    update_rate,        # Update rate in Hz
    cam_offset_pose,    # Offset pose
    1280,               # Image width
    720,                # Image height
    chrono.CH_PI / 3,   # Horizontal field of view
    0, # Use default supersampling factor for alias patterns
    sens.CameraLensModelType_PINHOLE, # Lens model
    False # Use rigid mounting (no lag)
)
camera.SetName("VehicleCamera")

# Add a filter to visualize the camera data
# This will open a separate window for the camera view
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View", False))
# Optional: Add a filter to save camera images
# camera.PushFilter(sens.ChFilterSave(os.path.join(out_dir, "images/")))
# Optional: Add a filter to access raw RGBA8 data (if you want to process it in Python)
# camera.PushFilter(sens.ChFilterRGBA8Access())

sensor_manager.AddSensor(camera)


# --- Point Lights for Sensor Scene ---
# These lights illuminate the scene for the *sensor* renderings (e.g., camera)
# They are distinct from the lights in the main Irrlicht visualization
light_intensity = 2.0 # Quite bright for HDR pipeline
light_radius = 10.0 # Radius of effect

# Light 1: Front-left of vehicle
light1_offset_pose = chrono.ChFramed(chrono.ChVector3d(2.0, 1.5, 1.0)) # Relative to chassis
point_light1 = sens.ChPointLightSensor(
    chassis_body,           # Body light is attached to
    100,                    # Update rate (high, effectively static relative to chassis)
    light1_offset_pose,     # Offset pose
    chrono.ChColor(light_intensity, light_intensity, light_intensity), # Color (white)
    light_radius            # Radius
)
point_light1.SetName("SensorPointLight1")
sensor_manager.AddSensor(point_light1)

# Light 2: Front-right of vehicle
light2_offset_pose = chrono.ChFramed(chrono.ChVector3d(2.0, -1.5, 1.0)) # Relative to chassis
point_light2 = sens.ChPointLightSensor(
    chassis_body,
    100,
    light2_offset_pose,
    chrono.ChColor(light_intensity, light_intensity, light_intensity),
    light_radius
)
point_light2.SetName("SensorPointLight2")
sensor_manager.AddSensor(point_light2)


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
print(f"Running simulation for {t_end} seconds...")
realtime_timer = chrono.ChRealtimeStepTimer()
realtime_timer.SetRealtimeStepUnscaled(1.0) # Attempt 1:1 realtime, will slow down if computation is heavy

# For data collection (optional)
# camera_data = []

while vis.Run():
    time = system.GetChTime()
    if time >= t_end:
        break

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (Synchronize)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    # Advance the system dynamics
    system.DoStepDynamics(step_size)

    # Update sensor manager
    # This will process sensor data, including rendering for the camera
    sensor_manager.Update()
    
    # Optional: Access sensor data
    # if camera.GetMostRecentRGBA8Buffer().HasData():
    #     img_data = camera.GetMostRecentRGBA8Buffer().GetRGBA8Data()
    #     # Process img_data (it's a flat list of R,G,B,A,R,G,B,A...)
    #     # For example, convert to a numpy array and reshape for OpenCV or PIL
    #     # import numpy as np
    #     # img_np = np.array(img_data, dtype=np.uint8).reshape((camera.GetHeight(), camera.GetWidth(), 4))

    # Irrlicht visualization
    vis.BeginScene()
    vis.Render()
    # Optional: Draw stats or other info
    # irr.DrawAllChronosObjects(system, vis.GetVideoDriver())
    vis.EndScene()
    
    # Try to run in real-time
    realtime_timer.Spin(step_size)

print("Simulation finished.")
# No explicit cleanup needed for vis or sensor_manager in Python, garbage collection handles it.