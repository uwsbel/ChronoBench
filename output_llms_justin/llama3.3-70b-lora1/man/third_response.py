import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import pychrono.sensor as sens
import numpy as np

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

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

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
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

# Update rate in Hz for sensors (set to the simulation step size)
update_rate = 1/step_size

# =============================================================================

# Create the MAN vehicle, set parameters, and initialize
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
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# =============================================================================

# Create the sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
intensity = 1.0
cam_update_rate = 1 / 30  # FPS = 30
vis_camera = veh.CastVisualSystemIrrlicht(vis)

# Create a camera
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 3), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0).GetUnit()))
cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov)
cam.PushFilter(sens.ChFilterVisualize(intensity))
manager.AddSensor(cam)

noise_model=None
if len(sys.argv) > 1:
    if sys.argv[1] == "noise":
        # Add a simple lens distortion to the camera
        cam.PushFilter(sens.ChFilterLensDistortion(0.5, 0.5))
        # Have the camera produce asynchronous data.
        # Asynchronous data is sent to the filter manager as soon as it is available
        cam.SetAsynchronous(True)
        # Add a filter to access the camera's async data
        cam.PushFilter(sens.ChFilterAsyncAccess())
        # Create a filter to add random noise to the camera. This noise models
        # the real-world noise you might see in a camera's image
        # First provide the parameters for the noise model
        noise_model = sens.ChNoiseModelAndrea()
        noise_model.AccessNoise().SetGaussian(0.0, 0.02)
        noise_model.AccessNoise().SetSpeckle(0.01)
        # Push the noise filter
        cam.PushFilter(sens.ChFilterCameraNoise(noise_model))

manager.Initialize()

# =============================================================================

# Simulation loop

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)
# Update interval for characters
char_update_steps = int(1.0 / (0.025 * step_size))

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    driver_inputs = driver.GetInputs()

    # Update sensor manager
    # Update all sensors installed in the manager
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    manager.Advance(step_size)

    # Update modules (advance simulation)
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # End simulation
    if (vehicle.GetChassisBody().GetPos().x > 100) :
        break

# =============================================================================

print("Test finished. Goodbye!")