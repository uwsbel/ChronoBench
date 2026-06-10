import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and set data paths
# ----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------------------
# Initial vehicle location and orientation
# ----------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (MESH, PRIMITIVES, NONE)
vis_type = veh.VisualizationType_MESH

# Collision/contact method
contact_method = chrono.ChContactMethod_NSC

# Type of tire model
tire_model = veh.TireModelType_TMEASY

# ----------------------------------------------------------------------------
# Rigid terrain parameters
# ----------------------------------------------------------------------------
terrainHeight = 0.0
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames (50 FPS)
render_step_size = 1.0 / 50.0

# ----------------------------------------------------------------------------
# 2. Create the ARTcar vehicle, set parameters, and initialize
# ----------------------------------------------------------------------------
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle subsystems
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# ----------------------------------------------------------------------------
# Create the rigid terrain with a custom texture
# ----------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)

# Apply a custom texture to the terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# ----------------------------------------------------------------------------
# 3. Create the Irrlicht visualization application
# ----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.5), 5.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ----------------------------------------------------------------------------
# Create the interactive driver system
# ----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # time to go from 0 to +1 (or -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ----------------------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------------------
# Number of simulation steps between two render frames
render_steps = math.ceil(render_step_size / step_size)

step_number = 0
render_frame = 0

vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render at the prescribed FPS
    if step_number % render_steps == 0:
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

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment the step number
    step_number += 1