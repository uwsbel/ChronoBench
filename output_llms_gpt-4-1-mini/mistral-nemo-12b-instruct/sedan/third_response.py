import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVector3d(0, -5, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (MESH)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain with highway mesh
terrain_model = veh.RigidTerrain.HIGHWAY
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method and visualization
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes (decreased for finer control)
step_size = 1e-4
tire_step_size = step_size

# Time interval between two render frames (decreased for finer control)
render_step_size = 1.0 / 100  # FPS = 100

# Steering response time (increased to 5 seconds)
steering_time = 5.0

# --------------
# Create systems
# --------------

# Create the Sedan vehicle, set parameters, and initialize
vehicle = veh.BMW_E90()
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

# Create the terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system with reference speed input and PID controller
driver = veh.ChInteractiveDriverIRR(vis)
reference_speed = 20.0  # Set your desired reference speed here (in m/s)

# Set the time response for steering and throttle keyboard inputs.
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.SetSpeedReference(reference_speed)
driver.SetThrottlePID(0.5, 0.0, 0.1)  # P, I, D gains for throttle control

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
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

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)