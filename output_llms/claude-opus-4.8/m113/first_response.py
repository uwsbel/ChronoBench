import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# -------------------------------------------------------------------
# 1. Initialize the PyChrono environment and set data path
# -------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------------------------------------
# 2. Vehicle Initial Conditions
# -------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.8)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Contact method for the system (SMC = penalty-based smooth contact)
contact_method = chrono.ChContactMethod_SMC

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Type of powertrain/track shoe collision representation
chassis_collision_type = veh.CollisionType_NONE

# -------------------------------------------------------------------
# 3. Create and Initialize the M113 Vehicle
# -------------------------------------------------------------------
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)

# Set initial position
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

# Driveline / Powertrain options
vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_SIMPLE)

# Initialize the vehicle
vehicle.Initialize()

# Set visualization types for the various vehicle subsystems
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Get the underlying Chrono system
system = vehicle.GetSystem()

# -------------------------------------------------------------------
# 4. Create the Rigid Terrain
# -------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

# Define the terrain contact material with friction and restitution
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)       # friction coefficient
patch_mat.SetRestitution(0.01)   # restitution coefficient
patch_mat.SetYoungModulus(2e7)   # for SMC contact

# Add a flat terrain patch
terrain_length = 100.0
terrain_width = 100.0
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length, terrain_width
)

# Set terrain texture
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -------------------------------------------------------------------
# 5. Create the Driver System (interactive control)
# -------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())

# Driver response time settings
driver.SetSteeringDelta(1.0 / 50)
driver.SetThrottleDelta(1.0 / 50)
driver.SetBrakingDelta(3.0 / 50)

driver.Initialize()

# -------------------------------------------------------------------
# 6. Create the Irrlicht Visualization System
# -------------------------------------------------------------------
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Tracked Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())
vis.AttachDriver(driver)

# -------------------------------------------------------------------
# 7. Simulation Loop
# -------------------------------------------------------------------
# Timestep
step_size = 1e-3

# Real-time stepping helper
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get the current driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize subsystems for this timestep
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems by one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Maintain real-time execution
    realtime_timer.Spin(step_size)