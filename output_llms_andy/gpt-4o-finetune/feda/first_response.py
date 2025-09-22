import pychrono.core as chrono  # Import core Chrono functionality
import pychrono.irrlicht as irr  # Import Irrlicht for visualization
import pychrono.vehicle as veh  # Import vehicle modules from Chrono
import pychrono.sensor as sens  # Import sensor modules (not used in this script)
import math  # Import math for any mathematical operations

# Set the Chrono data path for default vehicle models and textures
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)  # Set initial location (x, y, z)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # Set initial rotation as a quaternion (w, x, y, z)

# Visualization type for vehicle parts (primitives, meshes, or none)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (none, primitives, or meshes)
chassis_collision_type = veh.CollisionType_NONE

# Rigid terrain patch
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0  # Set terrain height
terrainLength = 100.0  # Set terrain length
terrainWidth = 100.0  # Set terrain width

# Point on ground tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3  # Define the simulation step size
tire_step_size = step_size  # Define the tire step size (same as simulation step size)

# Time interval between two render frames
render_step_size = 1.0 / 50  # Set rendering frame rate to 50 FPS

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()  # Instantiate the FEDA vehicle
vehicle.SetContactMethod(contact_method)  # Set the contact method for physics interactions
vehicle.SetTrackType(veh.FEDA_SCG)  # Set the track type for the vehicle
vehicle.SetTireType(veh.TireType_BlackMichelin)  # Specify the tire model
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)  # Define the engine model to use
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # Set the transmission model

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))  # Initialize the vehicle's position and orientation
vehicle.Initialize()  # Initialize vehicle systems with the specified configurations

vehicle.SetChassisVisualizationType(vis_type)  # Set visualization type for the chassis
vehicle.SetSuspensionVisualizationType(vis_type)  # Set visualization type for the suspension
vehicle.SetSteeringVisualizationType(vis_type)  # Set visualization type for the steering mechanism
vehicle.SetWheelVisualizationType(vis_type)  # Set visualization type for the wheels
vehicle.SetTireVisualizationType(vis_type)  # Set visualization type for the tires

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Use the Bullet collision system

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()  # Define material properties for the terrain
patch_mat.SetFriction(0.9)  # Set friction coefficient
patch_mat.SetRestitution(0.01)  # Set restitution (bounciness)
terrain = veh.RigidTerrain(vehicle.GetSystem())  # Create a rigid terrain in the simulation
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # Apply texture to the terrain
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))  # Set the color of the terrain patch
terrain.Initialize()  # Initialize the terrain

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()  # Create Irrlicht visualization system
vis.SetWindowTitle('FEDA vehicle')  # Set the window title
vis.SetWindowSize(1280, 720)  # Set the window size
vis.SetChaseCamera(trackPoint, 6.0, 0.5)  # Configure the chase camera
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo
vis.AddLightDirectional()  # Add directional light
vis.AddSkyBox()  # Add a skybox for better visuals
vis.AttachVehicle(vehicle.GetVehicle())  # Attach the vehicle to the visualization system

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)  # Create an interactive driver system
driver.SetSteeringDelta(0.02)  # Set steering sensitivity
driver.SetThrottleDelta(0.02)  # Set throttle sensitivity
driver.SetBrakingDelta(0.06)  # Set braking sensitivity
driver.Initialize()  # Initialize the driver system

# Output some vehicle parameters for reference
print("Vehicle mass:     ", vehicle.GetVehicle().GetMass())
print("Driveline type:   ", vehicle.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:       ", vehicle.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:        ", vehicle.GetTire(1, veh.LEFT).GetTemplateName())
print("Tire step size:   ", tire_step_size)

# Set the simulation time step
vehicle.GetVehicle().SetStep(step_size)

# Initialize time and simulation frame counter s
time = 0
Step = 0

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()  # Update simulation time

    # Render the scene and output data
    if (Step % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()  # Begin the scene rendering
        vis.Render()  # Render the scene
        vis.EndScene()  # End the scene rendering

    # Get driver inputs and synchronize the systems
    driver_inputs = driver.GetInputs()  # Retrieve inputs from the driver
    driver.Synchronize(time)  # Synchronize the driver system
    terrain.Synchronize(time)  # Synchronize the terrain
    vehicle.Synchronize(time, driver_inputs, terrain)  # Synchronize the vehicle with inputs and terrain

    # Advance the simulation for one timestep
    driver.Advance(step_size)  # Advance the driver system
    terrain.Advance(step_size)  # Advance the terrain system
    vehicle.Advance(step_size)  # Advance the vehicle system

    # Increment the simulation step counter
    Step = Step + 1