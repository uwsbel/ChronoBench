import pychrono as chrono  # Import the core Chrono library
import pychrono.vehicle as veh  # Import the vehicle module of Chrono
import pychrono.irrlicht as irr  # Import the Irrlicht visualization library for Chrono
import math  # Import the math library for mathematical functions

# Set the Chrono data path (this line is commented out, likely because the path is set elsewhere or not needed)
# chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Create the ARTcar vehicle and set its parameters
# Vehicle initial location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)  # Initial position (x, y, z)
initRot = chrono.QuatFromAngleZ(0)  # Initial rotation (no rotation)

vehicle = veh.ARTcar()  # Create an instance of the ARTcar vehicle
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)  # Set the contact method to non-smooth contact (NSC)
vehicle.SetChassisFix(false)  # Ensure the chassis is not fixed
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))  # Set the initial position and rotation of the vehicle
vehicle.SetShaftTorque(1.5)  # Set the torque applied to the vehicle's shaft
vehicle.SetSteering(0.1)  # Set the steering angle
vehicle.Initialize()  # Initialize the vehicle with the specified parameters

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)  # Set the visualization type for the chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_NONE)  # Set the visualization type for the suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_NONE)  # Set the visualization type for the steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)  # Set the visualization type for the wheels

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Set the collision system type to BULLET

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()  # Create a contact material for the terrain
patch_mat.SetFriction(0.9)  # Set the friction for the terrain
patch_mat.SetRestitution(0.01)  # Set the restitution (bounciness) for the terrain

terrain = veh.RigidTerrain(vehicle.GetSystem())  # Create a rigid terrain in the vehicle's system
# Add a patch to the terrain with specified dimensions and position
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 50, 50)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)  # Set the texture for the patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))  # Set the color of the patch
terrain.Initialize()  # Initialize the terrain

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()  # Create the Irrlicht visualization system for wheeled vehicles
vis.SetWindowTitle('dart')  # Set the title of the visualization window
vis.SetWindowSize(1280, 720)  # Set the size of the visualization window
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)  # Set the chase camera parameters
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddLightDirection()  # Add directional light to the scene
vis.AddSkyBox()  # Add a skybox to the scene
vis.AttachVehicle(vehicle.GetVehicle())  # Attach the vehicle to the visualization system

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)  # Create an interactive driver system using Irrlicht
# Set the time response for steering, throttle, and braking inputs
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.SetRenderSteering(True)  # Enable rendering of the steering input

driver.Initialize()  # Initialize the driver system

# Set the simulation step sizes
step_size = 1e-3  # Simulation step size
tire_step_size = step_size  # Tire step size (same as simulation step size)

# Initialize times and counters for managing the simulation loop
time = 0
realtime_timer = chrono.ChRealtimeStepTimer()  # Create a real-time step timer
render_step_size = 1 / 50  # Define the step size for rendering (50 FPS)
render_frame = 0  # Initialize render frame counter

while vis.Run():  # Main simulation loop
    time = vehicle.GetSystem().GetChTime()  # Get the current simulation time

    # Render the scene at specified render steps
    if time >= render_frame * render_step_size:
        vis.BeginScene()  # Begin the scene rendering
        vis.Render()  # Render the scene
        vis.EndScene()  # End the scene rendering
        render_frame += 1  # Increment the render frame counter

    # Get driver inputs
    driver_inputs = driver.GetInputs()  # Retrieve inputs from the driver

    # Update the vehicle dynamics, driver, and visualization
    driver.Synchronize(time)  # Synchronize the driver with the current time
    terrain.Synchronize(time)  # Synchronize the terrain with the current time
    vehicle.Synchronize(time, driver_inputs, terrain)  # Synchronize the vehicle with inputs and terrain
    vis.Synchronize(time, driver_inputs)  # Synchronize the visualization with inputs

    driver.Advance(step_size)  # Advance the driver simulation by one step
    terrain.Advance(step_size)  # Advance the terrain simulation by one step
    vehicle.Advance(step_size)  # Advance the vehicle simulation by one step

    # Increment the simulation frame counter
    nshots += 1

    # Ensure the simulation real-time step
    realtime_timer.Spin(step_size)