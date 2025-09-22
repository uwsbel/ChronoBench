import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# ---------------------------------------------------------------------
# Set PyChrono data paths
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'chrono_data', 'vehicle', ''))
# If the above veh.SetDataPath doesn't work because __file__ is not defined (e.g. in an interactive session)
# or if the relative path is incorrect, you might need to set it more explicitly based on your Chrono installation.
# A common alternative if CHRONO_DATA_DIR is set:
# veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
# Simulation step size
step_size = 1.0 / 50.0  # Corresponds to 50 FPS

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0.6, 0)  # x, y (height), z
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # Identity quaternion

# Tire model (TMEASY, PACEJKA, RIGID, FIALA)
tire_model = veh.TireModelType_TMEASY

# Camera parameters
camera_target_point = chrono.ChVectorD(0.0, 0.0, 1.75) # Point on the vehicle to track
camera_distance = 8.0  # Distance from the target point
camera_height_offset = 1.0 # Height of the camera relative to the target point

# Terrain dimensions
terrain_height = 0.0
terrain_size_x = 200.0  # Length
terrain_size_y = 200.0  # Width

# ---------------------------------------------------------------------
# Create the Chrono system
# ---------------------------------------------------------------------
# MBD system (NSC: Non-Smooth Contact)
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) # Set gravity

# Set solver settings if needed (defaults are often fine for demos)
# system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # Example solver
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# ---------------------------------------------------------------------
# Create the CityBus vehicle
# ---------------------------------------------------------------------
# Create the CityBus vehicle, specifying the ChSystem
bus = veh.CityBus(system)

# Initialize the vehicle at the specified position and orientation
bus.Initialize(chrono.ChCoordsysD(initLoc, initRot))

# Set visualization types for different parts
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH) # Wheels are typically meshes
bus.SetTireVisualizationType(veh.VisualizationType_MESH) # Tires are typically meshes

# Set the tire model
bus.SetTireType(tire_model)

# ---------------------------------------------------------------------
# Create the rigid terrain
# ---------------------------------------------------------------------
terrain = veh.ChRigidTerrain(system)

# Define contact material for the terrain
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
material.SetRestitution(0.01)
material.SetYoungModulus(2e7) # Optional: stiffness parameters
material.SetPoissonRatio(0.3) # Optional

# Create a flat patch of terrain
# The Y value for the patch center should be terrain_height - thickness / 2
patch_thickness = 0.2 # Give the patch some thickness for visualization
patch = terrain.AddPatch(material,
                         chrono.ChCoordsysD(chrono.ChVectorD(0, terrain_height - patch_thickness/2, 0), chrono.QUNIT), # Position and orientation
                         terrain_size_x, terrain_size_y, # Dimensions
                         patch_thickness, # Thickness
                         True, # Tiled UV mapping for texture
                         2.0, 2.0, # UV scaling factors
                         False, # No specific collision mesh (uses box primitive)
                         0.05) # Swept sphere radius for collision (if applicable)


# Set a custom texture for the terrain patch
texture_file = veh.GetDataFile("terrain/textures/tile4.jpg") # A common Chrono texture
patch.SetTexture(texture_file, (terrain_size_x / 10.0), (terrain_size_y / 10.0)) # Scale texture (e.g., repeat every 10m)

# Color for the patch if texture is not visible or for debugging
patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# Initialize the terrain
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the Irrlicht visualization application
# ---------------------------------------------------------------------
# Create the Irrlicht application
# The ChVehicleIrrApp automatically manages the Irrlicht rendering,
# camera, and event handling.
app = veh.ChVehicleIrrApp(bus, "CityBus on Rigid Terrain Simulation", chrono.dimension2du(1280, 720))

# Set up the chase camera
# Arguments: point on vehicle to track, distance from point, height of camera
app.SetChaseCamera(camera_target_point, camera_distance, camera_height_offset)
app.SetTimestep(step_size) # Important for synchronizing Irrlicht rendering with simulation

# Bind all vehicle and terrain assets for visualization
app.AssetBindAll()
app.AssetUpdateAll()

# Add a skybox
# app.AddTypicalSky() # This uses default skybox
# Or use a custom skybox from Chrono data
irrlicht_path = os.environ.get('CHRONO_IRRLICHT_DATA_DIR', '../../../chrono_data/irrlicht/')
app.AddTypicalSky(irrlicht_path + "skybox/")


# Add some lights
app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
# app.AddLight(irr.vector3df(50, 100, 50), 150, irr.SColorf(0.6,0.6,0.6,1)) # Example additional light

# ---------------------------------------------------------------------
# Create the interactive driver system
# ---------------------------------------------------------------------
# This driver uses keyboard inputs (arrows for throttle/braking, A/D for steering)
driver = veh.ChInteractiveDriverIRR(app)

# Set the time response for steering, throttle and braking controls
driver.SetSteeringDelta(0.04)  # Max steering K_i * step_size
driver.SetThrottleDelta(0.04)  # Max throttle K_i * step_size
driver.SetBrakingDelta(0.1)    # Max braking K_i * step_size
driver.SetGains(2.0, 5.0, 5.0) # K_p, K_i, K_d for PID (primarily K_i is used here)

# Initialize the driver system
driver.Initialize()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
# Number of simulation steps between miscellaneous events (printing data, etc.)
# render_steps = math.ceil(1.0 / (fps_viz * step_size)) # If you want different visualization FPS

# Simulation loop
time = 0.0

print("\nPyChrono simulation started. Controls:")
print("Steering:  A/D or Left/Right Arrow Keys")
print("Throttle:  W or Up Arrow Key")
print("Braking:   S or Down Arrow Key")
print("Camera:    Mouse or C (cycle modes), P (projection), T (tracking)")
print("           PageUp/PageDown (zoom), NumPad Arrows (pan/rotate)")
print("Quit:      ESC key\n")


while app.GetDevice().run():
    time = system.GetChTime()

    # Update the Irrlicht scene and draw all graphics
    app.BeginScene(True, True, chrono.ChColorToIrrColor(chrono.ChColor(0.3, 0.3, 0.4))) # Background color
    app.DrawAll() # Draws all visualizable items in the system

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) # For HUD display

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)
    app.Advance(step_size) # This also calls system.DoStepDynamics(step_size)

    # End the Irrlicht scene rendering
    app.EndScene()

    # Optional: Print simulation time or other info
    # if int(time / step_size) % 100 == 0: # Print every 100 steps
    #     print(f"Time: {time:.2f} s, "
    #           f"Throttle: {driver_inputs.m_throttle:.2f}, "
    #           f"Steering: {driver_inputs.m_steering:.2f}, "
    #           f"Braking: {driver_inputs.m_braking:.2f}")

# Cleanup (not strictly necessary for Irrlicht app, as it handles its own device closure)
# However, if you had other resources, you'd clean them up here.
print("Simulation ended.")