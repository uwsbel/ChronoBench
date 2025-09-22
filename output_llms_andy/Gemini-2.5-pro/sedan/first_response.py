import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# -----------------------------------------------------------------------------
# Set Chrono data directory
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(veh.GetDataFile("")) # Essential for loading vehicle/terrain data

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Simulation step size
step_size = 2e-3  # 2 milliseconds

# Initial vehicle position and orientation
initLoc = chrono.ChVectorD(0, 0.7, 0)  # x, y (height), z
initRot = chrono.Q_from_AngZ(0)      # Initial rotation (radians around Z)

# Terrain dimensions
terrain_length = 200.0  # x-dimension
terrain_width = 200.0   # z-dimension

# Texture for the terrain (ensure this file exists in the Chrono data path)
# You can replace this with your custom texture file.
# If your texture includes logos, they will appear.
# Example: "terrain/textures/concrete.jpg", "terrain/textures/grass.jpg"
terrain_texture_file = veh.GetDataFile("terrain/textures/tile4.jpg")
logo_texture_file = veh.GetDataFile("chrono_logo.png") # A separate logo example if needed
terrain_texture_repeat_x = 20  # How many times the texture repeats in X
terrain_texture_repeat_y = 20  # How many times the texture repeats in Y

# Visualization type for vehicle components
# Options: veh.VisualizationType_PRIMITIVES, veh.VisualizationType_MESH, veh.VisualizationType_NONE
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH # Meshes for wheels look best
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis
# Options: True or False
chassis_collide = True

# Tire model
tire_model = veh.TireModelType_TMEASY

# -----------------------------------------------------------------------------
# Create the Chrono system and Irrlicht application
# -----------------------------------------------------------------------------
print("Creating Chrono system...")
# Create a Chrono physical system
sys = chrono.ChSystemNSC() # Using Non-Smooth Contact (NSC) method
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) # Set gravitational acceleration

# Create the Irrlicht visualization system
print("Creating Irrlicht application...")
application = irr.ChIrrApp(sys, "BMW E90 Sedan on Rigid Terrain", irr.dimension2du(1280, 720))
application.SetTimestep(step_size)
application.SetTryRealtime(True) # Attempt to run in real-time

# Add typical Irrlicht components
application.AddTypicalSky()
application.AddTypicalLights( # Add a directional light
    chrono.ChVectorD(50, 100, 50), # light direction
    chrono.ChVectorD(-50, -100, -50), # light aim point (for shadow map)
    120, 120, # ortho box size
    250, # intensity
    irr.SColorf(0.8,0.8,0.9) # color
)
# Or simpler default lights:
# application.AddTypicalLights()

# -----------------------------------------------------------------------------
# Create the Vehicle: BMW E90 Sedan
# -----------------------------------------------------------------------------
print("Creating BMW E90 Sedan vehicle...")
# Create the Sedan vehicle object
# It will be associated with the ChSystem 'sys'
sedan = veh.Sedan(sys)

# Set initial position and orientation
sedan.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))

# Set the powertrain model (SHAFTS is a common choice for detailed simulation)
sedan.SetPowertrainType(veh.PowertrainModelType_SHAFTS)

# Set the drive type (Rear-Wheel Drive for typical E90)
sedan.SetDriveType(veh.DrivelineTypeWV_RWD)

# Set the tire model (TMEASY as requested)
sedan.SetTireType(tire_model)

# --- Vehicle Configuration ---
# Visualization
sedan.SetChassisVisualizationType(chassis_vis_type)
sedan.SetSuspensionVisualizationType(suspension_vis_type)
sedan.SetSteeringVisualizationType(steering_vis_type)
sedan.SetWheelVisualizationType(wheel_vis_type)
sedan.SetTireVisualizationType(tire_vis_type)

# Collision
sedan.SetChassisCollide(chassis_collide)

# Initialize the vehicle system
# This creates the actual bodies and constraints
sedan.Initialize()

# Get the chassis body for camera tracking
chassis_body = sedan.GetChassisBody()

# -----------------------------------------------------------------------------
# Create the Terrain
# -----------------------------------------------------------------------------
print("Creating rigid terrain...")
terrain = veh.RigidTerrain(sys)

# Define the contact material for the terrain
# This material will be shared by all patches
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
material.SetRestitution(0.01)
material.SetYoungModulus(2e7) # Stiffness
material.SetPoissonRatio(0.3)

# Add a flat terrain patch
# Arguments: material, center_point, normal_vector, length (x), width (z), height (thickness, optional)
patch = terrain.AddPatch(material,
                         chrono.ChVectorD(0, 0, 0),      # Center of the patch
                         chrono.ChVectorD(0, 1, 0),      # Upward normal
                         terrain_length, terrain_width)

# Set texture for the terrain patch
patch.SetTexture(terrain_texture_file, terrain_texture_repeat_x, terrain_texture_repeat_y)

# Optional: Add a small patch/body with a logo on the terrain
# This is one way to place a distinct logo.
# Alternatively, your main terrain_texture_file could already contain logos.
if os.path.exists(logo_texture_file):
    logo_size = 5.0 # meters
    logo_patch = terrain.AddPatch(material,
                                  chrono.ChVectorD(10, 0.01, 10), # Position it slightly above ground
                                  chrono.ChVectorD(0, 1, 0),
                                  logo_size, logo_size)
    logo_patch.SetTexture(logo_texture_file, 1, 1) # No repeat for a single logo
    # You can also make it a color:
    # logo_patch.SetColor(chrono.ChColor(0.8, 0.2, 0.2))


# Initialize the terrain
terrain.Initialize()

# -----------------------------------------------------------------------------
# Create the Interactive Driver System
# -----------------------------------------------------------------------------
print("Creating interactive driver...")
# Create the interactive driver system using Irrlicht GUI
driver = veh.ChIrrGuiDriver(application)

# Set the controls for the driver
# delta values determine how much each key press changes the value
driver.SetSteeringDelta(0.04)  # Radians per key press for steering
driver.SetThrottleDelta(0.04)  # Percent per key press for throttle
driver.SetBrakingDelta(0.10)   # Percent per key press for braking

# Set input mode (e.g., steering with A/D, throttle with W/S)
driver.SetInputMode(veh.ChIrrGuiDriver.InputMode_रानीपुरSTEERING) # Default and suitable

# Link the driver to the vehicle
# sedan.SetDriver(driver) # This line seems redundant if Initialize comes after
driver.Initialize() # Initialize the driver system

# -----------------------------------------------------------------------------
# Configure Irrlicht Visualization
# -----------------------------------------------------------------------------
# Attach the vehicle to the Irrlicht application for rendering
# This binds the vehicle's assets to the Irrlicht scene
application.AssetBindAll()
application.AssetUpdateAll()

# --- Set up Chase Camera ---
# Arguments: chassis_body, chase_distance (m), chase_height (m)
application.SetChaseCamera(chassis_body, 6.0, 0.5)
# Set camera vertical direction (Y is up in Chrono, typically)
application.SetCameraVertical(chrono.VerticalDir_Y)

# Optional: Display HUD elements
application.SetHUDDisplay(True) # Show basic HUD
application.SetVehicleSpeedUnit(irr.ChIrrApp.ASCII_KMH) # Display speed in km/h
application.SetShowVehicleStatus(True) # Show throttle/steering/braking levels


# -----------------------------------------------------------------------------
# Simulation Loop
# -----------------------------------------------------------------------------
print("\n--- Simulation Instructions ---")
print("Steering:     A/D keys")
print("Throttle:     W key")
print("Braking:      S key")
print("Camera Zoom:  Mouse Scroll Wheel")
print("Camera Rotate:Hold Left Mouse Button and Drag")
print("-----------------------------\n")


# Simulation loop
time = 0.0
while application.GetDevice().run():
    application.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) # Clear color
    application.DrawAll()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize systems (vehicle, terrain)
    # For rigid terrain, synchronize might not do much but good practice
    terrain.Synchronize(time)
    sedan.Synchronize(time, driver_inputs, terrain) # Pass driver inputs and terrain to vehicle
    driver.Synchronize(time) # Synchronize driver time

    # Advance simulation
    sys.DoStepDynamics(step_size)
    application.DoStep() # Advance Irrlicht's internal time (for GUI, real-time sync)

    application.EndScene()

    # Update time
    time += step_size

print("Simulation finished.")