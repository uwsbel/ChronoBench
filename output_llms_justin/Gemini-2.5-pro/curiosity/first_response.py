import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# -----------------------------------------------------------------------------
# Set Chrono data directory
# -----------------------------------------------------------------------------
# It's good practice to set the data path for Chrono and Chrono::Vehicle
# This allows Chrono to find its data files (textures, meshes, etc.)
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
# Simulation step size
time_step = 0.005  # seconds

# End time
time_end = 100     # seconds

# Initial rover position and orientation
init_loc = chrono.ChVectorD(0, 0.5, 0)  # x, y, z position (y is up)
init_rot = chrono.QUNIT                # No initial rotation (identity quaternion)

# Visualization type for rover parts (MESH or PRIMITIVES or NONE)
chassis_vis_type = veh.VisualizationType_MESHES
wheel_vis_type = veh.VisualizationType_MESHES

# Collision type for rover parts (PRIMITIVES or MESHES or NONE)
# For performance, PRIMITIVES is often better for complex models like rovers
chassis_collision_type = veh.CollisionType_PRIMITIVES

# Point on chassis tracked by the camera
camera_track_point = chrono.ChVectorD(0.0, 0.0, 0.0)
camera_chase_dist = 6.0
camera_chase_height = 1.5

# -----------------------------------------------------------------------------
# Create the Chrono system and a contact material
# -----------------------------------------------------------------------------
# Create a Chrono physical system
my_system = chrono.ChSystemNSC() # Using Non-Smooth Contact (NSC) method

# Set gravity
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver settings
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # A good robust solver
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# Create the ground
# -----------------------------------------------------------------------------
# Create a contact material for the ground (shared by all patches)
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)
ground_mat.SetYoungModulus(2e7) # Stiffness for NSC contacts

# Create the rigid terrain
terrain = veh.RigidTerrain(my_system)

# Add a flat patch of terrain
# Parameters: material, center point, normal, half-dimensions (length, width)
patch_size_x = 200.0
patch_size_y = 200.0
patch = terrain.AddPatch(ground_mat,
                         chrono.ChVectorD(0, 0, 0),    # Center of the patch
                         chrono.ChVectorD(0, 1, 0),    # Normal vector (up)
                         patch_size_x, patch_size_y)   # Dimensions

# Set texture for the patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) # Texture file and scaling

# Set visualization properties for the patch (optional, but good for shadows)
patch.GetGroundBody().GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.6))

# Initialize the terrain
terrain.Initialize()

# -----------------------------------------------------------------------------
# Create the Curiosity rover
# -----------------------------------------------------------------------------
# Create the Curiosity rover using the rigid tire model
rover = veh.Curiosity_Rigid(my_system)

# Set rover chassis visualization and collision types
rover.SetChassisVisualizationType(chassis_vis_type)
rover.SetChassisCollide(True) # Enable chassis collision

# Set rover wheel visualization type
rover.SetWheelVisualizationType(wheel_vis_type)

# Initialize the rover at the specified position and orientation
rover.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# -----------------------------------------------------------------------------
# Create the Irrlicht visualization application
# -----------------------------------------------------------------------------
# Create the Irrlicht application
# Parameters: system, title, dimensions, vertical sync, use shadows, antialiasing
app = irr.ChIrrApp(my_system, "Curiosity Rover Simulation", irr.dimension2du(1280, 720), False, True, True)

# Set up the camera
app.SetChaseCamera(rover.GetChassisBody(), camera_track_point, camera_chase_dist, camera_chase_height)
app.SetCameraPosition(init_loc + chrono.ChVectorD(5,3,5)) # Initial camera position, will be overridden by chase cam
app.SetCameraTarget(init_loc)

# Add typical lights
app.AddTypicalLights()
# You can also add custom lights:
# app.AddLight(irr.ChIrrTools_CreateLight(app.GetSceneManager(),
#                                          app.GetVideoDriver(),
#                                          irr.vector3df(100,100,100), # position
#                                          irr.SColorf(0.8,0.8,0.8),   # color
#                                          200))                      # radius

# Add shadows for all objects that can cast/receive them
app.AddShadowAll()

# Add a skybox for a more immersive background
# Textures for the skybox (make sure these files exist in the Chrono data path)
sky_path = veh.GetDataFile("skybox/")
app.GetSceneManager().addSkyBoxSceneNode(
    irr.readImage(sky_path + "sky_up.jpg"),    # Up
    irr.readImage(sky_path + "sky_dn.jpg"),    # Down
    irr.readImage(sky_path + "sky_lf.jpg"),    # Left
    irr.readImage(sky_path + "sky_rt.jpg"),    # Right
    irr.readImage(sky_path + "sky_ft.jpg"),    # Front
    irr.readImage(sky_path + "sky_bk.jpg")     # Back
)

# Add a Chrono logo to the GUI
logo_path = veh.GetDataFile("chrono_logo.png")
if os.path.exists(logo_path):
    app.GetGUIEnvironment().addImage(
        app.GetVideoDriver().getTexture(logo_path),
        irr.position2d_s32(10, 10) # Top-left corner
    )
else:
    print(f"Warning: Logo file not found at {logo_path}")


# Bind all visual assets (meshes, textures) to the Irrlicht scene
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# Create the interactive driver system
# -----------------------------------------------------------------------------
# This driver allows control of the rover using keyboard inputs (W,S,A,D, etc.)
driver = veh.ChInteractiveDriverIRR(app)

# Set steering, throttle, and braking sensitivity
driver.SetSteeringDelta(0.04)  # Radians per key press
driver.SetThrottleDelta(0.1)   # [-1,1] range, change per key press
driver.SetBrakingDelta(0.2)    # [0,1] range, change per key press

# Initialize the driver
driver.Initialize()

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
# Set the time step for the Irrlicht application
app.SetTimestep(time_step)
app.SetTryRealtime(True) # Try to run in real-time

# Simulation loop
current_time = 0
while app.GetDevice().run():
    current_time = my_system.GetChTime()
    if current_time >= time_end:
        break

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from driver)
    driver.Synchronize(current_time)      # Synchronize driver (get keyboard inputs)
    rover.Synchronize(current_time, driver_inputs, terrain) # Synchronize rover with inputs and terrain
    terrain.Synchronize(current_time)     # Synchronize terrain (if it has any dynamic parts)

    # Advance simulation for one timestep for all modules
    driver.Advance(time_step)
    rover.Advance(time_step)
    terrain.Advance(time_step)
    my_system.DoStepDynamics(time_step) # Advance the Chrono system

    # Irrlicht visualization
    app.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) # Background color (blueish)
    app.DrawAll()

    # Print some info (optional)
    # irr.ChIrrTools_drawVehicleChassisReferenceFrame(rover.GetVehicle(), app.GetVideoDriver(), 1.0) # Draw chassis frame
    # irr.ChIrrTools_drawAllCOGs(my_system, app.GetVideoDriver(), 1.0) # Draw COGs for all bodies
    
    # Display rover speed
    speed = rover.GetChassisBody().GetFrame_REF_to_abs().GetPos_dt().Length()
    app.GetVideoDriver().draw2DContext() # Prepare for 2D drawing
    font = app.GetFont()
    if font:
         font.draw("Time: {:.2f} s".format(current_time), irr.rect_s32(10, 50, 200, 70), irr.SColor(255, 0, 0, 0))
         font.draw("Speed: {:.2f} m/s".format(speed), irr.rect_s32(10, 70, 200, 90), irr.SColor(255, 0, 0, 0))
         font.draw("Throttle: {:.2f}".format(driver_inputs.m_throttle), irr.rect_s32(10, 90, 200, 110), irr.SColor(255, 0, 0, 0))
         font.draw("Steering: {:.2f}".format(driver_inputs.m_steering), irr.rect_s32(10, 110, 200, 130), irr.SColor(255, 0, 0, 0))
         font.draw("Braking: {:.2f}".format(driver_inputs.m_braking), irr.rect_s32(10, 130, 200, 150), irr.SColor(255, 0, 0, 0))


    app.EndScene()

# -----------------------------------------------------------------------------
# Cleanup
# -----------------------------------------------------------------------------
# Deleting the app object is important for proper Irrlicht shutdown
del app
print("Simulation finished.")