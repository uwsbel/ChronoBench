import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import os
import math

def main():
    # 0. Set paths
    # Get the Chrono data directory from environment variable
    CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR')
    if not CHRONO_DATA_DIR:
        print("Error: CHRONO_DATA_DIR environment variable not set.")
        print("Please set this variable to the location of your Chrono data files.")
        return
    
    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    # Set the path to the vehicle JSON files (relative to Chrono data directory)
    vehicle.SetDataPath(os.path.join(CHRONO_DATA_DIR, 'vehicle', ''))

    # 1. Simulation parameters
    step_size = 0.005  # Physics step size (s)
    render_fps = 50
    render_step_size = 1.0 / render_fps
    simulation_duration = 120  # seconds

    # Initial vehicle state
    initLoc = chrono.ChVectorD(0, 0, 0.6) # Start near origin for simpler SCM setup
    initRot = chrono.Q_from_AngZ(0)      # No initial Z-axis rotation

    # SCM Terrain parameters
    scm_initial_length = 30.0  # X direction (m) of the SCM grid
    scm_initial_width = 10.0   # Y direction (m) of the SCM grid
    scm_resolution = 0.05    # Grid cell size (m)

    # Moving patch parameters (relative to vehicle chassis)
    patch_look_ahead = 2.0       # m, look-ahead distance for patch center from vehicle CoG
    patch_half_length = 7.0      # m, half-length of the active SCM patch (vehicle's X-dir)
    patch_half_width = 3.5       # m, half-width of the active SCM patch (vehicle's Y-dir)
    patch_interaction_depth = 0.3 # m, SCM interaction depth (SCM's Z-dir, for soil displacement capacity)

    # Custom Soil Parameters (example values, adjust as needed for desired soil behavior)
    bekker_Kphi = 0.2e6    # Bekker K_phi (Pa/m^(n-1)) or (N/m^(n+1)) depending on formulation
    bekker_Kc = 0.0        # Bekker K_c (Pa/m^n) or (N/m^(n+2))
    bekker_n = 1.1         # Bekker exponent n (-)
    mohr_cohesion = 0.0    # Mohr-Coulomb cohesion (Pa)
    mohr_friction = 30.0   # Mohr-Coulomb friction angle (degrees)
    janosi_shear_k = 0.01  # Janosi-Hanamoto shear parameter K_j (m)
    elastic_K = 4e7      # Soil elastic stiffness (Pa/m) or (N/m^3)
    damping_K = 1.5e5    # Soil damping coefficient (Pa*s/m) or (N*s/m^3)

    # Camera parameters for chase cam
    camera_distance = 8.0      # Distance from vehicle's tracked point
    camera_height_offset = 2.5 # Height above vehicle's tracked point for camera position
    camera_chase_point_local = chrono.ChVectorD(0.5, 0.0, 0.5) # Point on chassis (local coords) to look at

    # 2. PyChrono environment and core components
    # Create a Chrono physical system
    system = chrono.ChSystemNSC() # Use Non-Smooth Contact method
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81)) # Set gravitational acceleration
    
    # Configure solver for better SCM performance (many frictional contacts)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetSolverForceTolerance(1e-4) # Lower tolerance for SCM can be beneficial

    # 3. Add HMMWV vehicle
    # Create the HMMWV vehicle system
    hmmwv = vehicle.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC) # Match system contact method
    hmmwv.SetChassisFixed(False) # Chassis is not fixed to ground
    hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot)) # Set initial position and orientation
    hmmwv.SetPowertrainType(vehicle.PowertrainModelType_SHAFTS) # Use a powertrain model with shafts
    hmmwv.SetDriveType(vehicle.DrivelineTypeWV_AWD) # All-Wheel Drive
    hmmwv.SetTireType(vehicle.TireModelType_RIGID) # Use RIGID tire model as specified

    # Apply MESH visualization to all major vehicle components
    hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
    
    hmmwv.Initialize() # Initialize the HMMWV vehicle
    
    # Get the chassis body for camera tracking and SCM patch following
    chassis_body = hmmwv.GetChassisBody()

    # 4. Add SCM Deformable Terrain
    # Create the SCM deformable terrain system
    terrain = vehicle.SCMDeformableTerrain(system)
    
    # Set SCM plane (optional, default is Z=0 in world frame, SCM's Z-up orientation)
    # The SCM terrain's internal height field is along its Y-axis.
    # The SCMDeformableTerrain class handles the transformation to Chrono's Z-up convention if needed.
    # By default, it aligns SCM's XY plane with Chrono's XY plane, with SCM's "height" (its local Y) along Chrono's Z.
    # terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT)) # Default is suitable

    # Set custom soil parameters
    terrain.SetSoilParameters(bekker_Kphi, bekker_Kc, bekker_n,
                              mohr_cohesion, mohr_friction, janosi_shear_k,
                              elastic_K, damping_K)

    # Initialize the SCM grid.
    # This defines the overall area and resolution of the SCM terrain.
    # The SCMDeformableTerrain assumes its local Y-axis is the "height" direction.
    # It's constructed in its own reference frame's XY plane.
    # For SCMDeformableTerrain, Initialize(length_x, width_y, grid_spacing_xy)
    terrain.Initialize(scm_initial_length, scm_initial_width, scm_resolution)
    
    # Enable moving patch feature, dynamically following the vehicle chassis
    terrain.EnableMovingPatch(True) 
    terrain.SetMovingPatchSettings(chassis_body,                             # Body to follow
                                   chrono.ChVectorD(patch_look_ahead, 0, 0), # Look-ahead point (local to chassis)
                                   chrono.ChVectorD(patch_half_length, patch_half_width, patch_interaction_depth)) # Half-dimensions of active patch

    # Visualize SCM sinkage with false color plotting
    # Arguments: plot_type, min_value_for_colormap, max_value_for_colormap
    terrain.SetPlotType(vehicle.SCMDeformableTerrain.PLOT_SINKAGE, 0.0, 0.2) # Color scale for 0 to 0.2m sinkage
    
    # Optional: Set texture for undeformed SCM terrain. Useful if parts remain undeformed.
    # terrain.SetTexture(chrono.GetChronoDataFile("sensor/textures/grass_texture.jpg"), 160, 120) # (file, scaleX, scaleY)


    # 5. Set up Irrlicht for visualization
    # Create the Irrlicht application
    app = irr.ChIrrApp(system, "HMMWV on SCM Deformable Terrain", irr.dimension2du(1280, 720))
    app.SetTimestep(step_size) # Link Irrlicht's internal timer to physics step for consistency

    # Add standard Irrlicht assets (skybox, lights)
    app.AddTypicalSky()
    app.AddTypicalLights()
    
    # Configure camera: manual update for chase-cam behavior
    # Initial camera position (will be updated dynamically in the simulation loop)
    app.GetCamera().SetPosition(irr.vector3df(initLoc.x + 10, initLoc.y + 5, initLoc.z + 5))
    app.GetCamera().SetTarget(irr.vector3df(initLoc.x, initLoc.y, initLoc.z))
    
    # Bind all Chrono assets (vehicle, terrain) to Irrlicht visualization objects
    # This process creates corresponding visual shapes in Irrlicht.
    app.AssetBindAll()
    app.AssetUpdateAll()

    # 6. Set up interactive driver system
    # Create an interactive driver for Irrlicht
    driver = vehicle.ChInteractiveDriverIRR(app)
    driver.SetSteeringDelta(0.04) # Steering input sensitivity
    driver.SetThrottleDelta(0.08) # Throttle input sensitivity
    driver.SetBrakingDelta(0.10)  # Braking input sensitivity
    driver.Initialize()
    hmmwv.SetDriver(driver) # Link the driver to the HMMWV vehicle

    # Custom event receiver for ESC key to quit the simulation
    class MyEventReceiver(irr.IEventReceiver):
        def __init__(self, app_ref):
            super().__init__()
            self.app_ref = app_ref # Reference to the Irrlicht application
            self.quit_simulation = False
        def OnEvent(self, event):
            # Check if the event is a key press
            if event.EventType == irr.EET_KEY_INPUT_EVENT and \
               not event.KeyInput.PressedDown and \
               event.KeyInput.Key == irr.KEY_ESCAPE: # Check for ESC key release
                print("Escape key pressed. Exiting simulation.")
                self.quit_simulation = True
                # Close the Irrlicht device to terminate the app.GetDevice().run() loop
                self.app_ref.GetDevice().closeDevice() 
                return True # Event was handled
            return False # Event was not handled by this receiver

    event_receiver = MyEventReceiver(app)
    app.SetUserEventReceiver(event_receiver) # Register the custom event receiver

    # 7. Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer() # Timer for real-time simulation
    current_render_time = 0.0

    print("\nINSTRUCTIONS:")
    print("Press UP/DOWN arrow keys for throttle/brake.")
    print("Press LEFT/RIGHT arrow keys for steering.")
    print("Press ESC to quit the simulation.\n")

    # Main simulation loop. Continues as long as Irrlicht window is open and ESC not pressed.
    while app.GetDevice().run() and not event_receiver.quit_simulation:
        time = system.GetChTime() # Get current simulation time

        # Stop simulation if duration is reached
        if time >= simulation_duration:
            print(f"Simulation ended after {simulation_duration} seconds.")
            break

        # Update camera position to follow the vehicle (chase camera)
        if chassis_body:
            ch_pos = chassis_body.GetPos() # Chassis position (world frame)
            ch_rot = chassis_body.GetRot() # Chassis orientation (world frame)
            
            # Point on chassis to be tracked by camera (world frame)
            cam_target_global = ch_pos + ch_rot.Rotate(camera_chase_point_local)
            
            # Calculate camera position:
            # Start from the tracked point, move back along vehicle's X-axis, then up.
            # Vehicle's local X-axis direction in world frame
            dir_forward_vehicle = ch_rot.GetXaxis() 
            # A nominal "up" vector for camera positioning (can be world Z or vehicle Z)
            dir_up_world = chrono.ChVectorD(0, 0, 1) 
            
            cam_pos_global = cam_target_global - dir_forward_vehicle * camera_distance + dir_up_world * camera_height_offset

            # Set Irrlicht camera position and target
            app.GetCamera().SetPosition(irr.vector3df(cam_pos_global.x, cam_pos_global.y, cam_pos_global.z))
            app.GetCamera().SetTarget(irr.vector3df(cam_target_global.x, cam_target_global.y, cam_target_global.z))


        # Render scene only at the specified rendering FPS
        if time >= current_render_time:
            app.BeginScene(
print("error happened with only start ```python")