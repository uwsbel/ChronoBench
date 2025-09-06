import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # =============================================================================
    # 1. Initialize Simulation Environment
    # =============================================================================
    # Create physical system
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    
    # Set global collision parameters
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    
    # =============================================================================
    # 2. Create Rigid Terrain
    # =============================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.8)  # High friction for vehicle traction
    patch_mat.SetRestitution(0.01)  # Low restitution for hard surfaces
    
    # Create terrain patch
    terrain.AddPatch(patch_mat, 
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                    200, 100)  # 200x100 meter patch
    terrain.Initialize()

    # =============================================================================
    # 3. Initialize M113 Vehicle
    # =============================================================================
    # Set vehicle parameters
    init_pos = chrono.ChVector3d(0, 0, 1.0)  # Start 1m above ground
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation
    
    # Create and configure vehicle
    m113 = veh.M113(system)
    m113.SetContactMethod(chrono.ChContactMethod_SMC)
    m113.SetChassisFixed(False)
    m113.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
    m113.Initialize()
    
    # Set vehicle visualization mode
    m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetRoadWheelAssemblyVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)

    # =============================================================================
    # 4. Initialize Driver System
    # =============================================================================
    driver = veh.ChInteractiveDriverIRR(m113.GetVehicle())
    
    # Configure driver inputs
    driver.SetSteeringDelta(0.02)  # Steering speed
    driver.SetThrottleDelta(0.02)  # Throttle response speed
    driver.SetBrakingDelta(0.02)   # Braking response speed
    
    # Initialize driver system
    driver.Initialize()

    # =============================================================================
    # 5. Set Up Visualization
    # =============================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("M113 Vehicle Simulation")
    vis.Initialize()
    
    # Configure camera position
    camera_pos = chrono.ChVector3d(0, -10, 3)  # Side view
    camera_target = init_pos
    vis.AddCamera(camera_pos, camera_target)
    
    # Add lighting and UI elements
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 20, 20, 
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 
                chrono.QuatFromAngleX(chrono.CH_PI_2)),
                chrono.ChColor(0.3, 0.3, 0.3))
    
    # Add vehicle-specific visualization
    vis.EnableGrid(False)
    m113.GetVehicle().AddVisualSystem(vis)

    # =============================================================================
    # 6. Simulation Loop
    # =============================================================================
    # Simulation parameters
    time_step = 0.001  # 1ms timestep for stability
    realtime_step = 0.01  # 10ms real-time step
    render_step = 1.0 / 50  # 50 FPS rendering
    
    # Initialize counters
    time = 0.0
    realtime_timer = chrono.ChRealtimeStepTimer()
    last_render_time = 0

    while vis.Run():
        time = system.GetChTime()
        
        # ========================
        # Driver Input Handling
        # ========================
        driver.Synchronize(time)
        
        # ========================
        # Physics Advancement
        # ========================
        # Advance vehicle and terrain states
        m113.Advance(time_step)
        terrain.Advance(time_step)
        
        # Advance system dynamics
        system.DoStepDynamics(time_step)
        
        # ========================
        # Visualization Update
        # ========================
        if time - last_render_time >= render_step:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            last_render_time = time
        
        # ========================
        # Real-time Synchronization
        # ========================
        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()