import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # ==========================================================================
    # 1. Initialize Simulation Environment
    # ==========================================================================
    # Create Chrono system with SMC contact method
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    
    # Set global collision margins
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

    # ==========================================================================
    # 2. Create Vehicle System
    # ==========================================================================
    # Initialize CityBus vehicle
    bus = veh.CityBus()
    
    # Set vehicle parameters
    bus.SetContactMethod(chrono.ChContactMethod_SMC)
    bus.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)
    bus.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.7),  # Initial position (above ground)
        chrono.QuatFromAngleY(chrono.CH_PI)  # Facing positive Y direction
    ))
    
    # Configure visualization
    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # Initialize vehicle system
    bus.Initialize()
    
    # Set tire parameters (TMeasy model)
    tire_type = veh.TireModelType_TMEASY
    bus.SetTireType(tire_type)
    bus.InitializeTires()
    
    # ==========================================================================
    # 3. Create Terrain
    # ==========================================================================
    # Initialize rigid terrain
    terrain = veh.RigidTerrain(system)
    
    # Create ground patch with custom texture
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    
    patch = terrain.AddPatch(patch_mat, 
                            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                            200, 100)  # 200m x 100m terrain
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    terrain.Initialize()

    # ==========================================================================
    # 4. Setup Visualization
    # ==========================================================================
    # Create Irrlicht visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("CityBus Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, 8, 1.5))  # Offset camera position
    vis.AddTypicalLights()
    
    # Attach vehicle visualization
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    # ==========================================================================
    # 5. Setup Interactive Driver System
    # ==========================================================================
    # Create interactive driver
    driver = veh.ChIrrGuiDriver(vis)
    
    # Set driver parameters
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    
    # Initialize driver system
    driver.Initialize()

    # ==========================================================================
    # 6. Simulation Loop Configuration
    # ==========================================================================
    # Simulation parameters
    step_size = 0.002  # Physics time step (500 Hz)
    render_step = 0.02  # Rendering interval (50 FPS)
    
    # Real-time simulation synchronization
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    # Initialize simulation counters
    step_number = 0
    render_steps = int(render_step / step_size)
    
    # ==========================================================================
    # 7. Main Simulation Loop
    # ==========================================================================
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        bus.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        
        # Advance vehicle and terrain states
        bus.Advance(step_size)
        terrain.Advance(step_size)
        
        # Advance physics
        system.DoStepDynamics(step_size)
        
        # Update visualization
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.RenderFrame(chrono.ChFramed(), 2)  # Draw reference frame
            vis.EndScene()
            
            # Update camera position to follow vehicle
            cam_pos = bus.GetChassis().GetPos() + chrono.ChVector3d(-8, -8, 3)
            vis.GetActiveCamera().SetPosition(cam_pos)
            vis.GetActiveCamera().SetTarget(bus.GetChassis().GetPos())
        
        # Increment counters
        step_number += 1
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()