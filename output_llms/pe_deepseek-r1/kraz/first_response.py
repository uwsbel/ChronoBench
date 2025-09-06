import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # ==========================================================================
    # 1. Initialize PyChrono Environment
    # ==========================================================================
    # Set data path for Chrono assets
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create SMC system (for smooth contact mechanics)
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ==========================================================================
    # 2. Create Rigid Terrain
    # ==========================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)       # High friction for realistic tires
    patch_mat.SetRestitution(0.01)   # Low restitution (minimal bounce)
    patch_mat.SetDampingF(0.2)       # Damping factor
    
    # Add terrain patch (100x100 meter flat surface)
    patch = terrain.AddPatch(patch_mat, 
                            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 
                                            chrono.QUNIT),
                            100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # ==========================================================================
    # 3. Create and Initialize Kraz Vehicle
    # ==========================================================================
    # Create Kraz tractor (articulated vehicle model)
    kraz = veh.Kraz(system)
    
    # Set initial position and orientation
    init_pos = chrono.ChVector3d(0, 0, 1.0)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    
    # Initialize vehicle system
    kraz.Initialize(veh.ChCoordsysd(init_pos, init_rot))
    
    # Set vehicle subsystems visualization modes
    kraz.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_MESH)
    kraz.SetTireVisualizationType(veh.VisualizationType_MESH)

    # ==========================================================================
    # 4. Create Driver Control System
    # ==========================================================================
    # Create interactive driver (keyboard control via Irrlicht)
    driver = veh.ChIrrGuiDriver()
    
    # Set driver parameters
    driver.SetThrottleDelta(0.02)
    driver.SetSteeringDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # ==========================================================================
    # 5. Set Up Irrlicht Visualization
    # ==========================================================================
    # Create Irrlicht visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Kraz Vehicle Simulation')
    vis.Initialize()
    
    # Configure visualization settings
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    
    # Set camera position and target
    vis.AddCamera(chrono.ChVector3d(8, 6, 2), chrono.ChVector3d(0, 0, 1))
    vis.AddTypicalLights(chrono.ChVector3d(30, 30, 30), chrono.ChVector3d(-30, -30, 30), 50, 50)
    
    # Add vehicle-specific visualization
    vis.EnableGrid(True)
    kraz.AddVisualizationAssets(vis)

    # ==========================================================================
    # 6. Simulation Loop
    # ==========================================================================
    # Simulation parameters
    time_step = 0.002  # 2ms time step
    realtime_step = True  # Maintain real-time synchronization

    while vis.Run():
        # Get current simulation time
        time = system.GetChTime()
        
        # Process driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        # Update vehicle systems
        kraz.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        # Advance simulation state
        system.DoStepDynamics(time_step)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        kraz.Advance(time_step)
        vis.EndScene()
        
        # Maintain real-time execution
        if realtime_step:
            chrono.ChRealtimeStepTimer.Spin(time_step)

if __name__ == "__main__":
    main()