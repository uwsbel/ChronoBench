import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # ==========================================================================
    # 1. Initialize PyChrono environment and core components
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create SMC system (for deformable terrain interaction)
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    
    # ==========================================================================
    # 2. Create HMMWV vehicle with specific initial conditions
    # ==========================================================================
    # Initialize HMMWV vehicle
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.5),  # Initial position (above terrain)
        chrono.Q_from_AngZ(0)          # Initial orientation (facing +X)
    ))
    hmmwv.Initialize()
    
    # Set vehicle subsystems visualization modes
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # Get vehicle system and add to main system
    vehicle_system = hmmwv.GetVehicle()
    system.Add(vehicle_system.GetSystem())
    
    # ==========================================================================
    # 3. Create SCM deformable terrain with soil parameters
    # ==========================================================================
    terrain = veh.SCMDeformableTerrain(system)
    
    # Configure soil parameters (Becke sand properties)
    terrain.SetSoilParameters(
        Kphi = 5301e3,    # Bekker Kphi
        Kc = 102e3,       # Bekker Kc
        n = 0.793,        # Bekker exponent n
        c = 1.3e3,        # Mohr cohesive limit (Pa)
        phi = 31.1,       # Mohr friction limit (degrees)
        K = 1.2e-2,       # Janosi shear coefficient (m)
        E_elastic = 2e6,  # Elastic stiffness (Pa/m)
        damping = 3e4     # Damping coefficient (Pa-s/m)
    )
    
    # Set terrain visualization
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
    
    # Initialize with flat height field
    terrain.Initialize(terrain_length=20.0, terrain_width=6.0, delta=0.05)
    
    # ==========================================================================
    # 4. Initialize driver system for vehicle control
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
    
    # Configure driver inputs
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    
    # Initialize driver system
    driver.Initialize()
    
    # ==========================================================================
    # 5. Set up real-time visualization
    # ==========================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3, 4, 1.5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    
    # Add vehicle-specific visualization assets
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    # ==========================================================================
    # 6. Simulation loop with real-time synchronization
    # ==========================================================================
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        time = system.GetChTime()
        
        # Render scene
        vis.BeginScene()
        vis.Render()
        
        # Draw GUI elements
        chronoirr.drawGrid(vis, 0.5, 0.5, 12, 12,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(chrono.CH_PI_2)),
                          chrono.ChColor(0.5, 0.5, 0.5), True)
        
        vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update driver system
        driver.Synchronize(time)
        
        # Update vehicle systems
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Maintain real-time execution
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()