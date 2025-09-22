import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# =============================================================================
# Main simulation function
# =============================================================================

def main():
    # Set path to Chrono data directory
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    
    # Create physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Create and initialize HMMWV vehicle
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # Create deformable terrain with SCM
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        Bekker_Kphi=1.0e6,
        Bekker_Kc=1.4e4,
        Bekker_n=1.0,
        Mohr_cohesion=3.3e4,
        Mohr_friction=30,
        Janosi_shear=0.01,
        elastic_K=2e7,
        damping_R=3e4
    )
    
    # Set terrain visualization
    terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 30000)
    terrain.Initialize(0.0, 0.0, 0.2, 300, 20)
    
    # Add driver system
    driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # Create Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on Deformable Terrain')
    vis.Initialize()
    
    # Configure camera
    vis.AddCamera(chrono.ChVectorD(-5, 5, 2), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    
    # Add vehicle-specific visualization
    vehicle.AddVisualSystemAssets(vis)
    
    # Simulation parameters
    time_step = 0.005
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    # Main simulation loop
    while vis.Run():
        # Get current simulation time
        time = system.GetChTime()
        
        # Synchronize vehicle systems
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        driver.Synchronize(time)
        
        # Advance simulation
        system.DoStepDynamics(time_step)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Maintain real-time execution
        realtime_timer.Spin(time_step)

# =============================================================================
# Entry point
# =============================================================================
if __name__ == '__main__':
    main()