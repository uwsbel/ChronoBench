import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    
    
    
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.5),  
        chrono.Q_from_AngZ(0)          
    ))
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
    vehicle_system = hmmwv.GetVehicle()
    system.Add(vehicle_system.GetSystem())
    
    
    
    
    terrain = veh.SCMDeformableTerrain(system)
    
    
    terrain.SetSoilParameters(
        Kphi = 5301e3,    
        Kc = 102e3,       
        n = 0.793,        
        c = 1.3e3,        
        phi = 31.1,       
        K = 1.2e-2,       
        E_elastic = 2e6,  
        damping = 3e4     
    )
    
    
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
    
    
    terrain.Initialize(terrain_length=20.0, terrain_width=6.0, delta=0.05)
    
    
    
    
    driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
    
    
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    
    
    driver.Initialize()
    
    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3, 4, 1.5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    
    
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    
    
    
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        time = system.GetChTime()
        
        
        vis.BeginScene()
        vis.Render()
        
        
        chronoirr.drawGrid(vis, 0.5, 0.5, 12, 12,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(chrono.CH_PI_2)),
                          chrono.ChColor(0.5, 0.5, 0.5), True)
        
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        
        
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()