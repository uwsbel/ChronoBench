import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr





def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
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
    
    
    terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 30000)
    terrain.Initialize(0.0, 0.0, 0.2, 300, 20)
    
    
    driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on Deformable Terrain')
    vis.Initialize()
    
    
    vis.AddCamera(chrono.ChVectorD(-5, 5, 2), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    
    
    vehicle.AddVisualSystemAssets(vis)
    
    
    time_step = 0.005
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    
    while vis.Run():
        
        time = system.GetChTime()
        
        
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        driver.Synchronize(time)
        
        
        system.DoStepDynamics(time_step)
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        realtime_timer.Spin(time_step)




if __name__ == '__main__':
    main()