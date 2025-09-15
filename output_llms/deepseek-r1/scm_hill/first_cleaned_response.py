import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetSoilParameters(
        Bekker_Kphi=5301e3,
        Bekker_Kc=102e3,
        Bekker_n=0.793,
        Mohr_cohesion=1.3e3,
        Mohr_friction=31.1,
        Janosi_shear=1.2e-2,
        Elastic_K=4e8,
        Damping_R=3e4
    )
    terrain.Initialize(20.0, 20.0, 0.2)  

    
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on Deformable Terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        chrono.ChVectorD(-5, 0, 8), chrono.ChVectorD(0, 0, 0),
        50, 5, 15, 50, 512, chrono.ChColor(0.8, 0.8, 0.8)
    )
    vis.EnableShadows()

    
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = system.GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        driver.Synchronize(time)

        
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == '__main__':
    main()