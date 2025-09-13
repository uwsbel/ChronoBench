import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    chrono.SetChronoDataPath("path/to/data/")  
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.8)
    patch_mat.SetRestitution(0.01)
    terrain.Initialize(chrono.ChCoordsysD(), patch_mat)

    
    vehicle = veh.M113(system)
    init_pos = chrono.ChVectorD(0, 0.5, 0)  
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    vehicle.Initialize(veh.ChCoordsysD(init_pos, init_rot))

    
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('M113 Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 2, 0), chrono.ChVectorD(0, 0.5, 0))  
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        chrono.ChVectorD(10, 10, 10),  
        chrono.ChVectorD(0, 0, 0),     
        50, 10, 50,                    
        60, 512, chrono.ChColor(0.8, 0.8, 0.8)
    )

    
    step_size = 0.005
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        driver.Synchronize(chrono.ChTimer.GetTime())
        
        
        vehicle.Synchronize(chrono.ChTimer.GetTime(), driver.GetInputs(), terrain)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)  

if __name__ == "__main__":
    main()