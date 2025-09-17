import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -0.5), chrono.QUNIT), 
                             200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_NSC)
    m113.SetChassisFixed(False)
    m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    m113.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
    m113.SetDriveType(veh.DrivelineTypeTV_BDS)
    m113.Initialize()

    
    m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetRoadWheelAssemblyVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    driver = veh.ChInteractiveDriverIRR(m113.GetVehicle())
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
    vis.AddCamera(chrono.ChVectorD(2, 1.5, 1), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, 5), chrono.ChVectorD(0, 0, 0.5), 50, 2, 30, 180, 512)
    vis.EnableShadows()

    
    step_size = 0.005
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = system.GetChTime()
        
        
        driver.Synchronize(time)
        
        
        m113.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()