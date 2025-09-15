import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(50)
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                            200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vehicle = veh.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    vehicle.SetDrivelineType(veh.DrivelineType_SIMPLE)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.Initialize()

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetRoadWheelAssemblyVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('M113 Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 4, 1), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, 5), chrono.ChVectorD(0, 0, 0), 50, 2, 30, 50, 512)

    
    step_size = 0.005
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = system.GetChTime()
        
        
        driver.Synchronize(time)
        
        
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        realtime_timer.Spin(step_size)

if __name__ == '__main__':
    main()