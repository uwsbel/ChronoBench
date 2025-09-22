import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath() + '\\vehicle\\')
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    
    
    
    
    init_pos = chrono.ChVectorD(0, 0.5, 0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    
    
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    
    
    terrain = veh.RigidTerrain(system)
    
    
    patch1_mat = chrono.ChMaterialSurfaceNSC()
    patch1 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 50, 30)
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 30)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    
    patch2 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(50, 0, -20), chrono.ChVectorD(0, 1, 0), 30, 40)
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 30, 40)
    patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
    
    
    patch3 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(30, 0, 20), chrono.ChVectorD(0, 1, 0), 10, 10)
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bump.obj"))
    patch3.SetMesh(mesh)
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 10, 10)
    
    
    patch4_mat = chrono.ChMaterialSurfaceNSC()
    patch4_mat.SetFriction(0.9)
    patch4 = terrain.AddPatch(
        patch4_mat, chrono.CSYSNORM, 
        veh.GetDataFile("terrain/height_maps/bump64.bmp"), 
        "heightmap", 40, 40, 0, 5
    )
    patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 40, 40)
    
    terrain.Initialize()
    
    
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV on Complex Terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 3, 6))
    vis.AddTypicalLights()
    
    
    vis.AddModel(chrono.GetChronoDataFile('models/trees.obj'), chrono.ChVectorD(20,0,10))
    vis.AddModel(chrono.GetChronoDataFile('models/trees.obj'), chrono.ChVectorD(40,0,-5))
    
    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    
    
    step_size = 0.02
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        time = system.GetChTime()
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()