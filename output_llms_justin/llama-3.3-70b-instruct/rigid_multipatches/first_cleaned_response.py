import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystem()


vis = chronoirr.ChVisualSystem()
vis.SetWindowSize(800, 600)
vis.AddSkyBox()
vis.AddTypicalLights()


veh_params = veh.HMMWV()
veh_params.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
veh_params.SetChassisFixed(False)
veh_params.SetSuspensionEnabled(True)
veh_params.SetSteeringEnabled(True)
veh_params.SetWheelsEnabled(True)
veh_params.SetEngineType(veh.HMMWV.E_ENGINE_TYPE_INTERNAL_COMBUSTION)
veh_params.SetDrivetrainType(veh.HMMWV.E_DRIVETRAIN_TYPE_ALL_WHEEL_DRIVE)
veh_params.Initialize()


sys.Add(veh_params.GetChassisBody())
sys.Add(veh_params.GetSteering())
sys.Add(veh_params.GetWheels())
sys.Add(veh_params.GetSuspension())


veh_params.GetChassisBody().SetPos(chrono.ChVectorD(0, 1, 0))
veh_params.GetChassisBody().SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))


terrain = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
sys.Add(terrain)


patch1 = chrono.ChBodyEasyBox(sys, 5, 0.1, 5, 1000, True, True)
patch1.SetPos(chrono.ChVectorD(-3, -0.9, 0))
patch1.SetMaterial(chrono.ChMaterialSurfaceNSC())
patch1.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("textures/concrete.jpg")))
sys.Add(patch1)


patch2 = chrono.ChBodyEasyBox(sys, 5, 0.1, 5, 1000, True, True)
patch2.SetPos(chrono.ChVectorD(3, -0.9, 0))
patch2.SetMaterial(chrono.ChMaterialSurfaceNSC())
patch2.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("textures/asphalt.jpg")))
sys.Add(patch2)


bump = chrono.ChBodyEasyMesh(sys, chrono.GetChronoDataFile("objects/bump.obj"), 1000, True, True)
bump.SetPos(chrono.ChVectorD(0, -0.9, 5))
bump.SetMaterial(chrono.ChMaterialSurfaceNSC())
sys.Add(bump)


heightmap = chrono.ChBodyEasyHeightfield(sys, 10, 10, 1000, True, True)
heightmap.SetPos(chrono.ChVectorD(0, -1, -5))
heightmap.SetMaterial(chrono.ChMaterialSurfaceNSC())
heightmap.SetHeight(chrono.ChVectorD(0, 0, 0), 0.5)
heightmap.SetHeight(chrono.ChVectorD(5, 0, 0), 1.0)
heightmap.SetHeight(chrono.ChVectorD(0, 5, 0), 0.0)
sys.Add(heightmap)


driver = veh.HMMWV_Driver(veh_params)
driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


sys.SetTimestep(0.01)
sys.SetMaxIters(10)


veh_params.AddVisualizationAssets()


while vis.Run():
    
    sys.DoStepDynamics(0.01)
    
    
    driver.Synchronize(0.01)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()