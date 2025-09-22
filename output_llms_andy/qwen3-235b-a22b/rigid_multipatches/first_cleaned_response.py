import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.ChTerrain()
terrain.SetContactMethod(chrono.ChContactMethod_NSC)


patch_concrete = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(20, 1, 20), veh.GetDataFile("terrain/textures/concrete.jpg"))
patch_concrete.SetName("concrete")


patch_grass = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChVectorD(0, 0, 25), chrono.ChVectorD(20, 1, 20), veh.GetDataFile("terrain/textures/grass.jpg"))
patch_grass.SetName("grass")
patch_grass.SetFriction(0.7)


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(veh.GetDataFile("vehicle/hmmwv/meshes/bump.obj"), False, True)
patch_bump = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), bump_mesh, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 50), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))), veh.GetDataFile("terrain/textures/dirt.jpg"))
patch_bump.SetName("bump")


heightmap = chrono.ChHeightmap()
heightmap.LoadFromFile(veh.GetDataFile("terrain/heightmaps/heightmap.bmp"), 0, 255)
patch_heightmap = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), heightmap, chrono.ChCoordsysD(chrono.ChVectorD(-30, 0, 0), chrono.QUNIT), veh.GetDataFile("terrain/textures/gravel.jpg"))
patch_heightmap.SetName("heightmap")

terrain.Initialize()


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
vehicle.SetDrivetrainType(veh.DrivetrainType_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize(terrain)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


application = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.AddTypicalLights()

application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


step_size = 1e-3

while application.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    application.Synchronize("", driver_inputs)

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    application.Advance(step_size)

    
    vehicle.GetSystem().DoStepDynamics(step_size)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()