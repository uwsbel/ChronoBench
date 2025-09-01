import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV_Full()
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.QUNIT))
vehicle.SetEngineType(veh.EngineType_SHAFTS)
vehicle.SetDrivetrainType(veh.DrivetrainType_4WD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()
system.Add(vehicle.GetChVehicle())



flat_patch = chrono.ChBody()
flat_patch.SetBodyFixed(True)
flat_patch.SetCollide(True)
flat_patch.GetCollisionModel().AddBox(chrono.ChMaterialSurfaceNSC(), 100, 0.5, 100, chrono.ChVectorD(0, 0, 0))
flat_patch_shape = chrono.ChBoxShape(100, 0.5, 100)
flat_patch_texture = chrono.ChTexture()
flat_patch_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
flat_patch_shape.AddAsset(flat_patch_texture)
flat_patch.AddAsset(flat_patch_shape)
system.Add(flat_patch)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bump.obj"))
mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetCollide(True)
mesh_body.GetCollisionModel().AddTriangleMesh(mesh, chrono.ChMaterialSurfaceNSC(), False, False)
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_texture = chrono.ChTexture()
mesh_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/grass.jpg"))
mesh_shape.AddAsset(mesh_texture)
mesh_body.AddAsset(mesh_shape)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 5))
system.Add(mesh_body)


heightmap = chrono.ChHeightmap()
heightmap.LoadFromFile(chrono.GetChronoDataFile("heightmaps/heightmap.png"))
heightmap_body = chrono.ChBody()
heightmap_body.SetBodyFixed(True)
heightmap_body.SetCollide(True)
heightmap_body.GetCollisionModel().AddHeightmap(heightmap, chrono.ChMaterialSurfaceNSC(), 10, 10, 0.1, 0.5)
heightmap_shape = chrono.ChHeightmapShape(heightmap)
heightmap_texture = chrono.ChTexture()
heightmap_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/dirt.jpg"))
heightmap_shape.AddAsset(heightmap_texture)
heightmap_body.AddAsset(heightmap_shape)
heightmap_body.SetPos(chrono.ChVectorD(10, 0, 10))
system.Add(heightmap_body)


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.AddTypicalLights()
application.AddSkyBox()
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()
while application.GetDevice().run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    realtime_timer.Spin(step_size)
    system.DoStepDynamics(step_size)