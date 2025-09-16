import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
application = irr.ChIrrApp(system, 'HMMWV on Complex Terrain', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 5, 10))
application.SetSymbolscale(0.01)
application.SetShowInfos(True)
application.SetPaused(False)


terrain = veh.ChTerrain(system)


patch1 = veh.ChTerrainPatchPlane()
patch1.SetName("flat_patch1")
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch1.SetFriction(0.9)
terrain.AddPatch(patch1, chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1,0,0,0), 100, 100)


patch2 = veh.ChTerrainPatchPlane()
patch2.SetName("flat_patch2")
patch2.SetTexture(veh.GetDataFile("terrain/textures/asphalt.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
patch2.SetFriction(0.8)
terrain.AddPatch(patch2, chrono.ChVectorD(20, 0, 0), chrono.ChQuaternionD(1,0,0,0), 100, 100)


bump_shape = chrono.ChBox()
bump_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(2, 2, 0.5))
bump_patch = veh.ChTerrainPatchMesh()
bump_patch.SetName("bump_patch")
bump_patch.SetMesh(bump_shape)
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
bump_patch.SetFriction(0.9)
terrain.AddPatch(bump_patch, chrono.ChVectorD(10, 0, 0.25), chrono.ChQuaternionD(1,0,0,0), 1, 1)


heightmap_patch = veh.ChTerrainPatchHeightmap()
heightmap_patch.SetName("heightmap_patch")
heightmap_patch.SetHeightmapFile(veh.GetDataFile("terrain/heightmaps/heightmap.png"))
heightmap_patch.SetScale(1.0)
heightmap_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
heightmap_patch.SetColor(chrono.ChColor(0.4, 0.2, 0.1))
heightmap_patch.SetFriction(0.6)
terrain.AddPatch(heightmap_patch, chrono.ChVectorD(-15, 0, 0), chrono.ChQuaternionD(1,0,0,0), 20, 20)


vehicle = veh.HMMWV()
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))  
vehicle.SetEngineType(veh.PowertrainModelType.SimpleMapPowertrain)  
vehicle.SetDrivelineType(veh.HMMWV_DrivelineType_4WD)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize(system, terrain, True, True)  


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle(), application.GetEventReceiver())
vehicle.SetDriver(driver)


application.Add(terrain.GetChVisualSystem())
application.Add(vehicle.GetChVisualization())


application.SetTimestep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.GetSystem().DoStepDynamics(0.01)
    application.OnFrameRender()
    application.EndScene()