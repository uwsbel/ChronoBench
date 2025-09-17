import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr


system = chrono.ChSystem()
app = chronoirr.ChIrrApp(system, 'HMMWV on Complex Terrain', chronoirr.dimension2du(1024, 768))
app.SetCamera(chronoirr.ChVectorD(0, 0, 5), chronoirr.ChVectorD(0, 0, 0))
app.AssetBind()
app.AssetUpdate()
app.AddTypicalLights()


terrain_system = chrono.ChTerrainSystem()
system.Add(terrain_system)


hmmwv = vehicle.HMMWV_Vehicle()
hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(vehicle.VisualizationType_MESH)


hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)), False)
hmmwv.SetTerrainSystem(terrain_system)



flat1 = vehicle.ChFlatTerrainPatch()
flat1.SetTexture('concrete.jpg')  
flat1.SetSize(20, 20)
flat1.SetPosition(chrono.ChVectorD(-10, -10, 0))
terrain_system.AddPatch(flat1)


flat2 = vehicle.ChFlatTerrainPatch()
flat2.SetTexture('grass.jpg')  
flat2.SetSize(20, 20)
flat2.SetPosition(chrono.ChVectorD(10, 10, 0))
terrain_system.AddPatch(flat2)


bump_patch = vehicle.ChMeshBasedTerrain()
bump_patch.SetMeshFile('bump.obj')  
bump_patch.SetPosition(chrono.ChVectorD(0, 0, 0))
terrain_system.AddPatch(bump_patch)


heightmap_patch = vehicle.ChHeightmapTerrain()
heightmap_patch.SetHeightmapFile('heightmap.png')  
heightmap_patch.SetHeightmapScale(10, 10, 1)  
heightmap_patch.SetPosition(chrono.ChVectorD(-5, 5, 0))
terrain_system.AddPatch(heightmap_patch)


driver = vehicle.ChKeyboardDriver(hmmwv.GetDriverInputs())


step_size = 0.01
while app.GetDevice().run():
    
    driver.Synchronize(system.GetChTime())
    
    
    hmmwv.Synchronize(system.GetChTime(), driver)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    system.DoStepDynamics(step_size)
    app.DoStep()