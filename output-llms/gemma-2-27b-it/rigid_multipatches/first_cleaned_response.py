import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle


chrono.SetChronoDataPath("data/")
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = chronovehicle.ChVehicle("HMMWV", system)


vehicle.SetInitPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)


engine = chronovehicle.ChEngineSimple()
engine.SetMaxTorque(500)
engine.SetMaxRpm(4000)
drivetrain = chronovehicle.ChDrivelineSimple()
drivetrain.SetEngine(engine)
vehicle.SetDrivetrain(drivetrain)


terrain = chrono.ChTerrainModel()


patch1 = chrono.ChPatch()
patch1.SetTexture(chrono.GetChronoDataPath() + "terrain/textures/grass.png")
patch1.SetSize(chrono.ChVectorD(10, 10, 0))
patch1.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.AddPatch(patch1)

patch2 = chrono.ChPatch()
patch2.SetTexture(chrono.GetChronoDataPath() + "terrain/textures/sand.png")
patch2.SetSize(chrono.ChVectorD(10, 10, 0))
patch2.SetPos(chrono.ChVectorD(10, 0, 0))
terrain.AddPatch(patch2)


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadFromFile(chrono.GetChronoDataPath() + "terrain/meshes/bump.obj")
bump_patch = chrono.ChPatch()
bump_patch.SetMesh(bump_mesh)
bump_patch.SetPos(chrono.ChVectorD(20, 0, 0))
terrain.AddPatch(bump_patch)


heightmap = chrono.ChHeightmap()
heightmap.LoadFromFile(chrono.GetChronoDataPath() + "terrain/heightmaps/terrain.png")
heightmap_patch = chrono.ChPatch()
heightmap_patch.SetHeightmap(heightmap)
heightmap_patch.SetPos(chrono.ChVectorD(30, 0, 0))
terrain.AddPatch(heightmap_patch)


system.Add(terrain)


driver = chronovehicle.ChDriver(vehicle)


vis = chronoirr.ChIrrApp(system, "HMMWV Simulation", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))


while vis.GetDevice()->run():
    vis.BeginScene()
    vis.Render()
    driver.Synchronize(vis.GetTimeStep())
    system.DoStepDynamics(vis.GetTimeStep())
    vis.EndScene()