import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


veh_hmmwv = veh.ChVehicleHMMWV()
veh_hmmwv.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
veh_hmmwv.SetChassisFixed(False)
veh_hmmwv.SetEngineType(veh.ChEngineType::INTERNAL_COMBUSTION)
veh_hmmwv.SetDrivetrainType(veh.ChDrivetrainType::FOUR_WHEEL_DRIVE)
veh_hmmwv.SetPosition(chrono.ChVectorD(0, 0, 1.5))
veh_hmmwv.AddMeshVisual(chrono.ChVisualMaterial(), "HMMWV", 1.0)


sys.Add(veh_hmmwv)


terrain = chrono.ChTerrain()
terrain.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))


patch1 = chrono.ChTerrainPatch()
patch1.SetTexture("flat1.jpg")
patch1.SetPosition(chrono.ChVectorD(-10, 0, 0))
patch1.SetSize(chrono.ChVectorD(10, 10, 1))
terrain.AddPatch(patch1)

patch2 = chrono.ChTerrainPatch()
patch2.SetTexture("flat2.jpg")
patch2.SetPosition(chrono.ChVectorD(10, 0, 0))
patch2.SetSize(chrono.ChVectorD(10, 10, 1))
terrain.AddPatch(patch2)


bump_mesh = chrono.ChTriangleMesh()
bump_mesh.LoadWavefrontMesh("bump.obj")
bump_patch = chrono.ChTerrainPatch()
bump_patch.SetMesh(bump_mesh)
bump_patch.SetPosition(chrono.ChVectorD(0, 0, 2))
bump_patch.SetSize(chrono.ChVectorD(5, 5, 2))
terrain.AddPatch(bump_patch)


heightmap = chrono.ChHeightfield()
heightmap.LoadHeightfield("heightmap.png")
heightmap_patch = chrono.ChTerrainPatch()
heightmap_patch.SetHeightfield(heightmap)
heightmap_patch.SetPosition(chrono.ChVectorD(20, 0, 0))
heightmap_patch.SetSize(chrono.ChVectorD(20, 20, 5))
terrain.AddPatch(heightmap_patch)


sys.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(veh_hmmwv)
driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


sys.SetTimestep(0.01)
sys.SetMaxIters(10)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()