import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


print("Copyright (c) 2017 projectchrono.org")


chrono.SetChronoDataPath('/path/to/data')


my_hmmwv = veh.HMMWV()
my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
my_hmmwv.SetChassisCollisionType(chrono.ChChassisCollisionType_BOX)
my_hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
my_hmmwv.SetDriveType(veh.DrivetrainTypeWV_AWD)
my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
my_hmmwv.SetTireStepSize(0.01)
my_hmmwv.Initialize()


my_hmmwv.AddVisualizationAssets()


terrain = veh.RigidTerrain(my_hmmwv.GetSystem())


patch1 = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 16, 16)
patch1.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)

patch2 = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0), chrono.QUNIT), 16, 16)
patch2.SetTexture(chrono.GetChronoDataFile('textures/asphalt.jpg'), 200, 200)

patch3 = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(32, 0, 0), chrono.QUNIT), 16, 16)
patch3.SetTexture(chrono.GetChronoDataFile('textures/grass.jpg'), 200, 200)

patch4 = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(48, 0, 0), chrono.QUNIT), 16, 16)
patch4.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)


bump = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile('meshes/bump.obj'), 1000, True, True, chrono.ChMaterialSurfaceNSC())
bump.SetPos(chrono.ChVector3d(32, -0.1, 0))
terrain.AddBody(bump)


heightmap = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile('meshes/heightmap.obj'), 1000, True, True, chrono.ChMaterialSurfaceNSC())
heightmap.SetPos(chrono.ChVector3d(64, -0.1, 0))
terrain.AddBody(heightmap)


terrain.Initialize()


myapplication = chronoirr.ChIrrApp(my_hmmwv.GetSystem(), 'HMMWV on Complex Terrain', chronoirr.dimension2du(800, 600))


driver = veh.ChIrrNodeApplicationDriver(myapplication, my_hmmwv, my_hmmwv.GetPowertrain())


myapplication.SetTimestep(0.01)
myapplication.SetTryRealtime(True)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()