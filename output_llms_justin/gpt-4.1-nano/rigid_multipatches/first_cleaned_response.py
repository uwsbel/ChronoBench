import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("data/")  
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(system, "HMMWV on Complex Terrain", irr.dimension2d(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3d(0, 3, -10))
application.AssetBindAll()
application.AssetUpdateAll()



vehicle = veh.ChWheeledVehicle(chrono.ChMaterialSurfaceNSC())


initial_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(0))
vehicle.SetT_are_chassis_initial_pos(initial_pos)




vehicle.SetChassisMeshFile("vehicle/hmmwv/hmmwv_chassis.obj")
vehicle.SetWheelMeshFile("vehicle/hmmwv/hmmwv_wheel.obj")
vehicle.SetSteeringType(veh.WHEEL steering)
vehicle.SetDrivetrainType(veh.WU_DRIVE)
vehicle.SetEngineType(veh.WU_V8)
vehicle.Initialize(InitialPos=initial_pos, fixed=False)


vehicle.GetChassis()->SetMeshFile("vehicle/hmmwv/hmmwv_chassis.obj")
for wheel in vehicle.GetWheels():
    wheel->SetMeshFile("vehicle/hmmwv/hmmwv_wheel.obj")
application.AssetBindAll()
application.AssetUpdateAll()




terrain1 = chrono.ChRigidTerrain(system)
terrain1.SetContactMethod(chrono.ChContactMethod_NSC)
terrain1.GetMaterialSurface()->SetFriction(0.8)
terrain1.GetMaterialSurface()->SetCompliance(0.01)
terrain1.SetPlaneGround(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
terrain1.GetGroundBody().SetPos(chrono.ChVectorD(0, 0, 0))
terrain1.EnableMeshVisualization()


terrain2 = chrono.ChRigidTerrain(system)
terrain2.SetContactMethod(chrono.ChContactMethod_NSC)
terrain2.GetMaterialSurface()->SetFriction(0.6)
terrain2.GetMaterialSurface()->SetCompliance(0.02)
terrain2.SetPlaneGround(chrono.ChCoordsysD(chrono.ChVectorD(50, 0, 0), chrono.Q_from_AngX(0)))
terrain2.GetGroundBody().SetPos(chrono.ChVectorD(50, 0, 0))
terrain2.EnableMeshVisualization()


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh("meshes/bump_mesh.obj")
bump_shape = chrono.ChTriangleMeshShape()
bump_shape.SetMesh(bump_mesh)
bump_body = chrono.ChBodyEasyMesh(bump_mesh, 1000, True, True)
bump_body.SetPos(chrono.ChVectorD(25, 0, 25))
system.Add(bump_body)



heightmap = chrono.ChHeightMapTerrain()
heightmap.SetSize(100, 100)
heightmap.SetHeight(20, 20, 2.0)  


heightmap_terrain = chrono.ChRigidTerrain(system)
heightmap_terrain.SetHeightMap(heightmap)
heightmap_terrain.SetTextureCreate(true)
heightmap_terrain.SetTexture("textures/grass.jpg")
heightmap_terrain.GetGroundBody().SetPos(chrono.ChVectorD(0, 0, 50))






driver = veh.ChIrrAppDriver(application)
application.SetUserDriver(driver)


application.SetTimestep(0.005)
while application.GetDevice().run():
    system.DoStepDynamics(0.005)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()