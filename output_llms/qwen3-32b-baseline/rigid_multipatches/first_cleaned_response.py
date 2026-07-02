import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import pychrono.utils as utils


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


hmmwv = vehicle.HMMWV_Full()
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 2), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv.SetEngineType(vehicle.EngineModelType.SHAFTS)
hmmwv.SetDrivelineType(vehicle.DrivelineTypeWV.AXLE)
hmmwv.SetTireType(vehicle.TireModelType.RIGID_MESH)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vehicle.VisualizationType.MESH)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType.MESH)
hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType.MESH)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType.MESH)


terrain = vehicle.RigidTerrain(my_system)


mat_asphalt = terrain.GetMesh().CreateMaterial(chrono.ChVisualMaterial())
mat_asphalt.SetTexture(chrono.GetChronoDataFile('textures/asphalt.jpg'))
patch_asphalt = terrain.AddPatch(mat_asphalt, 
                                 chrono.ChVectorD(0, 0, 0), 
                                 chrono.ChVectorD(0, 0, 1), 
                                 20, 20)
patch_asphalt.SetPos(chrono.ChVectorD(0, 0, 0))


mat_gravel = terrain.GetMesh().CreateMaterial(chrono.ChVisualMaterial())
mat_gravel.SetTexture(chrono.GetChronoDataFile('textures/gravel.jpg'))
patch_gravel = terrain.AddPatch(mat_gravel, 
                                chrono.ChVectorD(0, 0, 0), 
                                chrono.ChVectorD(0, 0, 1), 
                                20, 20)
patch_gravel.SetPos(chrono.ChVectorD(20, 0, 0))

terrain.Initialize()


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('meshes/bump.obj'), False, False)
bump_shape = chrono.ChVisualShapeTriangleMesh(bump_mesh)
bump_shape.SetMutable(False)
bump_body = chrono.ChBodyEasyMesh(bump_mesh, 1000, True, True)
bump_body.SetPos(chrono.ChVectorD(10, -10, 1))  
bump_body.SetFixed(True)
bump_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
my_system.Add(bump_body)


heightmap_terrain = vehicle.ChHeightMapTerrain(100, 100)
heightmap_terrain.LoadHeightMap(chrono.GetChronoDataFile('heightmaps/heightmap.png'), 0, 20)
heightmap_terrain.SetHeightScale(10)
heightmap_terrain.SetXcoordScale(1)
heightmap_terrain.SetYcoordScale(1)
heightmap_terrain.AddToSystem(my_system)


application = irr.ChIrrApp(my_system, 'HMMWV Simulation', irr.dimension2d_u32(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 2, -10))
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetContactsDrawMode(4)
application.SetTimestep(0.001)
application.SetTryRealtime(True)


driver = vehicle.ChInteractiveDriverIRR(hmmwv, application.GetDevice(), 'HMMWV Driver')
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.Initialize()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    driver.Update()
    
    
    time = my_system.GetChTime()
    hmmwv.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    
    
    my_system.DoStepDynamics(application.GetTimeStep())
    hmmwv.Advance(time)
    
    application.EndScene()