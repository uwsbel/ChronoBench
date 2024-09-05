import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "HMMWV Simulation", chronoirr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 5, -15))
application.AddTypicalLights()


terrain_collision_mesh = chrono.ChTriangleMeshConnected()
terrain_collision_mesh.LoadWavefront("Highway_col.obj", True, True)
terrain_collision_shape = chrono.ChCollisionModel.CreateBox(chrono.ChVectorD(0, 0, 0), 100, 1, 100)
terrain_collision_shape.SetName("Terrain")
terrain_collision_shape.SetStatic(True)
terrain_collision_shape.SetCollide(True)
terrain_collision_shape.SetMesh(terrain_collision_mesh)
system.Add(terrain_collision_shape)

terrain_visual_mesh = chrono.ChTriangleMeshConnected()
terrain_visual_mesh.LoadWavefront("Highway_vis.obj", True, True)
terrain_visual_shape = chrono.ChBodyEasyMesh(terrain_visual_mesh, 0, True, True)
terrain_visual_shape.SetPos(chrono.ChVectorD(0, 0, 0))
terrain_visual_shape.SetStatic(True)
system.Add(terrain_visual_shape)


hmmwv = chrono.ChVehicleHMMWV()
hmmwv.SetChassisVisualizationType(chrono.ChVehicleVisualType.MESH)
hmmwv.SetChassisMesh(chrono.GetChronoDataFile("HMMWV/Chassis.obj"))
hmmwv.SetWheelVisualizationType(chrono.ChVehicleVisualType.MESH)
hmmwv.SetWheelMesh(chrono.GetChronoDataFile("HMMWV/Wheel.obj"))
hmmwv.SetTireVisualizationType(chrono.ChVehicleVisualType.MESH)
hmmwv.SetTireMesh(chrono.GetChronoDataFile("HMMWV/Tire.obj"))


hmmwv.SetPos(chrono.ChVectorD(0, 0.5, 0))
hmmwv.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
hmmwv.SetContactMethod(chrono.ChContactMethod.SMART)
hmmwv.SetTireModel(chrono.ChTireModelType.TMEASY)


hmmwv.Initialize()


driver = chrono.ChDriverHMMWV(hmmwv)
driver.SetThrottle(0.5)  
driver.SetSteering(0.0)  
driver.SetBraking(0.0)   


system.Add(driver)


step_size = 1 / 50.0  


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    driver.SetThrottle(0.5)  
    driver.SetSteering(0.0)  
    driver.SetBraking(0.0)   

    
    system.DoStepDynamics(step_size)

    application.EndScene()


application.Close()