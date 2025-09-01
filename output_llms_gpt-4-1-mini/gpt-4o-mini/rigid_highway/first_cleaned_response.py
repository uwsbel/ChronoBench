import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.01)


application = chronoirr.ChIrrApp(chrono.ChSystemNSC(), "HMMWV Simulation", chronoirr.dimension2d(800, 600))
application.AddLogo(chrono.GetChronoDataPath() + "logo.png")
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 5, -15))
application.AddTypicalLights()
application.SetShowInfos(True)


terrain_col = chrono.ChTriangleMeshConnected()
chrono.ChTriangleMeshFileLoader.LoadWavefront(terrain_col, "Highway_col.obj", chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33(chrono.CH_C_PI / 2, chrono.CH_C_PI / 2, 0))
terrain_col.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33(chrono.CH_C_PI / 2, chrono.CH_C_PI / 2, 0))

terrain_vis = chrono.ChTriangleMeshConnected()
chrono.ChTriangleMeshFileLoader.LoadWavefront(terrain_vis, "Highway_vis.obj", chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33(chrono.CH_C_PI / 2, chrono.CH_C_PI / 2, 0))
terrain_vis.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33(chrono.CH_C_PI / 2, chrono.CH_C_PI / 2, 0))

terrain_body = chrono.ChBodyEasyMesh(terrain_col, 0, True, True)
terrain_body.SetCollide(True)
terrain_body.SetBodyFixed(True)
chrono.ChSystemNSC().Add(terrain_body)


hmmwv = chrono.ChVehicleHMMWV()
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(chrono.CH_C_PI / 2, 0, 0)))


tire_model = chrono.ChTireTMEASY()
hmmwv.SetTireModel(tire_model)


hmmwv.SetChassisVisualizationType(chrono.ChVehicleVisualType::MESH)
hmmwv.SetWheelVisualizationType(chrono.ChVehicleVisualType::MESH)
hmmwv.SetTireVisualizationType(chrono.ChVehicleVisualType::MESH)


chrono.ChSystemNSC().Add(hmmwv)


driver = chrono.ChDriverHMMWV()
driver.Initialize(hmmwv)


application.SetTimestep(1 / 50.0)  
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    chrono.ChSystemNSC().DoStepDynamics(application.GetTimestep())
    
    
    driver.UpdateInputs()
    
    application.EndScene()


application.Close()