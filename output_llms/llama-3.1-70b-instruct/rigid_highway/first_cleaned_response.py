import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.SetMaxItersSolverSpeed(100)


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
veh_hmmwv.SetChassisFixed(False)
veh_hmmwv.SetInitPosition(chrono.ChVectorD(0, 0.4, 0))
veh_hmmwv.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
veh_hmmwv.Initialize(mysystem)


veh_hmmwv.AddTiresTMEASY()


veh_hmmwv.AddVisualizationMeshes()


terrain = veh.RigidTerrain(mysystem)
terrain.SetName("terrain")
terrain.SetBodyFixed(True)
terrain.SetContactFriction(0.9)
terrain.SetContactRestitution(0.01)
terrain.SetContactMaterial(chrono.ChMaterialSurfaceNSC())


terrain.AddCollisionMesh(chrono.GetChronoDataFile("Highway_col.obj"))
terrain.AddVisualizationMesh(chrono.GetChronoDataFile("Highway_vis.obj"))


driver = veh.ChIrrVehicleDriverHMMWV(veh_hmmwv)
driver.SetSteeringMode(veh.ChIrrVehicleDriverHMMWV.SteeringMode_TYPE_2)


app = chronoirr.ChVisualSystemIrrlicht()
app.SetWindowSize(1024, 768)
app.SetWindowTitle("HMMWV on custom mesh terrain")
app.AddTypicalLights()
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(0, 1.5, -3), chrono.ChVectorD(0, 1, 0))
app.SetSymbolicCameraSpeed(0.5)
app.SetCameraUp(chrono.ChVectorD(0, 0, 1))
app.SetCameraSpeed(0.5)
app.SetTimestep(0.02)
app.SetTryRealtime(True)


while app.Run():
    mysystem.DoStepDynamics(0.02)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    driver.Synchronize(0.02)
    veh_hmmwv.Synchronize(0.02, mysystem)

    
    app.Render()

    
    app.SetTimestep(0.02)