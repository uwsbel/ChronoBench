import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = irr.ChIrrApp(system, 'HMMWV on Custom Terrain', irr.dimension2du(1024, 768))
app.SetCameraPosition(chrono.ChVectorD(0, 3, 15))
app.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y))


terrain_body = chrono.ChBodyEasyStatic()
terrain_body.SetPos(chrono.ChVectorD(0, 0, 0))


collision_mesh = chrono.ChTriangleMeshShape()
collision_mesh.SetMesh(chrono.importMesh('Highway_col.obj'))
collision_mesh.SetCollide(True)
terrain_body.AddAsset(collision_mesh)


visual_mesh = chrono.ChTriangleMeshShape()
visual_mesh.SetMesh(chrono.importMesh('Highway_vis.obj'))
terrain_body.AddAsset(visual_mesh)

system.Add(terrain_body)


vehicle = veh.CreateHMMWV(
    system,
    chrono.ChVectorD(0, 0.5, 0),  
    chrono.ChQuaternionD(1, 0, 0, 0),  
    chrono.ChCollisionSystemType.NSC,  
    veh.ChTireModelType.TMEASY  
)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(app)


app.SetTimestep(0.02)  


while app.GetDevice().run():
    
    driver.Synchronize(app.GetSystemTime(), vehicle)
    
    
    system.DoStepDynamics(app.GetTimestep())
    
    
    vehicle.Update(app.GetSystemTime())
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()