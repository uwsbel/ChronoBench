import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.driver as driver


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))


terrain_body = chrono.ChBody()
terrain_body.SetBodyFixed(True)
terrain_body.SetCollide(True)


col_mesh = chrono.ChTriangleMeshConnected()
col_mesh.LoadWavefrontMesh('Highway_col.obj', True, True)
terrain_body.GetCollisionModel().ClearModel()
chrono.ChCollisionModel.AddTriangleMesh(terrain_body.GetCollisionModel(), col_mesh, False, False, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D())
terrain_body.GetCollisionModel().BuildModel()
terrain_body.GetMaterialSurface().SetFriction(0.9)
terrain_body.GetMaterialSurface().SetRestitution(0.01)


vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh.LoadWavefrontMesh('Highway_vis.obj', True, True)
vis_shape = chrono.ChTriangleMeshShape()
vis_shape.SetMesh(vis_mesh)
terrain_body.AddAsset(vis_shape)

system.Add(terrain_body)


vehicle = veh.HMMWV_Full(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,1,0))))
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.GetVehicle().SetTireType(veh.TireModelType_TMeasy)
vehicle.Initialize()


driver = veh.InteractiveDriverIRR(application)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


step_size = 1.0 / 50.0
realtime_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs)
    driver.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    realtime_timer.Spin(step_size)