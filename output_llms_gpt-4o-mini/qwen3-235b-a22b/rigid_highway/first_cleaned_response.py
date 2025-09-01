import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


step_size = 1.0 / 50.0
time = 0.0


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV_Full()
vehicle.Initialize(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetTireType(veh.TireModelType_TMeasy)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.GetVehicle().SetChassisPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.QUNIT))


application = irr.ChIrrApp(system, 'HMMWV on Custom Terrain', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights(irr.vector3df(30, 100, 30), irr.vector3df(30, 0, 30), 256, 256)
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 1, 0))
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application.GetDevice(), vehicle.GetVehicle(), 20, 0.3, True, True)



mesh_col = chrono.ChTriangleMeshConnected()
mesh_col.LoadWavefrontMesh('Highway_col.obj', True, True)

mesh_vis = chrono.ChTriangleMeshConnected()
mesh_vis.LoadWavefrontMesh('Highway_vis.obj', True, True)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddTriangleMesh(mesh_col, True, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)


vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh_vis)
vis_shape.SetName('Highway Terrain')
ground.AddVisualShape(vis_shape)

system.AddBody(ground)


rt_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    
    rt_timer.Spin(step_size)
    time += step_size

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.GetVehicle().Synchronize(time, driver_inputs)

    
    system.DoStepDynamics(step_size)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()