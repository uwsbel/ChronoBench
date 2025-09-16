import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.driver as driver


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetStep(0.02)  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


tire_model = veh.TMeasy()
for i in range(vehicle.GetNumberAxles()):
    axle = vehicle.GetAxle(i)
    axle.GetWheelLeft().SetTireModel(tire_model)
    axle.GetWheelRight().SetTireModel(tire_model)

vehicle.Initialize()


terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
system.Add(terrain)


mesh_col = chrono.ChTriangleMeshConnected()
mesh_col.LoadWavefrontMesh("Highway_col.obj", False, True)
terrain.GetCollisionModel().AddTriangleMesh(mesh_col, False, False, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
terrain.GetCollisionModel().Build()


mesh_vis = chrono.ChTriangleMeshConnected()
mesh_vis.LoadWavefrontMesh("Highway_vis.obj", False, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh_vis)
terrain.AddVisualShape(vis_shape)


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


rt_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    rt_timer.Spin(system.GetStep())
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(driver_inputs)
    
    
    system.DoStepDynamics()
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()