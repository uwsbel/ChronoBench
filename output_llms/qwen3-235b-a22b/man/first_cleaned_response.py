import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


truck = veh.MAN10t(system)
truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.2), chrono.QUNIT))
truck.SetTireType(veh.TMEASY)  
truck.Initialize()


truck.SetChassisVisualizationType(veh.VisualizationType_MESH)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetCollide(True)
terrain.GetCollisionModel().AddPlane(chrono.ChCoordsysD(), 100, 100)
system.Add(terrain)


ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)
terrain.SetMaterialSurface(ground_mat)


application = irr.ChIrrApp(system, 'MAN 10t Truck Simulation', irr.dimension2d(1280, 720))
application.AddTypicalSky()
application.AddTypicalLights(irr.vector3df(30, 100, 30), irr.vector3df(-30, -100, -30))


application.AddGround(chrono.GridPlane(20, 20, 1, 1), "textures/concrete.jpg")


camera = irr.ChIrrChaseCamera(application.GetSceneManager(), truck.GetChassisBody(), 6.0, 0.5)
application.GetSceneManager().setActiveCamera(camera)


driver = veh.ChIrrGuiDriver(truck)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


application.AssetBindAll()
application.AssetUpdateAll()

step_timer = chrono.ChRealtimeStepTimer()
while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    truck.Synchronize(time, driver_inputs)
    
    
    step_timer.Start()
    system.DoStepDynamics(chrono.ChTimeStep(0.01))
    step_timer.Spin()
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()