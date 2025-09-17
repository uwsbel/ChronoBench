import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)


for axle in gator.GetAxles():
    tire = veh.TMeasy()
    axle.SetTire(tire)


terrain = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
terrain.SetBodyFixed(True)
system.Add(terrain)


texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.GetAssets().push_back(texture)


application = irr.ChIrrApp(system, 'Gator Simulation', irr.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))


driver = veh.ChIrrGuiDriver(application.GetDevice())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


step_timer = chrono.ChRealtimeStepTimer()
while application.GetDevice().run():
    step_timer.Spin(1/50.0)
    
    
    driver_inputs = driver.GetInputs()
    gator.ApplyDriverInputs(driver_inputs)
    
    
    system.DoStepDynamics(1/50.0)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()