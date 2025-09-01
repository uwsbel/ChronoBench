import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


rover = chrono.ChRoverCuriosity()
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
rover.SetChassisVisualizationType(chrono.ChRoverCuriosity.VisualizationType_MESH)
rover.SetWheelVisualizationType(chrono.ChRoverCuriosity.VisualizationType_MESH)
system.Add(rover)


app = irr.ChIrrApp(system, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))
app.SetLight(chrono.ChVectorD(0, 10, -10), 1.0)
app.EnableShadows()


driver = irr.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)


while app.GetDevice().run():
    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 194))
    
    
    app.DrawAll()
    
    
    rover.SetDriverInput(driver.GetThrottle(), driver.GetSteering())
    
    
    app.EndScene()
    
    
    system.DoStepDynamics(0.01)