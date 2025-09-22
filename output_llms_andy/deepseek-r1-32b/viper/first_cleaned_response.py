importchrono
import math
from chrono import ChSystem, ChBody, ChVectorD, ChQuaternionD, ChPlane
from chrono.vehicle import vehicle, driver
from chrono irrlicht import ChIrrApp, ChIrrDriver


system = ChSystem()
system.Set_G_acc(ChVectorD(0, -9.81, 0))


ground = ChBody()
ground.SetMass(0)
ground.SetPos(ChVectorD(0, 0, 0))
ground.SetRot(ChQuaternionD(1, 0, 0, 0))
ground_shape = ChPlane()
ground_shape.SetNormal(ChVectorD(0, 1, 0))
ground_shape.SetHeight(0)
ground.AddShape(ground_shape)
system.AddBody(ground)


rover = vehicle.Viper()
rover.Initialize(system, driver.ChIrrDriver(), True)
rover.SetInitPosition(ChVectorD(0, 0.5, 0))


app = ChIrrApp(system, "Viper Rover Simulation", 640, 480)
app.SetLightPosition(ChVectorD(0, 10, 0))
app.SetLightDir(ChVectorD(0, -1, 0))
app.SetCameraPosition(ChVectorD(5, 5, 5))
app.SetCameraTarget(ChVectorD(0, 0, 0))
app.AddTypicalLights()
app.AddAllTypicalLogo()
app.AddTypicalSky()


driver = ChIrrDriver()
driver.Initialize(rover, app)


simulation_time = 10.0
timestep = 0.01
time = 0.0


while app.Loop() and time < simulation_time:
    
    time += timestep
    driver.GetDriver().SetSteering((math.sin(time * 2) * 0.5))
    
    
    system.DoStepDynamics(timestep)
    app.Render()
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    if app.GetDevice().KeyDown( irr.KEY_ESCAPE ):
        break


app.End()