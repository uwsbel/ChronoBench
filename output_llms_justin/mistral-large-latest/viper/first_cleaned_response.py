import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(chrono.GetChronoDataPath() + "../../../data/")


system = chrono.ChSystemNSC()


system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetMaxItersSolverSpeed(70)
system.SetMaxItersSolverStab(40)
system.SetSolverWarmStarting(True)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 1, 50)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)
system.Add(ground)


rover = veh.Viper()
rover.Initialize(system)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


driver = veh.Viper_SimpleCVtDriver(rover.GetVehicle())


driver.Initialize()
driver.SetSteeringDelta(0.1)
driver.SetThrottle(0.5)


app = irr.ChIrrApp(rover.GetVehicle(), 'Viper Rover Simulation', irr.dimension2du(800, 600))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 2, -4), irr.vector3df(0, 1, 0))
app.AddTypicalLights()
app.AddLightWithShadow(irr.vector3df(5, 10, -5), irr.vector3df(15, -15, -15), 10, 5, 30)


app.SetTimestep(0.01)
app.SetTryRealtime(True)


steering_change_duration = 5  
steering_change_rate = 0.1 / steering_change_duration  

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()

    
    current_time = system.GetChTime()
    if current_time < steering_change_duration:
        new_steering = current_time * steering_change_rate
        driver.SetSteeringDelta(new_steering)
    else:
        driver.SetSteeringDelta(0.1)

    app.EndScene()

app.GetDevice().closeDevice()