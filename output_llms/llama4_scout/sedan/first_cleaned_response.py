import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.set_default_G_acc(9.81)


system = ch.ChSystemEulerImplicit()


app = irr.ChIrrApp(system, "BMW E90 Sedan Simulation", irr.IrrlichtInfo())


terrain = veh.ChTerrain()
terrain.SetSystem(system)
terrain.Initialize(0.0, 0.0, 0.0)


terrain.SetVisualize(enable=True)
terrain.SetCollide(enable=True)


vehicle = veh.ChBMW_E90()
vehicle.Initialize(system, terrain.GetGroundBody(), ch.ChCoordsysD(0, 1.0, 0.5, ch.ChQuaternionD(0, 0, 0, 1)))


vehicle.SetVisualize(enable=True)
vehicle.SetCollide(enable=True)


tire_model = veh.ChTMEASY()
vehicle.SetTireModel(tire_model)


driver = veh.ChInteractiveDriver()
driver.Initialize(vehicle)


driver.SetThrottle(0.5)
driver.SetSteering(0.0)
driver.SetBraking(0.0)


vehicle_node = app.AddChronoVehicle(vehicle, "vehicle")


camera = irr.ChChaseCamera()
camera.Initialize(vehicle_node, vehicle.GetChassisBody(), ch.ChVectorD(0, -10, 5))


light = irr.ChDirectionalLight()
light.SetDirection(ch.ChVectorD(1, -1, -1))
app.AddLight(light)


skybox = irr.ChSkyBox()
app.AddSkyBox(skybox)


terrain.SetTexture(irr.ChTexture("terrain.jpg"))
terrain.SetLogo(irr.ChLogo("logo.png"))


while app.Run():
    system.DoStepDynamics(1e-3)
    app.Synchronize("", 1e-3)
    app.Advance(1e-3)