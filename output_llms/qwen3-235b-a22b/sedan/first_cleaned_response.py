import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("CHRONO_DATA_DIR")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.ChWheeledVehicle()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetChassisCollide(True)
vehicle.SetWheelCollide(True)


for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.ChTMeasyTire("Tire")
        tire.SetRadius(0.35)
        tire.SetWidth(0.25)
        tire.SetTMeasyParams(40000, 0.8, 1.0, 1.0, 0.0, 0.0)  
        wheel.SetTire(tire)


powertrain = veh.ChSimplePowertrain()
vehicle.InitializePowertrain(powertrain)


terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetCollide(True)
box = chrono.ChBoxShape()
box.GetBoxGeometry().Size = chrono.ChVectorD(100, 1, 100)
terrain.AddAsset(box)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))


texture = chrono.ChVisualTexture()
texture.SetFile(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.AddAsset(texture)
system.Add(terrain)


app = irr.ChIrrApp(system, 'BMW E90 Simulation', irr.dimension2du(1024, 768))
app.AddSkyBox()
app.AddLightDirectional(chrono.ChVectorD(1, -1, -1))


camera = irr.ChIrrCamera(
    app.GetSceneManager(),
    vehicle.GetChassisBody(),
    irr.vector3df(0, 2, -6),
    irr.vector3df(0, 1, 0)
)
app.SetCamera(camera)


driver = veh.ChInteractiveDriverIRR(app.GetDevice(), vehicle, 0.5, 0.5)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


while app.GetDevice().run():
    driver.Synchronize(system.GetChTime())
    system.DoStepDynamics(1e-3)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()