import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()


app = irr.ChIrrApp(my_system, 'Gator Vehicle Simulation', irr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 3, -5), irr.vector3df(0, 0, 0))
app.AddTypicalLights()


gator = veh.Gator(my_system)
gator.SetInitPosition(chrono.ChCoordsys(chrono.ChVector(0, 0.5, 0), chrono.QUNIT))
gator.SetEngineType(veh.EngineModelType_SIMPLE)
gator.SetTransmissionType(veh.TransmissionModelType_SIMPLE_MAP)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(0.001)
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(my_system)
patch = terrain.AddPatch(chrono.ChCoordsys(), chrono.ChVector(200, 1, 200), 0.8)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()


driver = veh.ChIrrGuiDriver(app.GetDevice(), gator)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


manager = sens.ChSensorManager(my_system)


camera = sens.ChCameraSensor(
    gator.GetChassisBody(),
    60,  
    chrono.ChFrame(chrono.ChVector(0, 1.5, 2), chrono.Q_from_AngAxis(0, chrono.ChVector(0, 1, 0))),
    1280, 720,  
    chrono.CH_C_PI / 3  
)
camera.SetName("Front Camera")
manager.AddSensor(camera)


light = sens.ChPointLight(chrono.ChVector(2, 3, 2), chrono.ChColor(1, 1, 1))
manager.AddLight(light)


step_size = 0.001  
time = 0

while app.GetDevice().run():
    time = my_system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update(time)
    
    
    my_system.DoStepDynamics(step_size)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    manager.Advance(step_size)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()