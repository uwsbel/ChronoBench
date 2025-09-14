import chrono.chrono as chrono
import chrono.chrono_vehicle as veh
import chrono.chrono_irrlicht as irr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()


vehicle = veh.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TireType_RIGID)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(), 
    chrono.ChVectorD(200, 200, 0.5), 
    veh.TerrainType_GRANULAR
)
patch.SetTexture(veh.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()


app = irr.ChIrrApp(system, "CityBus Simulation", 1024, 768)
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 0, 0))
app.SetChaseCamera(vehicle.GetChassisBody(), 8.0, 0.5)
app.AssetBindAll()
app.AssetUpdateAll()


driver = veh.ChInteractiveDriverIRR(app)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()


step_size = 0.02


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    app.BeginScene()
    app.DrawAll()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize("", driver_inputs)
    
    
    app.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    app.EndScene()