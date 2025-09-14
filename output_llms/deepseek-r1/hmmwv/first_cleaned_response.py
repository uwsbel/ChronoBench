import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("data/")  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


hmmwv = veh.HMMWV.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(100, 100, 1), 
                        "textures/concrete.jpg")
patch.SetContactFriction(0.9)
patch.SetContactRestitution(0.01)
terrain.Initialize()


app = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2du(800, 600))
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 3, 3), irr.vector3df(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(app)
hmmwv.SetDriver(driver)


step_size = 0.02
app.SetTimestep(step_size)
app.SetTryRealtime(True)


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()
    
    
    hmmwv.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    
    
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()