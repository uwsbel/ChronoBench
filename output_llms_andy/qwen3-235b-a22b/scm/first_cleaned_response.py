import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.driver as driver


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = irr.ChIrrApp(system, 'HMMWV on SCM Terrain', irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 3, -5), chrono.ChVectorD(0, 0, 0))


vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()


terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2e6,    
                          0.2,    
                          0.01,   
                          30,     
                          0.01,   
                          2000)   

terrain.AddMovingPatch(vehicle.GetChassisBody(), 
                      chrono.ChVectorD(0, 0, 0), 
                      chrono.ChVectorD(10, 2, 10))

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE_COLOR)
terrain.Initialize()


driver = veh.ChDriver(vehicle)
driver.Initialize()


app.AssetBindAll()
app.AssetUpdateAll()


step_size = 1.0 / 50.0
rt_timer = chrono.ChRealtimeStepTimer()

while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    rt_timer.Spin(step_size)