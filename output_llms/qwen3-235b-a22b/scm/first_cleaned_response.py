import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.terrain as terrain
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irrlicht.ChIrrApp(system, 'HMMWV on SCM Terrain', irrlicht.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


hmmwv = vehicle.HMMWV_Full(system)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
hmmwv.SetTireType(vehicle.TireModelType_RIGID)
hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.Initialize()


scm_terrain = terrain.SCMDeformableTerrain(system)
scm_terrain.SetPlane(chrono.ChCoordsysD(), 100, 100)
scm_terrain.SetSoilParameters(
    2e6,    
    0,      
    1.1,    
    0,      
    30,     
    0.01,   
    0.5     
)
scm_terrain.AddMovingPatch(hmmwv.GetChassis(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 1, 5))
scm_terrain.SetPlotType(terrain.SCMDeformableTerrain.PLOT_SINKAGE, 0.0, 0.2)
scm_terrain.Initialize()


driver = vehicle.DriverIRL(hmmwv)
driver.Initialize()


step_size = 1.0 / 50.0
system.SetStep(step_size)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs)
    scm_terrain.Synchronize(time)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(step_size)