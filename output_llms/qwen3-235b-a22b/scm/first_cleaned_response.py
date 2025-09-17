import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.terrain as terrain
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.GetChassis().SetFixed(False)
init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
vehicle.Initialize(system, init_pos)  
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_RIGID)  
vehicle.SetTireStepSize(1e-3)


scm_terrain = terrain.SCMTerrain(system)
scm_terrain.SetSoilParameters(
    2e4,    
    0,      
    1.1,    
    0,      
    30,     
    0.01,   
    4e3     
)

scm_terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 1, 3))
scm_terrain.SetPlotType(scm_terrain.PLOT_SINKAGE_COLOR)  
scm_terrain.Initialize()


application = irr.ChIrrApp(system, 'HMMWV on SCM Terrain', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights(irr.vector3df(30, 30, 30), irr.vector3df(-30, -30, -30))
application.SetChaseCamera(chrono.ChVectorD(0, 1.5, 0), 6.0, 0.5)  
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChInteractiveDriverIRR(application)
driver.SetSteeringDelta(0.05)   
driver.SetThrottleDelta(0.05)   
driver.SetBrakingDelta(0.05)    
driver.Initialize()


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, scm_terrain)
    scm_terrain.Synchronize(time)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(1e-3)