import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.terrain as terrain
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV_Full(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.Initialize()


scm_terrain = terrain.SCMDeformableTerrain(system)
scm_terrain.SetSoilParameters(
    5000,    
    800,     
    1.1,     
    0.2e4,   
    30,      
    1e-3,    
    5e3      
)

scm_terrain.Initialize(chrono.ChVectorD(0, 0, 0), 20, 20, 0.1)


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 1, 0))
application.AssetBindAll()
application.AssetUpdateAll()


step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, scm_terrain)
    scm_terrain.Synchronize(time)
    
    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    scm_terrain.Advance(step_size)
    
    
    system.DoStepDynamics(step_size)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    realtime_timer.Spin(step_size)