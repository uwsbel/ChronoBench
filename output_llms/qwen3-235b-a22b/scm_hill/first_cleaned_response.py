import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0)))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(0.005)
vehicle.Initialize()


terrain = chrono.ChSCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
terrain.SetSoilParameters(
    2e6,    
    0.3,    
    0.01,   
    35,     
    2000,   
    0.01,   
    0.0     
)
terrain.Initialize(chrono.ChVectorD(0, 0, 0), 10, 10, 'heightmap.png')  
terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))


driver = veh.ChIrrGuiDriver(vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


application = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV on SCM Terrain', irr.dimension2du(1280, 720))
application.AddTypicalLights(irr.vector3df(30, 100, 30), irr.vector3df(30, -100, 30), 90, 90, chrono.ChColor(0.8, 0.8, 0.8))
application.AddSkyBox()
application.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
application.AddCamera(chrono.ChVectorD(0, -5, 2), chrono.ChVectorD(0, 0, 1))
application.SetTimestep(0.01)
application.SetRealtime(True)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(application.GetTimestep())
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()