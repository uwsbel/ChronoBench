import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht
import pychrono.driver as driver


system = chrono.ChSystemNSC()


application = irrlicht.ChIrrApp(system, 'Gator Vehicle Simulation', irrlicht.dimension2du(1024, 768))
application.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.SetTexture(chrono.GetChronoDataFile("hmmwv/terrain/textures/grass.jpg"))  
terrain.Initialize(200, 200)  


gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireType(veh.TMEASY)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
gator.Initialize(terrain)


driver_system = driver.ChIrrGuiDriver(gator.GetVehicle())
driver_system.SetSteeringDelta(0.05)
driver_system.SetThrottleDelta(0.05)
driver_system.SetBrakingDelta(0.05)


step_size = 1.0 / 50.0  
realtime_timer = chrono.ChRealtimeStepTimer()


while application.GetDevice().run():
    realtime_timer.Spin(step_size)
    
    
    driver_inputs = driver_system.GetInputs()
    gator.Synchronize(system.GetChTime(), driver_inputs)
    
    
    system.DoStepDynamics(step_size)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()