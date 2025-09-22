import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 20, 20)


vehicle = veh.CityBus(system)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.2), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetTireModel(veh.TMeasyTireModel())


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)


application = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 0))


driver = veh.ChIrrGuiDriver(vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


time = 0
step_size = 1.0 / 50.0  


while application.GetDevice().run():
    
    driver.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    time += step_size
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()