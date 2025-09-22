import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.terrain as terrain
import pychrono.driver as driver
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


m113_vehicle = vehicle.M113(system)
m113_vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.2), chrono.QUNIT))
m113_vehicle.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
m113_vehicle.SetSuspensionVisualizationType(vehicle.VisualizationType_PRIMITIVES)
m113_vehicle.SetTrackShoeVisualizationType(vehicle.VisualizationType_MESH)


terrain_obj = terrain.ChRigidTerrain(system)
terrain_obj.SetFriction(0.9)
terrain_obj.SetRestitution(0.1)
terrain_obj.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
terrain_obj.Initialize()


app = irrlicht.ChIrrApp(system, 'M113 Simulation', irrlicht.dimension2du(1024, 768))
driver_irr = driver.ChIrrGuiDriver(app.GetDevice())
driver_irr.Initialize()


app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
app.AssetBindAll()
app.AssetUpdateAll()


step_size = 1e-3  
while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver_irr.Synchronize(time)
    m113_vehicle.Synchronize(time, driver_irr.GetInputs(), terrain_obj)
    terrain_obj.Synchronize(time)
    
    
    driver_irr.Advance(step_size)
    m113_vehicle.Advance(step_size)
    terrain_obj.Advance(step_size)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    system.DoStepDynamics(step_size)