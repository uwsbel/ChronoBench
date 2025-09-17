import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


terrain_length = 100
terrain_width = 100
terrain_height = 1
terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)

terrain_body = chrono.ChBodyEasyBox(terrain_length, terrain_height, terrain_width, 1000, True, True)
terrain_body.SetPos(chrono.ChVectorD(0, -terrain_height/2, 0))
terrain_body.SetBodyFixed(True)
terrain_body.GetMaterialSurfaceNSC().SetFriction(0.9)
terrain_body.GetMaterialSurfaceNSC().SetRestitution(0.01)
system.Add(terrain_body)


m113 = veh.ChM113()
m113.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))
m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
m113.SetSprocketVisualizationType(veh.VisualizationType_MESH)
m113.SetIdlerVisualizationType(veh.VisualizationType_MESH)
m113.SetSuspensionVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChDriver(m113.GetVehicle())
driver.Initialize()


app = irrlicht.ChIrrApp(system, 'M113 Simulation', irrlicht.dimension2du(800, 600))
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 3, -5), chrono.ChVectorD(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()


step_size = 0.005
realtime_timer = chrono.ChRealtimeStepTimer()


while app.GetDevice().run():
    time = system.GetChTime()

    
    driver.Synchronize(time)
    m113.Synchronize(time)
    
    
    driver.Advance(step_size)
    m113.Advance(step_size)

    
    system.DoStepDynamics(step_size)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    realtime_timer.Spin(step_size)