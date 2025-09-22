import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens  
import os




chrono.SetChronoDataPath(os.path.join(os.getenv('HOME'), 'chrono_data'))

contact_method = chrono.ChContactMethod_SMC





init_loc = chrono.ChVectorD(0, 0.6, 0)              
init_rot = chrono.Q_from_AngY(0)                    
tire_model = veh.VehicleTireModelType_TMEASY
step_size = 1e-3                                   


vehicle = veh.HMMWV_Vehicle(contact_method)
vehicle.SetInitPosition(chrono.ChCoordsys(init_loc, init_rot))
vehicle.SetTireType(tire_model)
vehicle.SetChassisFixed(False)
vehicle.Initialize()


vis = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis)
vehicle.SetSuspensionVisualizationType(vis)
vehicle.SetSteeringVisualizationType(vis)
vehicle.SetWheelVisualizationType(vis)
vehicle.SetTireVisualizationType(vis)




terrain = veh.RigidTerrain(vehicle.GetSystem())


patch = terrain.AddPatch(
    chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 20, 0.1))
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

terrain.Initialize()




app = veh.ChVehicleIrrApp(vehicle,                
                          "HMMWV on Flat Terrain",
                          irr.dimension2du(1280, 720),
                          veh.VisualizationType_PRIMITIVES)
app.SetSkyBox()               
app.AddTypicalLights()        
app.SetChaseCamera(           
    chrono.ChVectorD(-8, 3, 0),
    5.0, 0.5)
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)
driver.Initialize()





render_fps = 50
render_step = 1.0 / render_fps
next_frame_time = 0.0

system = vehicle.GetSystem()
time = 0.0

print("Starting simulation. Press WASD keys to drive, mouse to steer.")
while app.GetDevice().run():
    
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(time, driver_inputs)

    
    if time >= next_frame_time:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        next_frame_time += render_step

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

print("Simulation finished.")