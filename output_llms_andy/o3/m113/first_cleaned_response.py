import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())




step_size      = 1.0e-3          
render_step    = 1.0/60.0        
t_end          = 60.0            


terrain_mu     = 0.8             
terrain_e      = 0.01            




contact_method = chrono.ChContactMethod_NSC
shoe_type      = veh.TrackShoeType.BAND_BUSHING        
chassis_fixed  = False

vehicle = veh.M113(shoe_type)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(chassis_fixed)
vehicle.SetTrackAssemblyType(veh.SuspensionType.TORSION_BAR)
vehicle.SetInitPosition(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)  
)
vehicle.Initialize()


system = vehicle.GetSystem()




terrain = veh.RigidTerrain(system)

patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),   
    chrono.ChVectorD(400, 400, 0.02),                              
)
patch.SetContactFrictionCoefficient(terrain_mu)
patch.SetContactRestitutionCoefficient(terrain_e)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 40, 40)

terrain.Initialize()




app = veh.ChTrackedVehicleIrrApp(
    vehicle,                                   
    "M113 simulation demo",                    
    irr.dimension2du(1280, 720)                
)

app.SetSkyBox()
app.AddTypicalLights(                                     
    chrono.ChVectorD(+120, +100, 200),                   
    chrono.ChVectorD(-120, -100, 200), 250, 130          
)
app.SetChaseCamera(chrono.ChVectorD(0, 0, 0.0),          
                  8.0,                                   
                  0.5)                                   
app.Initialize()
app.AddLogo()                                            




driver = veh.ChIrrGuiDriver(app)
driver.SetTimestep(step_size)
driver.Initialize()


print("\nControls:")
print("  W/S  : throttle up / down")
print("  A/D  : steer left / right")
print("  SPACE: full brake\n")




realtime_timer   = chrono.ChRealtimeStepTimer()
render_accum     = 0.0

while app.GetDevice().run():

    
    if system.GetChTime() >= t_end:
        break

    
    driver_inputs = driver.GetInputs()

    
    step = step_size

    
    time = system.GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize("M113 demo", driver_inputs)

    
    driver.Advance(step)
    vehicle.Advance(step)
    terrain.Advance(step)
    app.Advance(step)

    
    render_accum += step
    if render_accum >= render_step:
        app.BeginScene(True, True, chrono.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        render_accum = 0.0

    
    if not realtime_timer.Spin(step):
        
        pass




print("Simulation finished.")