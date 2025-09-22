import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())        
chrono.SetCollisionEnvelope(0.03)                           


contact_method   = chrono.ChContactMethod_NSC               
step_size        = 1.0e-3                                   
render_FPS       = 50                                       
render_steps     = int(math.floor(1.0 / (render_FPS*step_size)))  
realtime_timer   = chrono.ChRealtimeStepTimer()             






init_loc         = chrono.ChVectorD(0.0, 0.0, 0.8)          
init_rot         = chrono.ChQuaternionD(1,0,0,0)            
chassis_fixed    = False


vehicle = veh.hmmwv.HMMWV_Full()                            
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(chassis_fixed)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType.PRIMITIVES)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetPowertrainType(veh.PowertrainModelType.SHAFTS)
vehicle.SetTireType(veh.TireModelType.TMEASY)               
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()


visPRIM = veh.VisualizationType.PRIMITIVES
vehicle.SetVisualizationTypeChassis(visPRIM)
vehicle.SetVisualizationTypeSuspension(visPRIM)
vehicle.SetVisualizationTypeSteering(visPRIM)
vehicle.SetVisualizationTypeWheels(visPRIM)
vehicle.SetVisualizationTypeTires(visPRIM)


system  = vehicle.GetSystem()                               
terrain = veh.RigidTerrain(system)


if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChMaterialSurfaceNSC()
else:
    patch_mat = chrono.ChMaterialSurfaceSMC()

length = 400.0                                              
width  = 400.0                                              
patch  = terrain.AddPatch(patch_mat,
                          chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                          length, width)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 8.0, 8.0)
patch.SetColor(chrono.ChColor(0.6,0.6,0.6))
terrain.Initialize()




app = veh.ChVehicleIrrApp(vehicle,                            
                          "HMMWV ‑ Full ‑ Rigid terrain",
                          irr.dimension2du(1280,720))

app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.SetChaseCamera( chrono.ChVectorD(-6.0, 0.0, 1.5), 6.0, 0.5)    
app.SetTimestep(step_size)


driver = veh.ChIrrGuiDriver(app)
driver.SetThrottleDelta(0.02)
driver.SetSteeringDelta(0.03)
driver.SetBrakingDelta (0.1)
driver.Initialize()




step_number = 0
while app.GetDevice().run():

    
    if step_number % render_steps == 0:
        app.BeginScene(True, True, chrono.SColor(255,140,161,192))
        app.DrawAll()
        app.EndScene()

    
    time      = system.GetChTime()
    driver_inputs = driver.GetInputs()                       

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("HMMWV demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1