import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh





contact_method      = chrono.ChContactMethod_NSC      
chassis_fixed       = False                           
step_size_dynamics  = 1.0e-3                          
step_size_render    = 1.0/50.0                        
render_steps        = math.ceil(step_size_render / step_size_dynamics)


steering_time       = 1.0
throttle_time       = 1.0
braking_time        = 0.3

initial_loc         = chrono.ChVectorD(0, 0, 0.5)     
initial_yaw         = 0.0                             





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisFixed(chassis_fixed)
hmmwv.SetInitPosition( chrono.ChCoordsysD(initial_loc,
                        chrono.Q_from_AngZ(initial_yaw)) )
hmmwv.SetTireType(veh.TireModelType.TMEASY)
hmmwv.SetTireStepSize(step_size_dynamics)

vis = veh.VisualizationType.MESH
hmmwv.SetChassisVisualizationType(vis)
hmmwv.SetSuspensionVisualizationType(vis)
hmmwv.SetSteeringVisualizationType(vis)
hmmwv.SetWheelVisualizationType(vis)
hmmwv.SetTireVisualizationType(vis)
hmmwv.Initialize()




terrain = veh.RigidTerrain(system)

contact_mat = chrono.ChContactMaterialNSC()


patch = terrain.AddPatch( contact_mat,
                          veh.RigidTerrain.PatchType.MESH,        
                          "Highway_col.obj",                      
                          chrono.ChVectorD(0, 0, 0),              
                          chrono.ChQuaternionD(1,0,0,0) )         

patch.SetMeshTexture("Highway_vis.obj")        
patch.SetColor(chrono.ChColor(0.8,0.8,0.8))
patch.SetThickness(0.1)                        
terrain.Initialize()




app = veh.ChVehicleIrrApp(hmmwv.GetVehicle(),
                          "HMMWV  |  Mesh terrain  |  TMeasy",
                          irr.dimension2du(1280,720))
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.SetHUDLocation(500, 20)
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.4), 4.0, 0.5)  
app.SetTimestep(step_size_dynamics)
app.AssetBindAll()
app.AssetUpdateAll()
app.AddLightWithShadow( chrono.ChVectorD(30, 30, 60),
                        chrono.ChVectorD(0, 0, 0),
                        300, 1, 20, 40 )




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta( math.radians(1.0) )         
driver.SetThrottleDelta( 0.05 )
driver.SetBrakingDelta( 0.2 )
driver.SetSteeringTime( steering_time )
driver.SetThrottleTime( throttle_time )
driver.SetBrakingTime( braking_time )
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()




frame = 0
while app.GetDevice().run():

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    if frame % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("HMMWV demo", driver_inputs)

    
    hmmwv.Advance(step_size_dynamics)
    terrain.Advance(step_size_dynamics)
    driver.Advance(step_size_dynamics)
    app.Advance(step_size_dynamics)

    
    realtime_timer.Spin(step_size_dynamics)

    frame += 1

print("Simulation finished.")