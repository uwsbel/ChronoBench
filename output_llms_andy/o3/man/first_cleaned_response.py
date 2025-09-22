import math
import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath( chrono.GetChronoDataPath() )          
veh.SetDataPath        ( veh.GetDataPath() )                    




system = chrono.ChSystemNSC()                                   




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(
            patch_mat,
            chrono.CSYSNORM,                                    
            500.0, 500.0)                                       
patch.SetTexture(                
      veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetTextureScale(1.0, 1.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()




init_loc   = chrono.ChVectorD( -250.0, 0.15, 0.0)               
init_rot   = chrono.ChQuaternionD(1, 0, 0, 0)                   
truck      = veh.MAN_10t(system)
truck.SetChassisFixed  (False)
truck.SetCollisionType( veh.ChassisCollisionType.NONE )         
truck.SetInitPosition  ( chrono.ChCoordsysD(init_loc, init_rot))
truck.SetTireType      ( veh.TireModelType.TMEASY )
truck.SetTireStepSize  ( 1e-3 )
truck.Initialize()


truck.SetChassisVisualizationType        ( veh.VisualizationType.MESH )
truck.SetSuspensionVisualizationType     ( veh.VisualizationType.PRIMITIVES )
truck.SetSteeringVisualizationType       ( veh.VisualizationType.PRIMITIVES )
truck.SetWheelVisualizationType          ( veh.VisualizationType.MESH )
truck.SetTireVisualizationType           ( veh.VisualizationType.MESH )




app = veh.ChIrrAppVehicle(truck.GetVehicle(),               
                          "MAN-10t 8×8 Truck Demo",
                          irr.dimension2du(1280, 720))


app.AddTypicalLogo( chrono.GetChronoDataFile("logo_pychrono_alpha.png") )
app.AddTypicalSky( chrono.GetChronoDataFile("skybox/"), 10.0, 800.0, 0.8 )
app.AddTypicalLights( chrono.ChVectorF( 100, 100, 100),   
                      chrono.ChVectorF( 0,   0,   0),     
                      250.0, 180.0 )                      
app.SetChaseCamera( chrono.ChVectorD(0.0, 0.5, 0.0),      
                    6.0,                                 
                    0.5 )                                

app.AssetBindAll()
app.AssetUpdateAll()








steering_time = 1.0      
throttle_time = 0.5      
braking_time  = 0.3      
driver = veh.ChIrrGuiDriver(app,
        steering_time,
        throttle_time,
        braking_time,
        False)            


driver.SetSteeringDelta( 0.04 )
driver.SetThrottleDelta( 0.02 )
driver.SetBrakingDelta ( 0.08 )




step_size        = 2e-3                                      
tire_step_size   = 1e-3
app.SetTimestep(step_size)







realtime_timer = chrono.ChRealtimeStepTimer()
while app.GetDevice().run():

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    time = system.GetChTime()

    
    driver.Synchronize(time)
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking  = driver.GetBraking()

    
    truck.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)
    driver.Advance(step_size)

    
    realtime_timer.Spin(step_size)        
    truck.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)

print("Simulation terminated.")