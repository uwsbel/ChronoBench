import pychrono            as chrono
import pychrono.vehicle    as veh
import pychrono.irrlicht   as irr
import math
import os




contact_method     = chrono.ChContactMethod_NSC           
step_size          = 1.0e-3                               
render_fps         = 50.0                                 
render_step        = 1.0 / render_fps                     
tire_step_size     = 1.0e-3                               




chrono.SetChronoDataPath( chrono.GetChronoDataPath() )    

system = chrono.ChSystemNSC()
system.Set_G_acc( chrono.ChVectorD(0, 0, -9.81) )


app = veh.ChVehicleIrrAppInterface(
            "Gator – TMeasy tires on rigid terrain",
            system,
            irr.dimension2du(1280, 720),
            False,     
            False )    




terrain = veh.RigidTerrain(system)

length = 200.0          
width  = 200.0
thick  = 0.2
patch  = terrain.AddPatch(
            chrono.ChCoordsysD( chrono.ChVectorD(0, 0, 0), chrono.QUNIT ),
            veh.RigidTerrain.BrickMaterial(chrono.ChContactMethod_NSC),
            length, width, thick )

patch.SetTexture( chrono.GetChronoDataFile("vehicle/terrain/grass.jpg"), 10, 10 )
terrain.Initialize()





init_loc = chrono.ChVectorD(0.0, 0.0, 0.25)                
init_rot = chrono.Q_from_AngY( math.radians(10.0) )        
init_pos = chrono.ChCoordsysD(init_loc, init_rot)

vehicle = veh.Gator( system,                              
                     False,                               
                     veh.ChassisCollisionType.NONE )      

vehicle.SetContactMethod( contact_method )
vehicle.SetInitPosition( init_pos )
vehicle.SetTireType( veh.TireModelType.TMEASY )
vehicle.SetTireStepSize( tire_step_size )
vehicle.SetChassisVisualizationType( veh.VisualizationType.MESH )
vehicle.SetSuspensionVisualizationType( veh.VisualizationType.MESH )
vehicle.SetSteeringVisualizationType  ( veh.VisualizationType.MESH )
vehicle.SetWheelVisualizationType     ( veh.VisualizationType.MESH )

vehicle.Initialize()


powertrain = veh.Gator_SimplePowertrain( "GatorPowertrain" )
vehicle.InitializePowertrain( powertrain )




driver = veh.ChIrrGuiDriver( app )


driver.SetSteeringGain( 1.0 )          
driver.SetThrottleDelta( 0.02 )        
driver.SetBrakeDelta   ( 0.02 )

driver.Initialize()




app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.AddTypicalCamera( chrono.ChVectorD(1.5, -3, 1.2),
                      chrono.ChVectorD(0, 0, 0.5) )

app.AssetBindAll()
app.AssetUpdateAll()


realtime_timer = chrono.ChRealtimeStepTimer()




time = 0.0
render_accumulator = 0.0

print("Interactive Gator demo. Press ESC to quit.")

while app.GetDevice().run():

    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    driver.Draw   ( app.GetDevice() )   
    app.DrawAll()

    
    time = system.GetChTime()

    driver_inputs = driver.GetInputs()

    driver.Synchronize ( time )
    terrain.Synchronize( time )
    vehicle.Synchronize( time, driver_inputs, terrain )
    app.Synchronize    ( "", driver_inputs )

    
    system.DoStepDynamics( step_size )

    
    realtime_timer.Spin( step_size )

    app.EndScene()

    
    if app.IsKeyDown( irr.KEY_KEY_ESCAPE ):
        break




print("Simulation terminated.")