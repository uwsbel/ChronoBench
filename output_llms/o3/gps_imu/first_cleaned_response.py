import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath( chrono.GetChronoDataPath() + "vehicle/" )


step_size      = 1.0e-3          
render_steps   = 10              
sensor_steps   = 1               
sim_end        = 15.0            






contact_method = chrono.ChContactMethod_NSC
my_hmmwv       = veh.HMMWV_Full( fixed              = False,
                                 contact_method     = contact_method,
                                 brake_type         = veh.BrakeType_SHAFTS )


init_loc  = chrono.ChVectorD( 0, 0, 1.0 )           
init_rot  = chrono.ChQuaternionD( 1,0,0,0 )         
init_pos  = chrono.ChCoordsysD( init_loc, init_rot )

my_hmmwv.SetInitPosition( init_pos )
my_hmmwv.SetChassisFixed( False )
my_hmmwv.SetPowertrainType( veh.PowertrainModelType_SIMPLE )
my_hmmwv.SetDriveType(      veh.DriveType_4WD )
my_hmmwv.SetTireType(       veh.TireModelType_TMEASY )
my_hmmwv.Initialize()


sys = my_hmmwv.GetSystem()

print("Vehicle mass  : {:.1f} kg".format( my_hmmwv.GetVehicle().GetMass() ))
print("Total bodies  : {}".format( sys.Get_bodylist().size() ))




terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction   ( 0.9 )
patch_mat.SetRestitution( 0.01 )

patch = terrain.AddPatch( patch_mat,
                          chrono.ChVectorD(0,0,0),
                          chrono.ChVectorD(0,0,1),
                          300.0,   
                          300.0 )  
patch.SetTexture( chrono.GetChronoDataFile("textures/soil.jpg"), 200, 200 )
terrain.Initialize()




app = veh.ChWheeledVehicleIrrApp(
        my_hmmwv.GetVehicle(),
        "HMMWV – IMU & GPS demo",
        chrono.dimension2du(1280, 720),
        False,                       
        True )                       

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera( chrono.ChVectorD( 0.0, 1.6, 4.0 ), 6.0 )
app.EnableHUD( True )
app.SetTimestep( step_size )


driver = veh.ChIrrGuiDriver( app )
driver.Initialize()




manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight( chrono.ChVectorF(100,100,100), chrono.ChColor(1,1,1), 500 )


imu_rate = 200.0             
gps_rate = 20.0              
lag      = 0.0               
collect  = step_size         


imu_offset = chrono.ChFrameD( chrono.ChVectorD(0, 0, 0), chrono.QUNIT )
imu = sens.ChIMUSensor( my_hmmwv.GetChassisBody(),
                        imu_rate, lag, collect, imu_offset )


f_imu_acc = sens.ChFilterIMUAccess()
imu.PushFilter( f_imu_acc )

manager.AddSensor( imu )


gps_offset = chrono.ChFrameD( chrono.ChVectorD(0, 0, 0), chrono.QUNIT )
gps = sens.ChGPSSensor( my_hmmwv.GetChassisBody(),
                        gps_rate, lag, collect, gps_offset )

f_gps_acc = sens.ChFilterGPSAccess()
gps.PushFilter( f_gps_acc )

manager.AddSensor( gps )




step_number = 0
while (app.GetDevice().run() and sys.GetChTime() < sim_end):

    time = sys.GetChTime()

    
    if step_number % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize ( time )
    terrain.Synchronize( time )
    my_hmmwv.Synchronize( time, driver_inputs, terrain )
    app.Synchronize( "", driver_inputs )

    
    if step_number % sensor_steps == 0:
        manager.Update()

        
        imu_data = f_imu_acc.GetMostRecentBuffer()
        if imu_data:
            imu_read = imu_data[0]          
            
            if step_number % int(0.1/step_size) == 0:
                print("t={:6.2f}s  IMU acc_z={:+8.2f}  ang_vel_z={:+7.2f}"
                      .format(time,
                              imu_read.Accel[2],
                              imu_read.Gyro[2]) )

        gps_data = f_gps_acc.GetMostRecentBuffer()
        if gps_data:
            gps_read = gps_data[0]
            
            if step_number % int(0.5/step_size) == 0:
                print("        GPS lat={:+.6f} lon={:+.6f} alt={:+.2f}"
                      .format(gps_read.Latitude,
                              gps_read.Longitude,
                              gps_read.Altitude) )

    
    driver.Advance ( step_size )
    terrain.Advance( step_size )           
    my_hmmwv.Advance( step_size )
    app.Advance  ( step_size )             
    step_number += 1