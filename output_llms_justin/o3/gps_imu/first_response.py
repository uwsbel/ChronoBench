# =============================================================================
#  HMMWV + IMU + GPS – complete PyChrono example
#
#  • Full HMMWV vehicle model
#  • Rigid‐terrain subsystem
#  • Interactive GUI based on Irrlicht
#  • Interactive driver (steering / throttle / braking by keyboard)
#  • ChSensorManager with an IMU and a GPS mounted on the vehicle chassis
#  • Synchronous simulation/visualisation loop
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# ----------------------------------------------------------------------------- 
# 1. Chrono initialisation and a few general parameters
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath( chrono.GetChronoDataPath() + "vehicle/" )

# Simulation parameters -------------------------------------------------------
step_size      = 1.0e-3          # integration step [s]
render_steps   = 10              # Irrlicht draw frequency
sensor_steps   = 1               # sensor manager update frequency
sim_end        = 15.0            # end time [s]

# ----------------------------------------------------------------------------- 
# 2. The physical system (a wheeled‐vehicle system is implicitly created
#    inside the HMMWV_Full wrapper and returned through GetSystem())          
# -----------------------------------------------------------------------------
# Contact method and gravity
contact_method = chrono.ChContactMethod_NSC
my_hmmwv       = veh.HMMWV_Full( fixed              = False,
                                 contact_method     = contact_method,
                                 brake_type         = veh.BrakeType_SHAFTS )

# Initial vehicle state --------------------------------------------------------
init_loc  = chrono.ChVectorD( 0, 0, 1.0 )           # initial position
init_rot  = chrono.ChQuaternionD( 1,0,0,0 )         # (no rotation)
init_pos  = chrono.ChCoordsysD( init_loc, init_rot )

my_hmmwv.SetInitPosition( init_pos )
my_hmmwv.SetChassisFixed( False )
my_hmmwv.SetPowertrainType( veh.PowertrainModelType_SIMPLE )
my_hmmwv.SetDriveType(      veh.DriveType_4WD )
my_hmmwv.SetTireType(       veh.TireModelType_TMEASY )
my_hmmwv.Initialize()

# Handy pointer to the Chrono system that contains every rigid body
sys = my_hmmwv.GetSystem()

print("Vehicle mass  : {:.1f} kg".format( my_hmmwv.GetVehicle().GetMass() ))
print("Total bodies  : {}".format( sys.Get_bodylist().size() ))

# ----------------------------------------------------------------------------- 
# 3. Terrain subsystem
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction   ( 0.9 )
patch_mat.SetRestitution( 0.01 )

patch = terrain.AddPatch( patch_mat,
                          chrono.ChVectorD(0,0,0),
                          chrono.ChVectorD(0,0,1),
                          300.0,   # length
                          300.0 )  # width
patch.SetTexture( chrono.GetChronoDataFile("textures/soil.jpg"), 200, 200 )
terrain.Initialize()

# ----------------------------------------------------------------------------- 
# 4. Irrlicht visual interface + driver
# -----------------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(
        my_hmmwv.GetVehicle(),
        "HMMWV – IMU & GPS demo",
        chrono.dimension2du(1280, 720),
        False,                       # fullscreen
        True )                       # shadows

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera( chrono.ChVectorD( 0.0, 1.6, 4.0 ), 6.0 )
app.EnableHUD( True )
app.SetTimestep( step_size )

# Interactive driver (uses the Irrlicht window events)
driver = veh.ChIrrGuiDriver( app )
driver.Initialize()

# ----------------------------------------------------------------------------- 
# 5. Sensor manager + sensors
# -----------------------------------------------------------------------------
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight( chrono.ChVectorF(100,100,100), chrono.ChColor(1,1,1), 500 )

# Shared settings
imu_rate = 200.0             # Hz
gps_rate = 20.0              # Hz
lag      = 0.0               # sensor output lag
collect  = step_size         # collection window

# IMU -------------------------------------------------------------------------
imu_offset = chrono.ChFrameD( chrono.ChVectorD(0, 0, 0), chrono.QUNIT )
imu = sens.ChIMUSensor( my_hmmwv.GetChassisBody(),
                        imu_rate, lag, collect, imu_offset )

# Raw IMU update → make data accessible in Python
f_imu_acc = sens.ChFilterIMUAccess()
imu.PushFilter( f_imu_acc )

manager.AddSensor( imu )

# GPS -------------------------------------------------------------------------
gps_offset = chrono.ChFrameD( chrono.ChVectorD(0, 0, 0), chrono.QUNIT )
gps = sens.ChGPSSensor( my_hmmwv.GetChassisBody(),
                        gps_rate, lag, collect, gps_offset )

f_gps_acc = sens.ChFilterGPSAccess()
gps.PushFilter( f_gps_acc )

manager.AddSensor( gps )

# ----------------------------------------------------------------------------- 
# 6. Simulation loop
# -----------------------------------------------------------------------------
step_number = 0
while (app.GetDevice().run() and sys.GetChTime() < sim_end):

    time = sys.GetChTime()

    # a) Render scene ---------------------------------------------------------
    if step_number % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    # b) Driver inputs --------------------------------------------------------
    driver_inputs = driver.GetInputs()

    # c) Synchronize all modules ---------------------------------------------
    driver.Synchronize ( time )
    terrain.Synchronize( time )
    my_hmmwv.Synchronize( time, driver_inputs, terrain )
    app.Synchronize( "", driver_inputs )

    # d) Sensor manager (called at a higher rate if desired) ------------------
    if step_number % sensor_steps == 0:
        manager.Update()

        # Access the most recent IMU and GPS data
        imu_data = f_imu_acc.GetMostRecentBuffer()
        if imu_data:
            imu_read = imu_data[0]          # newest sample
            # Example: print vertical acceleration every 0.1 s
            if step_number % int(0.1/step_size) == 0:
                print("t={:6.2f}s  IMU acc_z={:+8.2f}  ang_vel_z={:+7.2f}"
                      .format(time,
                              imu_read.Accel[2],
                              imu_read.Gyro[2]) )

        gps_data = f_gps_acc.GetMostRecentBuffer()
        if gps_data:
            gps_read = gps_data[0]
            # Example: print latitude / longitude occasionally
            if step_number % int(0.5/step_size) == 0:
                print("        GPS lat={:+.6f} lon={:+.6f} alt={:+.2f}"
                      .format(gps_read.Latitude,
                              gps_read.Longitude,
                              gps_read.Altitude) )

    # e) Advance simulation ---------------------------------------------------
    driver.Advance ( step_size )
    terrain.Advance( step_size )           # does nothing but kept for symmetry
    my_hmmwv.Advance( step_size )
    app.Advance  ( step_size )             # Irrlicht camera & HUD
    step_number += 1