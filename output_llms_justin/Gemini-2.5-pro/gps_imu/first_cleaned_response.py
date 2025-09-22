import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import os
import math




chrono.SetChronoDataPath(os.environ.get("CHRONO_DATA_DIR", "../../chrono_data/"))
vehicle.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))
sens.SetSensorDataPath(os.path.join(chrono.GetChronoDataPath(), 'sensor/'))




step_size = 2e-3  
time_end = 100    


init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0) 



gps_reference = chrono.ChVectorD(42.2808, -83.7430, 250.0) 


imu_update_rate = 100  
gps_update_rate = 10   




print("Initializing Chrono system...")


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81)) 


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)






print("Creating HMMWV vehicle...")
my_hmmwv = vehicle.HMMWV_Full("myHMMWV")
my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC) 
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
my_hmmwv.SetPowertrainType(vehicle.PowertrainModelType_SHAFTS) 
my_hmmwv.SetDriveType(vehicle.DrivelineTypeWV_AWD)
my_hmmwv.SetSteeringType(vehicle.SteeringTypeWV_PITMAN_ARM)
my_hmmwv.SetTireType(vehicle.TireModelType_TMEASY) 

my_hmmwv.SetTireStepSize(step_size)
my_hmmwv.Initialize()


my_hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH) 
my_hmmwv.SetTireVisualizationType(vehicle.VisualizationType_MESH)   


veh = my_hmmwv.GetVehicle()
chassis_body = veh.GetChassisBody()
print(f"Initial Vehicle Mass: {veh.GetMass()} kg")


print("Creating terrain...")
terrain = vehicle.RigidTerrain(my_system)
patch_mat = chrono.ChMaterialSurfaceNSC() 
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat,
                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 
                         200, 200) 
patch.SetTexture(vehicle.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


print("Creating Irrlicht visualization...")



app = vehicle.ChWheeledVehicleIrrApp(veh, "HMMWV Sensor Demo")
app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5) 

app.SetTimestep(step_size)


app.AssetBindAll()
app.AssetUpdateAll()


print("Creating driver system...")

driver = vehicle.ChIrrGuiDriver(app)

driver.SetSteeringControllerFile(vehicle.GetDataFile("hmmwv/SteeringController.json"))
driver.SetSpeedControllerFile(vehicle.GetDataFile("hmmwv/SpeedController.json"))
driver.Initialize()
app.SetDriver(driver) 


print("Creating sensor manager and sensors...")
sensor_manager = sens.ChSensorManager(my_system)









sensor_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.3), chrono.Q_from_AngAxis(0, chrono.VECT_Y))


print(f"  Adding IMU sensor with update rate: {imu_update_rate} Hz")
imu_sensor = sens.ChIMUSensor(
    chassis_body,          
    imu_update_rate,       
    sensor_offset_pose,    
    sens.ChNoiseNone()     
)
imu_sensor.SetName("IMU_Sensor")
imu_sensor.SetLag(0.0) 
imu_sensor.SetCollectionWindow(0.0) 
sensor_manager.AddSensor(imu_sensor)


print(f"  Adding GPS sensor with update rate: {gps_update_rate} Hz")
gps_sensor = sens.ChGPSSensor(
    chassis_body,          
    gps_update_rate,       
    sensor_offset_pose,    
    gps_reference,         
    sens.ChNoiseNone()     
)
gps_sensor.SetName("GPS_Sensor")
gps_sensor.SetLag(0.0)
gps_sensor.SetCollectionWindow(0.0)
sensor_manager.AddSensor(gps_sensor)







print("\nStarting simulation loop...")
realtime_timer = chrono.ChRealtimeStepTimer()
current_time = 0

frame_counter = 0
output_frequency = int(1.0 / step_size / 2) 

while app.GetDevice().run():
    current_time = my_system.GetChTime()
    if current_time >= time_end:
        break

    
    
    driver_inputs = driver.GetInputs() 

    
    
    driver.Synchronize(current_time)
    
    terrain.Synchronize(current_time)
    
    my_hmmwv.Synchronize(current_time, driver_inputs, terrain)
    
    
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs.m_steering, driver_inputs.m_throttle, driver_inputs.m_braking)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size) 
    app.Advance(step_size)      

    
    my_system.DoStepDynamics(step_size)

    
    sensor_manager.Update()

    
    if frame_counter % output_frequency == 0:
        print(f"\n--- Time: {current_time:.3f} s ---")
        print(f"Vehicle Position: {veh.GetPos().x:.2f}, {veh.GetPos().y:.2f}, {veh.GetPos().z:.2f}")
        print(f"Vehicle Speed: {veh.GetSpeed():.2f} m/s")
        print(f"Vehicle Mass: {veh.GetMass()} kg") 

        
        imu_buffer = imu_sensor.GetMostRecentBufferIMU()
        if imu_buffer.HasData():
            imu_data = imu_buffer.GetLastData() 
            print(f"  IMU Accel: ({imu_data.Accel[0]:.2f}, {imu_data.Accel[1]:.2f}, {imu_data.Accel[2]:.2f}) m/s^2")
            print(f"  IMU Gyro:  ({imu_data.Gyro[0]:.2f}, {imu_data.Gyro[1]:.2f}, {imu_data.Gyro[2]:.2f}) rad/s")
            
        else:
            print("  IMU: No new data")

        
        gps_buffer = gps_sensor.GetMostRecentBufferGPS()
        if gps_buffer.HasData():
            gps_data = gps_buffer.GetLastData() 
            print(f"  GPS Coords (Lat,Lon,Alt): ({gps_data.Latitude:.6f}, {gps_data.Longitude:.6f}, {gps_data.Altitude:.2f})")
            print(f"  GPS Time: {gps_data.Time}")
        else:
            print("  GPS: No new data")

    
    app.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) 
    app.DrawAll()
    app.EndScene()

    
    

    frame_counter += 1




print("\nSimulation ended.")