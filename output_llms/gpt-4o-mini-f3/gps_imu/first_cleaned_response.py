import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


hmmwv_path = chrono.GetChronoDataPath() + 'vehicle/hmmwv'


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


vehicle = veh.HMMWV_Full()(True)
vehicle.SetDataPath(hmmwv_path + '/data')
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetFixed(False)


vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle.SetDrivetrainType(veh.DrivetrainType_SHAFTS)
vehicle.SetTerrainType(veh.TerrainType_PLANE)


vehicle.SetTireType(veh.TireType_TMEASY)
vehicle.SetTireStepSize(1e-3)


vehicle.SetMaxWheelLoadRatio(0.9)


vehicle.Initialize()


vehicle_interface = veh.ChWheeledVehicleInterfaceChrono()
vehicle_interface.SetVehicle(vehicle.GetVehicle())




sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


offset_pose = chrono.ChPose()
offset_pose.pos = chrono.ChVector3d(-10, 0, 1)
imu_sensor = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),                     
                                        10,        
                                        offset_pose,  
                                        sens.ChNoiseNone())  
imu_sensor.SetName("IMU Sensor")
imu_sensor.SetLag(0)
imu_sensor.SetCollectionWindow(0)

imu_sensor.PushFilter(sens.ChFilterAccelAccess())

sensor_manager.AddSensor(imu_sensor)


gps_sensor = sens.ChGPSSensor(vehicle.GetChassisBody(),                     
                              10,        
                              offset_pose,  
                              chrono.ChVector3d(-89.400, 43.070, 260.0),  
                              sens.ChNoiseNone())  
gps_sensor.SetName("GPS Sensor")
gps_sensor.SetLag(0)
gps_sensor.SetCollectionWindow(0)

gps_sensor.PushFilter(sens.ChFilterGPSAccess())

sensor_manager.AddSensor(gps_sensor)




vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 720)
vis.SetChanels(32)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())




time_step = 1e-3             
end_time = 100.0             
time_interval = 1.0          


vehicle.SetTimeStep(time_step)


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = vehicle.GetSoftStopTime()
while vis.Run() :
    time = vehicle.GetTime()
    
    if time - time_interval >= 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        time_interval += 1

    
    sensor_manager.Update()
    
    vehicle.Synchronize(time)
    sensor_manager.Synchronize(time)
    
    vehicle.Advance(time_step)
    sensor_manager.Advance(time_step)
    
    realtime_timer.Spin(step)