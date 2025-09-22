import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import matplotlib.pyplot as plt




offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),
                                 10,
                                 offset_pose,
                                 sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)


gps = sens.ChGPSSensor(vehicle.GetChassisBody(),
                       10,
                       offset_pose,
                       chrono.ChVector3d(-89.400, 43.070, 260.0),
                       sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)



gps_data = []

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    if step_number % log_steps == 0:
        
        gps_coor = gps.GetMostRecentGPSBuffer().GetGPSData()
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])
    
    
    driver.SetThrottle(0.5)
    driver.SetSteering(0.6)
    driver_inputs = driver.GetInputs()
    
    

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


latitudes = [data[0] for data in gps_data]
longitudes = [data[1] for data in gps_data]

plt.figure(figsize=(10, 6))
plt.plot(longitudes, latitudes)
plt.title('GPS Trajectory')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.grid(True)
plt.show()

print("GPS Data: ", gps_data)