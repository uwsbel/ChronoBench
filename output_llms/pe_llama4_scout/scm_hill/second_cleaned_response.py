import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import numpy as np
import math






obstacles = []
for _ in range(5):
    obstacle_body = chrono.ChBody()
    obstacle_body.SetPos(chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), 0))
    obstacle_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
    obstacle_body.AddVisualShape(obstacle_shape)
    vehicle.GetSystem().Add(obstacle_body)
    obstacles.append(obstacle_body)


sensor_manager = sensor.SensorManager(vehicle.GetSystem())


lidar_sensor = sensor.ChLidarSensor(chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(math.pi / 2)))
lidar_sensor.SetPointsDistribution(sensor.ChLidarSensor.PolarDistribution(360, 1))
lidar_sensor.SetScanRate(20)
lidar_sensor.SetVisualization(True)
lidar_sensor.SetFilter(sensor.ChLidarSensor.NoFilter())
sensor_manager.AddSensor(lidar_sensor)




while vis.Run() :
    

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    sensor_manager.Update()