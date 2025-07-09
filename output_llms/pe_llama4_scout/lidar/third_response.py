import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math

# Define used variables
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

def main():
    # Create the ARTcar vehicle
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(veh.ChassisCollisionType_BOX)
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(0)))
    car.Initialize()

    # Create a driver for the vehicle
    driver = veh.ChDriver(car)
    driver.Initialize()

    # Create a rigid terrain
    terrain = veh.RigidTerrain(car.GetSystem())
    terrain_mat = chrono.ChMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.3)
    patch = terrain.AddPatch(terrain_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
    patch.SetTexture(chrono.GetChronoDataFile("textures/blue.png"), 10, 10)
    patch.SetColor(chrono.ChColor(0.4, 0.7, 0.4))
    terrain.Initialize()

    # Create the sensor manager
    manager = sens.ChSensorManager(car.GetSystem())

    # Create a lidar and add it to the sensor manager
    offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(), 
        update_rate, 
        offset_pose, 
        horizontal_samples, 
        vertical_samples, 
        horizontal_fov, 
        max_vert_angle, 
        min_vert_angle, 
        100.0, 
        sens.LidarBeamShape_RECTANGULAR, 
        sample_radius, 
        divergence_angle, 
        divergence_angle, 
        return_mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(), 
        update_rate, 
        offset_pose, 
        horizontal_samples, 
        1, 
        horizontal_fov, 
        0.0, 
        0.0, 
        100.0, 
        sens.LidarBeamShape_RECTANGULAR, 
        sample_radius, 
        divergence_angle, 
        divergence_angle, 
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Create a third-person camera
    camera = sens.ChCameraSensor(car.GetChassisBody())
    camera.SetName("Third Person Camera")
    camera.SetResolution(chrono.ChVector2i(640, 480))
    camera.SetFOV(chrono.CH_PI / 2)
    camera.SetNearPlane(0.1)
    camera.SetFarPlane(100)
    manager.AddSensor(camera)

    # Simulate system
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update sensor manager
        manager.Update()

        # Perform step of dynamics
        car.GetSystem().DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = car.GetSystem().GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()