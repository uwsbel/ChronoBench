import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    vehicle = veh.ARTcar()
    vehicle.Initialize()
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisCollisionType(chrono.ChMaterialSurface.SURF_WHEEL)
    vehicle.SetSuspensionType(veh.SuspensionType_MCpherson)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)))
    vehicle.SetInitFwdVel(5)
    vehicle.SetPowertrainType(veh.PowertrainModelType_SimpleMap)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)))
    vehicle.SetInitFwdVel(5)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    terrain.SetContactFrictionCoefficient(0.9)
    terrain.SetContactRestitutionCoefficient(0.01)
    terrain.SetContactMaterial(chrono.ChMaterialSurface.SURF_CONCRETE)
    terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize(vehicle.GetSystem(), 200, 200, 0, 0, 20)

    # Create the vehicle driver
    driver = veh.ChDriver(vehicle)
    driver.Initialize()
    driver.SetSteeringControllerType(veh.SteeringControllerType_PID)
    driver.SetSpeedControllerType(veh.SpeedControllerType_PID)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(vehicle.GetSystem())

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),
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

    # Add filters and push the lidar to the sensor manager
    # ... (same as before)

    # Create 2D lidar sensor
    lidar_2d =  sens.ChLidarSensor(
        vehicle.GetChassisBody(),
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

    # Add filters and push the 2D lidar to the sensor manager
    # ... (same as before)

    # Add a third person camera
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        1000,
        offset_pose,
        horizontal_fov,
        math.radians(60),
        0.01,
        1000
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update vehicle, driver, and terrain
        vehicle.Synchronize(ch_time)
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        vehicle.Advance(step_size)
        driver.Advance(step_size)
        terrain.Advance(step_size)

        # Get the current time of the simulation
        ch_time = vehicle.GetSystem().GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# ... (same as before)

main()