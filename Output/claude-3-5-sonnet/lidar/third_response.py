import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    system = chrono.ChSystemNSC()

    # Create the vehicle
    vehicle = veh.ARTCar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVectord(0, 0, 1), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # Add vehicle to the system
    vehicle.GetSystem().AddBodyToSystem(vehicle.GetChassisBody())

    # Create and initialize the driver
    driver = veh.ChDriver(vehicle.GetVehicle())

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChVectord(0, 0, 0), chrono.ChVectord(0, 0, 1),
                             300, 300)
    patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(system)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters to the lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d =  sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        1,                      # only 1 vertical channel for 2D lidar
        horizontal_fov,         # Horizontal field of view
        0.0,                    # Maximum vertical field of view
        0.0,                    # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    
    # Add filters to the 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Add third person camera
    cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 3), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        30.0,                      # Update rate in Hz
        cam_offset_pose,           # Offset pose
        1280,                      # Image width
        720,                       # Image height
        1.608                      # Camera FOV
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Synchronize vehicle and terrain systems
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain.GetTerrain())
        terrain.Synchronize(ch_time)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)

        # Update sensor manager
        manager.Update()

        # Perform step of dynamics
        system.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Lidar parameters
# -----------------

noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_C_PI
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------

step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

main()