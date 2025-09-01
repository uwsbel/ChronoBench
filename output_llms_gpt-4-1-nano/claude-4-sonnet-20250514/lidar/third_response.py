import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the vehicle system
    # -----------------
    # Create the vehicle, set parameters, and initialize
    artcar = veh.ARTcar()
    artcar.SetContactMethod(chrono.ChContactMethod_NSC)
    artcar.SetChassisCollisionType(veh.CollisionType_NONE)
    artcar.SetChassisFixed(False)
    artcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
    artcar.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    artcar.SetDriveType(veh.DrivelineTypeWV_AWD)
    artcar.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    artcar.SetTireType(veh.TireModelType_TMEASY)
    artcar.SetTireStepSize(step_size)
    artcar.Initialize()

    artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # Get the vehicle system
    mphysicalSystem = artcar.GetSystem()

    # ----------------------------------
    # Add a box to be sensed by lidars
    # ----------------------------------
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(10, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    # ----------------------------------
    # Create and initialize the driver
    # ----------------------------------
    driver = veh.ChInteractiveDriverIRR()

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # ----------------------------------
    # Create the terrain
    # ----------------------------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                            chrono.CSYSNORM, 
                            200.0, 200.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),    # Body lidar is attached to
        update_rate,                # Scanning rate in Hz
        offset_pose,                # Offset pose
        horizontal_samples,         # Number of horizontal samples
        vertical_samples,           # Number of vertical channels
        horizontal_fov,             # Horizontal field of view
        max_vert_angle,             # Maximum vertical field of view
        min_vert_angle,             # Minimum vertical field of view
        100.0,                      # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,              # Sample radius
        divergence_angle,           # Divergence angle
        divergence_angle,           # Divergence angle (again, typically same value)
        return_mode                 # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassisBody(),    # Body lidar is attached to
        update_rate,                # Scanning rate in Hz
        offset_pose,                # Offset pose
        horizontal_samples,         # Number of horizontal samples
        1,                          # only 1 vertical channel for 2D lidar
        horizontal_fov,             # Horizontal field of view
        0.0,                        # Maximum vertical field of view
        0.0,                        # Minimum vertical field of view
        100.0,                      # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,              # Sample radius
        divergence_angle,           # Divergence angle
        divergence_angle,           # Divergence angle (again, typically same value)
        return_mode                 # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # ------------------------------------------------
    # Create a third person camera and add it to the sensor manager
    # ------------------------------------------------
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 3), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        artcar.GetChassisBody(),    # Body camera is attached to
        30,                         # Scanning rate in Hz
        camera_offset_pose,         # Offset pose
        1280,                       # Image width
        720,                        # Image height
        chrono.CH_PI / 3           # Field of view
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0)
    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    # ---------------
    # Simulate system
    # ---------------
    render_time = 0
    t1 = time.time()

    while mphysicalSystem.GetChTime() < end_time:
        time_now = mphysicalSystem.GetChTime()

        # Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time_now)
        terrain.Synchronize(time_now)
        artcar.Synchronize(time_now, driver_inputs, terrain)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        artcar.Advance(step_size)

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Render step size
render_step_size = 1.0/50  # FPS = 50

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

if __name__ == "__main__":
    main()