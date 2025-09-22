# =============================================================================
#  Modified PyChrono – Lidar demo
#  *  triangular mesh was replaced by a simple cube
#  *  two lidars are mounted on the cube: a regular 3-D scanner and a 2-D
#    (single-row) scanner
#  *  several small problems in the original file were fixed
# =============================================================================
import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


# ----------------------------------------------------------------------------- 
# USER-DEFINED PARAMETERS
# -----------------------------------------------------------------------------
#
# Lidar parameters ------------------------------------------------------------
noise_model        = "NONE"                         # "NONE" | "CONST_NORMAL_XYZI"
return_mode        = sens.LidarReturnMode_STRONGEST_RETURN
update_rate        = 5.0                            # [Hz]

horizontal_samples = 800                            # pixels
vertical_samples   = 300                            # channels  (the 3-D lidar)

horizontal_fov     = 2 * chrono.CH_PI              # 360 [rad]
max_vert_angle     =  chrono.CH_PI / 12.0          # 15  [rad]
min_vert_angle     = -chrono.CH_PI /  6.0          # –30 [rad]

lag                = 0.0
collection_time    = 1.0 / update_rate
sample_radius      = 2
divergence_angle   = 0.003                          # [rad]

# 2-D lidar specific -----------------------------------------------------------
vertical_samples_2d = 1                             # one vertical channel

# Simulation parameters --------------------------------------------------------
step_size          = 1e-3
end_time           = 40.0

vis                = True
out_dir            = "SENSOR_OUTPUT/"
save               = False      # <— left here for completeness, not used below
# -----------------------------------------------------------------------------


def build_lidar(parent_body,
                offset_pose,
                h_samples,
                v_samples,
                name_suffix=""):

    """ Helper that creates a lidar, adds the standard filter graph,
        and returns the sensor object. """

    lidar = sens.ChLidarSensor(
        parent_body,                 # body the sensor is attached to
        update_rate,                 # update (scanning) rate  [Hz]
        offset_pose,                 # relative pose wrt parent
        h_samples,                   # horizontal samples
        v_samples,                   # vertical channels
        horizontal_fov,              # horizontal FOV
        max_vert_angle,              # max vertical FOV
        min_vert_angle,              # min vertical FOV
        100.0,                       # max range [m]
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,            # second divergence for dual-gaussian, keep same
        return_mode
    )

    lidar.SetName(f"Lidar Sensor{name_suffix}")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # ---------------- Filter Graph -------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        # Standard deviations on XYZ (m) and I (intensity)
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualize(
                h_samples, v_samples, f"Raw Lidar Depth Data{name_suffix}"
            )
        )

    # Depth/Intensity access
    lidar.PushFilter(sens.ChFilterDIAccess())

    # XYZI point-cloud conversion
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640, 480, 1.0, f"Lidar Point Cloud{name_suffix}"
            )
        )

    # XYZI access
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    return lidar


def main():
    # -------------------------------------------------------------------------
    # 1.  Create the Chrono-physics system
    # -------------------------------------------------------------------------
    sys = chrono.ChSystemNSC()

    # -------------------------------------------------------------------------
    # 2.  Add a simple cube that will be scanned
    # -------------------------------------------------------------------------
    side      = 2.0                                        # cube edge length [m]
    density   = 1000                                       # [kg/m³] – irrelevant (body is fixed)

    cube_body = chrono.ChBodyEasyBox(side, side, side,     # size
                                     density,
                                     True,  True)          # visualize & collide

    cube_body.SetPos(chrono.ChVector3d(0, 0, 0))
    cube_body.SetFixed(True)

    # add a simple texture to the cube
    texture = chrono.ChVisualShapeTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    cube_body.AddVisualShape(texture)

    sys.Add(cube_body)

    # -------------------------------------------------------------------------
    # 3.  Create the sensor manager
    # -------------------------------------------------------------------------
    manager = sens.ChSensorManager(sys)

    # (optional) – set the intensity of the environment light
    manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 500)

    # -------------------------------------------------------------------------
    # 4.  Attach lidars
    # -------------------------------------------------------------------------
    # Common initial offset pose (will be changed through the sim loop)
    offset_pose_3d = chrono.ChFrameD(
        chrono.ChVector3d(-12, 0, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)                   # no rotation initially
    )

    # 4-A) Full 3-D lidar ------------------------------------------------------
    lidar_3d = build_lidar(cube_body,
                           offset_pose_3d,
                           horizontal_samples,
                           vertical_samples,
                           "_3D")

    manager.AddSensor(lidar_3d)

    # 4-B) Additional 2-D lidar (single vertical channel) ----------------------
    #      mounted at the same place, could be moved independently
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVector3d(-12, 0, 1.5),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = build_lidar(cube_body,
                           offset_pose_2d,
                           horizontal_samples,
                           vertical_samples_2d,
                           "_2D")

    manager.AddSensor(lidar_2d)

    # -------------------------------------------------------------------------
    # 5.  Simulation loop
    # -------------------------------------------------------------------------
    orbit_radius = 10.0
    orbit_rate   = 0.1

    render_time  = 0.0
    t_wall_start = time.time()

    while sys.GetChTime() < end_time:

        ch_time = sys.GetChTime()

        # make the 3-D lidar orbit around the cube
        new_offset = chrono.ChFrameD(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1.0
            ),
            chrono.ChQuaternionD(                             # rotate the lidar as it flies
                chrono.ChRotationD(chrono.Q_from_AngAxis(ch_time * orbit_rate,
                                                          chrono.ChVector3d(0, 0, 1)))
            )
        )
        lidar_3d.SetOffsetPose(new_offset)

        # 2-D lidar stays still (for variety you could also move it)

        # update all sensors (renders, filters, etc.)
        manager.Update()

        # example: access latest XYZI buffer from 3-D lidar
        xyzi_buf = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buf.HasData():
            xyzi_data = xyzi_buf.GetXYZIData()
            print(f"[{ch_time:6.2f}s] 3-D lidar  XYZI received – "
                  f"resolution {xyzi_buf.Width}×{xyzi_buf.Height}  "
                  f"max-val {np.max(xyzi_data):.4f}")

        # step the dynamics
        sys.DoStepDynamics(step_size)

    print("Simulation time: {:.2f} s   |   Wall-clock time: {:.2f} s"
          .format(end_time, time.time() - t_wall_start))


# ----------------------------------------------------------------------------- 
#  Run the demo
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    main()