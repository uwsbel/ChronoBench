"""
Fixed NSC scene with a Wavefront OBJ mesh sensed by an orbiting lidar.

The script creates one fixed mesh body from a bundled OBJ file, attaches a
Chrono::Sensor lidar to that body, visualizes depth and point-cloud data,
saves point-cloud output, and prints lidar buffer summaries while the sensor
offset pose orbits around the mesh.
"""

import math
import traceback

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === named parameters keep sensor timing and geometry reviewable
TIME_STEP = 1e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MESH_FILE = "vehicle/hmmwv/hmmwv_chassis.obj"
MESH_SCALE = 2.0
LIDAR_UPDATE_RATE = 5.0
HORIZONTAL_SAMPLES = 800
VERTICAL_SAMPLES = 32
HORIZONTAL_FOV = 2.0 * chrono.CH_PI
MAX_VERT_ANGLE = chrono.CH_PI / 12.0
MIN_VERT_ANGLE = -chrono.CH_PI / 6.0
MAX_RANGE = 100.0
SAMPLE_RADIUS = 2
DIVERGENCE_ANGLE = 0.003
ORBIT_RADIUS = 5.0
ORBIT_HEIGHT = 1.0
ORBIT_RATE = 0.6


def make_lidar_pose(time_value):
    """Return the mesh-relative lidar pose for a circular orbit around it."""
    angle = time_value * ORBIT_RATE
    return chrono.ChFramed(
        chrono.ChVector3d(
            -ORBIT_RADIUS * math.cos(angle),
            -ORBIT_RADIUS * math.sin(angle),
            ORBIT_HEIGHT,
        ),
        chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1)),
    )


def main():
    # === System & mesh body === fixed mesh scene: no moving contacts are required
    system = chrono.ChSystemNSC()

    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(MESH_FILE), False, True)
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(MESH_SCALE))

    mesh_shape = chrono.ChVisualShapeTriangleMesh()
    mesh_shape.SetMesh(mesh)
    mesh_shape.SetName("Fixed Wavefront Mesh")
    mesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetName("fixed_wavefront_mesh")
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(mesh_shape)
    mesh_body.SetFixed(True)
    system.Add(mesh_body)

    mesh_pos = mesh_body.GetPos()  # cache: mesh center reused for cameras and logging

    # === Sensor manager & lidar === lidar filters provide depth/point-cloud views and saved data
    manager = sens.ChSensorManager(system)

    lidar = sens.ChLidarSensor(
        mesh_body,
        LIDAR_UPDATE_RATE,
        make_lidar_pose(0.0),
        HORIZONTAL_SAMPLES,
        VERTICAL_SAMPLES,
        HORIZONTAL_FOV,
        MAX_VERT_ANGLE,
        MIN_VERT_ANGLE,
        MAX_RANGE,
        sens.LidarBeamShape_RECTANGULAR,
        SAMPLE_RADIUS,
        DIVERGENCE_ANGLE,
        DIVERGENCE_ANGLE,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("Orbiting Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

    if hasattr(sens, "ChFilterLidarNoiseXYZI"):
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    else:
        print("ChFilterLidarNoiseXYZI unavailable in this PyChrono build; using deterministic lidar samples")
    lidar.PushFilter(sens.ChFilterVisualize(HORIZONTAL_SAMPLES, VERTICAL_SAMPLES, "Raw Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_points/"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)


    # === Irrlicht visualization === standard window renders the fixed mesh and orbiting sensor review
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Orbiting Lidar over Fixed OBJ Mesh")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), mesh_pos)
    vis.AddTypicalLights()
    vis.AddGrid(
        1.0,
        1.0,
        20,
        20,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.0), chrono.QUNIT),
        chrono.ChColor(0.4, 0.4, 0.4),
    )


    # === Main loop === update lidar orbit, print buffers, then step the fixed scene
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                lidar_pose = make_lidar_pose(sim_time)
                lidar.SetOffsetPose(lidar_pose)

                xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
                has_xyzi = xyzi_buffer.HasData()  # guard: skip frames before the lidar fills a buffer
                max_xyzi = float("nan")
                if has_xyzi:
                    xyzi_data = xyzi_buffer.GetXYZIData()
                    max_xyzi = float(np.max(xyzi_data))
                    print(
                        "XYZI buffer received from lidar. Lidar resolution: {0}x{1}".format(
                            xyzi_buffer.Width,
                            xyzi_buffer.Height,
                        )
                    )
                    print("Max Value: {0}".format(max_xyzi))
                else:
                    print("No XYZI buffer available from lidar at time {0:.3f}".format(sim_time))


                manager.Update()
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError, OSError) as exc:  # sensor/render failures or bad file output
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
