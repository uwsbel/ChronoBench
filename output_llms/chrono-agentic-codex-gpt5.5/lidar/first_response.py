"""Fixed Wavefront mesh with an orbiting lidar sensor.

This PyChrono NSC scene loads a triangular OBJ mesh as a fixed visual body,
attaches a lidar to that body through a ChSensorManager, and updates the lidar
offset pose so it scans the mesh from an orbiting viewpoint. The lidar filter
graph visualizes raw depth and point-cloud data, saves point-cloud frames, and
prints guarded depth/intensity and XYZI buffer summaries at every simulation
step.
"""

import math
import traceback

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
# Simulation, mesh, lidar, and review cadence values are precomputed once.
STEP_SIZE = 0.01
END_TIME = 3.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
LIDAR_UPDATE_RATE = 5.0
HORIZONTAL_SAMPLES = 320
VERTICAL_SAMPLES = 32
HORIZONTAL_FOV = 2.0 * chrono.CH_PI
MAX_VERT_ANGLE = chrono.CH_PI / 12.0
MIN_VERT_ANGLE = -chrono.CH_PI / 6.0
MAX_RANGE = 100.0
ORBIT_RADIUS = 8.0
ORBIT_HEIGHT = 1.0
ORBIT_RATE = 2.0
LIDAR_MARKER_RADIUS = 0.18
MESH_SCALE = 2.0


def lidar_pose(time_value):
    """Return the orbiting lidar offset pose relative to the fixed mesh body."""
    angle = ORBIT_RATE * time_value
    sensor_x = -ORBIT_RADIUS * math.cos(angle)
    sensor_y = -ORBIT_RADIUS * math.sin(angle)
    yaw_to_center = angle
    return chrono.ChFramed(
        chrono.ChVector3d(sensor_x, sensor_y, ORBIT_HEIGHT),
        chrono.QuatFromAngleZ(yaw_to_center),
    )


def print_lidar_buffers(lidar_sensor, time_value):
    """Print available lidar buffer summaries after checking for valid data."""
    di_buffer = lidar_sensor.GetMostRecentDIBuffer()  # cache: reused for guard and data access
    if di_buffer.HasData():  # guard: lidar buffer is empty before first sensor tick
        di_data = di_buffer.GetDIData()
        print(
            "time={:.3f} DI shape={} max_depth={:.4f} max_intensity={:.4f}".format(
                time_value,
                di_data.shape,
                float(np.max(di_data[:, :, 0])),
                float(np.max(di_data[:, :, 1])),
            )
        )
    else:
        print("time={:.3f} DI buffer has no data".format(time_value))

    xyzi_buffer = lidar_sensor.GetMostRecentXYZIBuffer()  # cache: reused for guard and data access
    if xyzi_buffer.HasData():  # guard: point cloud appears after depth conversion filter runs
        xyzi_data = xyzi_buffer.GetXYZIData()
        print(
            "time={:.3f} XYZI shape={} max_value={:.4f}".format(
                time_value,
                xyzi_data.shape,
                float(np.max(xyzi_data)),
            )
        )
    else:
        print("time={:.3f} XYZI buffer has no data".format(time_value))


def main():
    """Build and run the mesh-lidar simulation."""
    # === System & gravity ===
    # A fixed visual mesh is sensed without contact, so no collision system is needed.
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    # === Fixed OBJ mesh body ===
    # Load a bundled Wavefront OBJ as a triangular visual mesh for lidar sensing.
    mesh_path = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(mesh_path, False, True)
    mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(MESH_SCALE))

    mesh_shape = chrono.ChVisualShapeTriangleMesh()
    mesh_shape.SetMesh(mesh)
    mesh_shape.SetName("Fixed Wavefront OBJ Mesh")
    mesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetName("fixed_obj_mesh")
    mesh_body.SetFixed(True)
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(mesh_shape)
    system.Add(mesh_body)
    target_body = mesh_body  # cache: fixed body reused by lidar, Irrlicht, and loop

    lidar_marker = chrono.ChBodyEasySphere(LIDAR_MARKER_RADIUS, 1000.0, True, False)
    lidar_marker.SetName("visible_lidar_marker")
    lidar_marker.SetFixed(True)
    lidar_marker.SetPos(lidar_pose(0.0).GetPos())
    lidar_marker.GetVisualShape(0).SetColor(chrono.ChColor(1.0, 0.05, 0.02))
    system.Add(lidar_marker)

    # === Sensor manager & lidar ===
    # The lidar filter graph visualizes, saves, and exposes host-access buffers.
    manager = sens.ChSensorManager(system)
    lidar = sens.ChLidarSensor(
        target_body,
        LIDAR_UPDATE_RATE,
        lidar_pose(0.0),
        HORIZONTAL_SAMPLES,
        VERTICAL_SAMPLES,
        HORIZONTAL_FOV,
        MAX_VERT_ANGLE,
        MIN_VERT_ANGLE,
        MAX_RANGE,
        sens.LidarBeamShape_RECTANGULAR,
        2,
        0.003,
        0.003,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("Orbiting Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

    if hasattr(sens, "ChFilterLidarNoiseXYZI"):
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    else:
        print("ChFilterLidarNoiseXYZI unavailable in this PyChrono binding; continuing without lidar noise filter")
    lidar.PushFilter(sens.ChFilterVisualize(HORIZONTAL_SAMPLES, VERTICAL_SAMPLES, "Raw Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_points/"))
    manager.AddSensor(lidar)

    # === Visualization ===
    # Irrlicht provides the review view; sensor visualization windows show lidar data.
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Orbiting Lidar Around Fixed OBJ Mesh")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, -12, 6), chrono.ChVector3d(0, 0, 0.6))
    vis.AddTypicalLights()
    vis.AddGrid(
        1.0,
        1.0,
        16,
        16,
        chrono.ChCoordsysd(),
        chrono.ChColor(0.35, 0.35, 0.35),
    )

    # === Main loop ===
    # The lidar pose is updated before each sensor-manager update.
    try:
        while vis.Run() and system.GetChTime() < END_TIME:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                current_time = system.GetChTime()
                current_pose = lidar_pose(current_time)  # cache: same orbit drives sensor and visible marker
                lidar.SetOffsetPose(current_pose)
                lidar_marker.SetPos(current_pose.GetPos())
                manager.Update()
                print_lidar_buffers(lidar, current_time)
                system.DoStepDynamics(STEP_SIZE)
                if system.GetChTime() >= END_TIME:
                    break
    except (RuntimeError, ValueError, OSError) as exc:  # rendering/sensor/runtime failures
        traceback.print_exc()
        raise exc
    finally:
        print("Simulation finished at t={:.3f}".format(system.GetChTime()))


if __name__ == "__main__":
    main()
