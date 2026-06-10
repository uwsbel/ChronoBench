"""Static NSC lidar scene with a box-mounted 3D lidar and added 2D lidar.

The simulation replaces a mesh payload with a textured rigid box, attaches both
lidar sensors to that box, and visualizes raw depth plus point-cloud streams.
The box is fixed in a Z-up world so the sensor geometry can be inspected without
motion-induced ambiguity.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === named geometry, timing, and sensor parameters
time_step = 1e-3
sim_end = 4.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

side = 1.0
box_density = 1000.0
box_center = chrono.ChVector3d(0.0, 0.0, side / 2.0)
lidar_rate = 5.0
lidar_collection_window = 1.0 / lidar_rate
lidar_3d_h_samples = 800
lidar_3d_v_samples = 300
lidar_2d_h_samples = 800
lidar_2d_v_samples = 1


# === System & Materials === NSC rigid scene with collision geometry for OptiX
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)


# === Bodies === fixed textured box replaces the mesh body and carries the lidars
box = chrono.ChBodyEasyBox(side, side, side, box_density, True, True, mat)
box.SetName("box_lidar_mount")
box.SetPos(box_center)
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
lidar_3d_marker = chrono.ChVisualShapeBox(0.28, 0.16, 0.12)
lidar_3d_marker.SetColor(chrono.ChColor(0.02, 0.02, 0.02))
box.AddVisualShape(lidar_3d_marker, chrono.ChFramed(chrono.ChVector3d(-0.18, 0.0, side / 2.0 + 0.06), chrono.QUNIT))
lidar_2d_marker = chrono.ChVisualShapeBox(0.36, 0.08, 0.10)
lidar_2d_marker.SetColor(chrono.ChColor(0.0, 0.8, 0.1))
box.AddVisualShape(lidar_2d_marker, chrono.ChFramed(chrono.ChVector3d(0.22, 0.0, side / 2.0 + 0.08), chrono.QUNIT))
beam_x = chrono.ChVisualShapeCylinder(0.01, 4.0)
beam_x.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
box.AddVisualShape(beam_x, chrono.ChFramed(chrono.ChVector3d(0.22, 0.0, side / 2.0 + 0.17), chrono.QuatFromAngleY(chrono.CH_PI_2)))
beam_y = chrono.ChVisualShapeCylinder(0.01, 4.0)
beam_y.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
box.AddVisualShape(beam_y, chrono.ChFramed(chrono.ChVector3d(0.22, 0.0, side / 2.0 + 0.17), chrono.QuatFromAngleX(-chrono.CH_PI_2)))
sys.Add(box)

floor = chrono.ChBodyEasyBox(8.0, 8.0, 0.05, box_density, True, True, mat)
floor.SetName("floor")
floor.SetPos(chrono.ChVector3d(0.0, 0.0, -0.025))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)


# === Sensor Manager === lidar sensors are attached to the actual box body
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 5.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    20.0,
)
box_body = box  # cache: real protagonist body reused for both lidar attachments

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-0.18, 0.0, side / 2.0 + 0.18),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)

lidar = sens.ChLidarSensor(
    box_body,
    lidar_rate,
    lidar_offset_pose,
    lidar_3d_h_samples,
    lidar_3d_v_samples,
    2.0 * chrono.CH_PI,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 6.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("3D Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(lidar_collection_window)
lidar.SetOffsetPose(lidar_offset_pose)
lidar.PushFilter(sens.ChFilterVisualize(lidar_3d_h_samples, lidar_3d_v_samples, "Raw 3D Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

lidar_2d_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.22, 0.0, side / 2.0 + 0.17),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
lidar_2d = sens.ChLidarSensor(
    box_body,
    lidar_rate,
    lidar_2d_offset_pose,
    lidar_2d_h_samples,
    lidar_2d_v_samples,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0.0)
lidar_2d.SetCollectionWindow(lidar_collection_window)
lidar_2d.SetOffsetPose(lidar_2d_offset_pose)
lidar_2d.PushFilter(sens.ChFilterVisualize(lidar_2d_h_samples, lidar_2d_v_samples, "Raw 2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)


# === Visualization === Irrlicht review window built separately from sensor output
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Box-Mounted Lidar Sensors")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -5.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main Loop === render once per frame, update lidars once per physics step
frame = 0


try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()
            di_buffer = lidar.GetMostRecentDIBuffer()
            if di_buffer.HasData():
                pass  # guard: only process lidar depth after data exists
            di_buffer_2d = lidar_2d.GetMostRecentDIBuffer()
            if di_buffer_2d.HasData():
                pass  # guard: 2D lidar depth can be empty before first tick
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid sensor state
    traceback.print_exc()
    raise
finally:
    manager.Update()


# === Post-processing === assemble only review artifacts, then strip for scoring
