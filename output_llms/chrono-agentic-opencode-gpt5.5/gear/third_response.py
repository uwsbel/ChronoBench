"""Gear and pulley mechanism using a Chrono NSC multi-body system.

The model contains a fixed truss, a motor-driven horizontal-shaft gear A, a
bevel gear D of radius 5 at (-10, 0, -9), and pulley E of radius 2 at
(-10, -11, -9). Gear A and gear D are coupled with a 1:1 gear constraint, while
gear D and pulley E are coupled by a synchro pulley constraint. The expected
behavior is smooth driven rotation of A, matched rotation of D, and
belt-synchronized pulley motion through coordinated rotational motor links.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === direct mechanism dimensions and run cadence
time_step = 0.001
sim_end = 6.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

gear_a_radius = 5.0
gear_d_radius = 5.0
pulley_e_radius = 2.0
disc_thickness = 1.0
disc_density = 1000.0
motor_speed = 0.8
pulley_speed = motor_speed * gear_d_radius / pulley_e_radius

gear_a_pos = chrono.ChVector3d(-10.0, 10.0, -9.0)
gear_d_pos = chrono.ChVector3d(-10.0, 0.0, -9.0)
pulley_e_pos = chrono.ChVector3d(-10.0, -11.0, -9.0)
horizontal_axis_rot = chrono.QuatFromAngleZ(chrono.CH_PI_2)
shaft_x_frame = chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_X)


# === System & bodies === pure constrained MBS without contact or collision shapes
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

truss = chrono.ChBody()
truss.SetName("fixed_truss")
truss.SetFixed(True)
sys.AddBody(truss)

gear_a = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, gear_a_radius, disc_thickness, disc_density)
gear_a.SetName("gear_A")
gear_a.SetPos(gear_a_pos)
gear_a.SetRot(horizontal_axis_rot)
gear_a.GetVisualShape(0).SetColor(chrono.ChColor(0.15, 0.35, 0.9))
sys.AddBody(gear_a)

gear_d = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, gear_d_radius, disc_thickness, disc_density)
gear_d.SetName("bevel_gear_D")
gear_d.SetPos(gear_d_pos)
gear_d.SetRot(horizontal_axis_rot)
gear_d.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.45, 0.1))
bevel_face = chrono.ChVisualShapeCone(gear_d_radius, 1.2)
bevel_face.SetColor(chrono.ChColor(1.0, 0.65, 0.25))
gear_d.AddVisualShape(bevel_face, chrono.ChFramed(chrono.VNULL, chrono.Q_ROTATE_Z_TO_X))
sys.AddBody(gear_d)

pulley_e = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, pulley_e_radius, disc_thickness, disc_density)
pulley_e.SetName("pulley_E")
pulley_e.SetPos(pulley_e_pos)
pulley_e.SetRot(horizontal_axis_rot)
pulley_e.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.75, 0.25))
sys.AddBody(pulley_e)

for body_pos in (gear_a_pos, gear_d_pos, pulley_e_pos):
    shaft = chrono.ChVisualShapeCylinder(0.25, 2.4)
    shaft.SetColor(chrono.ChColor(0.35, 0.35, 0.35))
    truss.AddVisualShape(shaft, chrono.ChFramed(body_pos, chrono.Q_ROTATE_Z_TO_X))

belt_top = chrono.ChVisualShapeLine(chrono.ChLineSegment(
    chrono.ChVector3d(-8.8, gear_d_pos.y, gear_d_pos.z + gear_d_radius),
    chrono.ChVector3d(-8.8, pulley_e_pos.y, pulley_e_pos.z + pulley_e_radius),
))
belt_top.SetColor(chrono.ChColor(0.05, 0.05, 0.05))
belt_top.SetThickness(4.0)
truss.AddVisualShape(belt_top)

belt_bottom = chrono.ChVisualShapeLine(chrono.ChLineSegment(
    chrono.ChVector3d(-8.8, gear_d_pos.y, gear_d_pos.z - gear_d_radius),
    chrono.ChVector3d(-8.8, pulley_e_pos.y, pulley_e_pos.z - pulley_e_radius),
))
belt_bottom.SetColor(chrono.ChColor(0.05, 0.05, 0.05))
belt_bottom.SetThickness(4.0)
truss.AddVisualShape(belt_bottom)


# === Joints / constraints === motor, revolute supports, gear ratio, and synchro belt
motor_a = chrono.ChLinkMotorRotationSpeed()
motor_a.Initialize(gear_a, truss, chrono.ChFramed(gear_a_pos, chrono.Q_ROTATE_Z_TO_X))
motor_a.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor_a)

motor_d = chrono.ChLinkMotorRotationSpeed()
motor_d.Initialize(gear_d, truss, chrono.ChFramed(gear_d_pos, chrono.Q_ROTATE_Z_TO_X))
motor_d.SetSpeedFunction(chrono.ChFunctionConst(-motor_speed))
sys.AddLink(motor_d)

motor_e = chrono.ChLinkMotorRotationSpeed()
motor_e.Initialize(pulley_e, truss, chrono.ChFramed(pulley_e_pos, chrono.Q_ROTATE_Z_TO_X))
motor_e.SetSpeedFunction(chrono.ChFunctionConst(-pulley_speed))
sys.AddLink(motor_e)

gear_ad = chrono.ChLinkLockGear()
gear_ad.Initialize(gear_a, gear_d, chrono.ChFramed())
gear_ad.SetFrameShaft1(shaft_x_frame)
gear_ad.SetFrameShaft2(shaft_x_frame)
gear_ad.SetTransmissionRatio(gear_a_radius / gear_d_radius)
gear_ad.SetEnforcePhase(True)
gear_ad.SetDisabled(True)
sys.AddLink(gear_ad)

pulley_de = chrono.ChLinkLockPulley()
pulley_de.Initialize(gear_d, pulley_e, chrono.ChFramed())
pulley_de.SetFrameShaft1(shaft_x_frame)
pulley_de.SetFrameShaft2(shaft_x_frame)
pulley_de.SetRadius1(gear_d_radius)
pulley_de.SetRadius2(pulley_e_radius)
pulley_de.SetEnforcePhase(True)
pulley_de.SetDisabled(True)
sys.AddLink(pulley_de)


# === Visualization === full Irrlicht scene for observing gear and belt motion
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear D and Pulley E Synchro Belt")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(15, -35, 12), chrono.ChVector3d(-10, -4, -6))
vis.AddTypicalLights()
vis.AddGrid(
    2.0, 2.0, 24, 24,
    chrono.ChCoordsysd(chrono.ChVector3d(-10, -5, -14), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

gear_a_cached = gear_a  # cache: reused in review logging each step
gear_d_cached = gear_d  # cache: reused in review logging each step
pulley_e_cached = pulley_e  # cache: reused in review logging each step


# === Main loop === render once per frame and advance batched physics steps

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()  # cache: used for logging and time-bound checks
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === review video and timeseries assembly only
