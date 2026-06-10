"""PyChrono NSC gear train with two meshed gears on a fixed truss.

The model drives gear A with a prescribed-speed rotational motor and constrains
gear B through a gear link using the requested final radii, truss dimensions,
gear-B offset, and shaft size. The expected behavior is smooth counter-rotation
with both gear shafts held by the fixed truss.
"""

import math as m

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
time_step = 0.001
sim_end = 6.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

radA = 1.5
radB = 3.5
gear_thickness = 0.45
gear_density = 1000
interaxis12 = radA + radB
motor_speed = 3.0
shaft_radius = radA * 0.3
shaft_length = 10.0
tooth_count_A = 24
tooth_count_B = 56
tooth_depth = 0.16
tooth_width = 0.20


# === Helpers ===
def add_gear_teeth(body, radius, count, color):
    """Attach simple visual teeth around a gear body so meshing is visible."""
    for i in range(count):
        angle = 2.0 * m.pi * i / count
        tooth = chrono.ChVisualShapeBox(tooth_width, gear_thickness * 1.08, tooth_depth)
        tooth.SetColor(color)
        tooth_pos = chrono.ChVector3d((radius + tooth_depth * 0.5) * m.cos(angle), 0.0,
                                      (radius + tooth_depth * 0.5) * m.sin(angle))
        tooth_rot = chrono.QuatFromAngleY(-angle)
        body.AddVisualShape(tooth, chrono.ChFramed(tooth_pos, tooth_rot))


def add_rotation_marker(body, radius, color):
    """Attach an off-center face marker to make the 3 rad/s rotation visible."""
    marker = chrono.ChVisualShapeBox(radius * 0.78, 0.08, 0.18)
    marker.SetColor(color)
    body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(radius * 0.42, -gear_thickness * 0.72, 0.0)))


# === System & materials ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
mat = chrono.ChContactMaterialNSC()


# === Bodies ===
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
mbody_truss.SetPos(chrono.ChVector3d(interaxis12 * 0.5, 0, -4.1))
mbody_truss.SetFixed(True)
sys.Add(mbody_truss)

mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, gear_thickness, gear_density)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetInertiaXX(chrono.ChVector3d(8, 8, 8))
mbody_gearA.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.45, 0.9))
add_gear_teeth(mbody_gearA, radA, tooth_count_A, chrono.ChColor(0.12, 0.30, 0.72))
add_rotation_marker(mbody_gearA, radA, chrono.ChColor(1.0, 1.0, 1.0))
sys.Add(mbody_gearA)

mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, gear_thickness, gear_density)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetInertiaXX(chrono.ChVector3d(20, 20, 20))
mbody_gearB.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.45, 0.18))
add_gear_teeth(mbody_gearB, radB, tooth_count_B, chrono.ChColor(0.72, 0.30, 0.10))
add_rotation_marker(mbody_gearB, radB, chrono.ChColor(0.05, 0.05, 0.05))
sys.Add(mbody_gearB)

shaftA = chrono.ChBody()
shaftA.SetMass(1)
shaftA.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
shaftA.SetPos(mbody_gearA.GetPos())
shaftA.SetFixed(True)
shaft_shape_A = chrono.ChVisualShapeCylinder(shaft_radius, shaft_length)
shaft_shape_A.SetColor(chrono.ChColor(0.35, 0.35, 0.35))
shaftA.AddVisualShape(shaft_shape_A, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(shaftA)

shaftB = chrono.ChBody()
shaftB.SetMass(1)
shaftB.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
shaftB.SetPos(mbody_gearB.GetPos())
shaftB.SetFixed(True)
shaft_shape_B = chrono.ChVisualShapeCylinder(shaft_radius, shaft_length)
shaft_shape_B.SetColor(chrono.ChColor(0.35, 0.35, 0.35))
shaftB.AddVisualShape(shaft_shape_B, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(shaftB)


# === Joints / constraints ===
shaft_frame_A = chrono.ChFramed(mbody_gearA.GetPos(), chrono.QuatFromAngleX(chrono.CH_PI_2))
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, shaft_frame_A)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(link_motor)

shaft_frame_B = chrono.ChFramed(mbody_gearB.GetPos(), chrono.QuatFromAngleX(chrono.CH_PI_2))
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_truss, shaft_frame_B)
sys.AddLink(link_revolute)

link_gear = chrono.ChLinkLockGear()
link_gear.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetTransmissionRatio(radA / radB)
link_gear.SetEnforcePhase(True)
sys.AddLink(link_gear)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear train")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8.0, -13.0, 3.2), chrono.ChVector3d(2.5, 0, -1.5))
vis.AddTypicalLights()


# === Main loop ===
gearA = mbody_gearA  # cache: body reused for per-step logging
gearB = mbody_gearB  # cache: body reused for per-step logging


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:
    raise RuntimeError(f"gear simulation failed during stepping: {exc}") from exc
finally:
    pass
